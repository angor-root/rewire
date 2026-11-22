// IMU MPU6050 del chasis (ver imu.h). Registros y escalas iguales a prizm_pro/mpu6050_test (giro +-500 deg/s,
// acel +-4 g, DLPF 44 Hz). Las cuentas del filtro estan en imu_math.h (probadas en test/test_imu.c).
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include "imu.h"
#include "imu_math.h"
#include "net_link.h"

bool imu_ok = false, imu_still = false;
float imu_u[3] = {0, 0, 1}, imu_w[3] = {0, 0, 0}, imu_alin[3] = {0, 0, 0}, imu_psi = 0, imu_aorg[3] = {0, 0, 0};
bool imu_aorg_ok = false;

static const uint8_t ADDR = 0x68;
static const uint32_t PERIOD_US = 5000;   // 200 Hz
static const float TAU_ATT = 0.5f;        // s, filtro complementario (cuanto se confia en el giro frente al acelerometro)
static const float TAU_ALIN = 0.03f;      // s, filtro de la aceleracion (~5 Hz: el servo no sigue mas rapido)
static const float TAU_BIAS = 5.0f;       // s, recalibracion del sesgo con el robot quieto
static float bias[3] = {0, 0, 0};         // sesgo del giro (rad/s, ejes del chasis)
static float aoff[3] = {0, 0, 0};         // offset del acelerometro con el robot nivelado ('E Z')
static int q_mount = 0;                   // hacia donde apunta el eje x del MPU: 0 adelante 1 izq 2 atras 3 der
static float dpsi_acc = 0, still_s = 0;
static uint32_t next_us = 0, last_us = 0, retry_ms = 0, n_err = 0, n_ok = 0;
static int fails = 0;
static uint32_t tel_hz = 0, tel_ms = 0;
static Preferences prefs;
static uint32_t ext_ms = 0;               // ultima 'E X' (IMU de la Pi): mientras llegue, manda sobre el MPU local
static const uint32_t EXT_WD_MS = 100;

static bool wr(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

// lectura cruda en ejes del CHASIS (ver q_mount; shortcut: supone el chip con z hacia arriba, sin montaje de cabeza)
static bool read_raw(float a[3], float w[3]) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(ADDR, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  Wire.readBytes(b, 14);
  int16_t v[7];
  for (int i = 0; i < 7; ++i) v[i] = (int16_t)(b[2 * i] << 8 | b[2 * i + 1]);
  const float ka = IMU_G / 8192.0f, kg = 0.0174533f / 65.5f;
  const float am[2] = {v[0] * ka, v[1] * ka}, wm[2] = {v[4] * kg, v[5] * kg};
  // x_mpu en el chasis = direccion q; y_mpu = z x x_mpu  (inversa de body_to_plate)
  static const int8_t X[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  const int8_t* x = X[q_mount & 3];
  a[0] = am[0] * x[0] - am[1] * x[1];
  a[1] = am[0] * x[1] + am[1] * x[0];
  a[2] = v[2] * ka;
  w[0] = wm[0] * x[0] - wm[1] * x[1];
  w[1] = wm[0] * x[1] + wm[1] * x[0];
  w[2] = v[6] * kg;
  return true;
}

static bool configure() {
  Wire.setClock(400000);  // 14 bytes en ~0.4 ms (a 100 kHz serian ~1.8 ms en cada vuelta de 200 Hz)
  Wire.setTimeOut(5);
  Wire.beginTransmission(ADDR);
  if (Wire.endTransmission() != 0) return false;
  bool ok = wr(0x6B, 0x01);  // despertar, reloj del PLL del giro x
  ok &= wr(0x1B, 0x08);      // giro +-500 deg/s
  ok &= wr(0x1C, 0x08);      // acel +-4 g
  ok &= wr(0x1A, 0x03);      // DLPF 44 Hz
  delay(100);                // el MPU tarda en arrancar tras salir de sleep
  return ok;
}

// promedio de 1 s con el robot quieto: sesgo del giro y acelerometro medio
static bool average(float am[3], float wm[3]) {
  float sa[3] = {0, 0, 0}, sw[3] = {0, 0, 0}, wmax = 0;
  int n = 0;
  for (int i = 0; i < 200; ++i, delay(5)) {
    float a[3], w[3];
    if (!read_raw(a, w)) continue;
    for (int k = 0; k < 3; ++k) { sa[k] += a[k]; sw[k] += w[k]; }
    wmax = fmaxf(wmax, fmaxf(fabsf(w[0]), fmaxf(fabsf(w[1]), fabsf(w[2]))));
    ++n;
  }
  if (n < 150) return false;
  for (int k = 0; k < 3; ++k) { am[k] = sa[k] / n; wm[k] = sw[k] / n; }
  if (wmax - fmaxf(fabsf(wm[0]), fmaxf(fabsf(wm[1]), fabsf(wm[2]))) > 0.05f)
    Serial.println("AVISO E el robot se movio durante la calibracion: repetir 'E B' con el robot quieto");
  return true;
}

static void calib_bias() {
  float am[3], wm[3];
  if (!average(am, wm)) { Serial.println("ERR E calibracion sin lecturas"); return; }
  memcpy(bias, wm, sizeof bias);
  float a[3];
  for (int k = 0; k < 3; ++k) a[k] = am[k] - aoff[k];
  const float n = v3_len(a);
  for (int k = 0; k < 3; ++k) imu_u[k] = a[k] / n;  // arrancar el filtro ya en la inclinacion actual
  Serial.printf("OK E sesgo giro = %.4f %.4f %.4f rad/s\n", bias[0], bias[1], bias[2]);
}

void imu_begin() {
  prefs.begin("imu", false);
  q_mount = prefs.getInt("q", 0);
  aoff[0] = prefs.getFloat("ax", 0);
  aoff[1] = prefs.getFloat("ay", 0);
  aoff[2] = prefs.getFloat("az", 0);
  imu_ok = configure();
  if (!imu_ok) { Serial.println("AVISO E MPU6050 no responde en 0x68 (GPIO 8/9): sigo sin IMU"); return; }
  calib_bias();
  next_us = last_us = micros();
}

float imu_take_dpsi() {
  const float d = dpsi_acc;
  dpsi_acc = 0;
  return d;
}

float imu_pitch() { return att_pitch(imu_u); }
float imu_roll() { return att_roll(imu_u); }

// 'E X ux uy uz wx wy wz ax ay az dpsi [aox aoy aoz]' (Pi, imu_pi.c, ~100 Hz): estado ya filtrado en ejes del chasis;
// dpsi = giro alrededor de la vertical desde la linea anterior (se acumula igual que el del MPU local); ao = aceleracion
// sin gravedad en el origen del chasis (imu_pi desde el 28-oct; sin ella no hay fusion con los encoders)
static void imu_ext(const char* a) {
  float v[13];
  const int n = sscanf(a, "%f %f %f %f %f %f %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6],
                       &v[7], &v[8], &v[9], &v[10], &v[11], &v[12]);
  if (n != 10 && n != 13) return;
  for (int i = 0; i < n; ++i) if (!isfinite(v[i])) return;
  memcpy(imu_u, v, sizeof imu_u);
  memcpy(imu_w, v + 3, sizeof imu_w);
  memcpy(imu_alin, v + 6, sizeof imu_alin);
  dpsi_acc += v[9];
  imu_psi += v[9];
  imu_aorg_ok = n == 13;
  if (imu_aorg_ok) memcpy(imu_aorg, v + 10, sizeof imu_aorg);
  if (!ext_ms || millis() - ext_ms > EXT_WD_MS) Serial.println("OK E IMU de la Pi recibida");
  ext_ms = millis();
  imu_ok = true;
}

void imu_tick() {
  if (ext_ms) {  // IMU de la Pi
    if (millis() - ext_ms <= EXT_WD_MS) return;
    ext_ms = 0;
    imu_ok = false;
    imu_aorg_ok = false;
    dpsi_acc = 0;
    Serial.println("AVISO E sin 'E X' de la Pi (100 ms): dpsi de los encoders");
  }
  const uint32_t now = micros();
  if (!imu_ok) {  // se perdio: reintentar cada 1 s, sin recalibrar (el robot puede estar andando)
    if (millis() - retry_ms > 1000) {
      retry_ms = millis();
      if ((imu_ok = configure())) { fails = 0; next_us = last_us = micros(); Serial.println("OK E MPU6050 de vuelta"); }
    }
    return;
  }
  if ((int32_t)(now - next_us) < 0) return;
  next_us += PERIOD_US;
  if ((int32_t)(now - next_us) > 0) next_us = now + PERIOD_US;  // atrasado: no acumular
  float a[3], wr_[3];
  if (!read_raw(a, wr_)) {
    ++n_err;
    if (++fails >= 5) { imu_ok = false; Serial.println("AVISO E MPU6050 sin respuesta (5 lecturas)"); }
    return;
  }
  fails = 0;
  ++n_ok;
  const float dt = constrain((now - last_us) * 1e-6f, 0.001f, 0.05f);
  last_us = now;
  for (int k = 0; k < 3; ++k) { a[k] -= aoff[k]; imu_w[k] = wr_[k] - bias[k]; }
  // quieto (ruedas paradas y giro casi nulo) por 0.5 s: el giro medido es sesgo
  const bool calm = fabsf(imu_w[0]) < 0.02f && fabsf(imu_w[1]) < 0.02f && fabsf(imu_w[2]) < 0.02f;
  still_s = imu_still && calm ? still_s + dt : 0;
  if (still_s > 0.5f)
    for (int k = 0; k < 3; ++k) { bias[k] += imu_w[k] * dt / TAU_BIAS; imu_w[k] = 0; }
  const float dpsi = att_step(imu_u, imu_w, a, dt, TAU_ATT);
  dpsi_acc += dpsi;
  imu_psi += dpsi;
  const float k = dt / (TAU_ALIN + dt);
  for (int i = 0; i < 3; ++i) imu_alin[i] += k * (a[i] - IMU_G * imu_u[i] - imu_alin[i]);

  if (tel_hz && millis() - tel_ms >= 1000 / tel_hz) {
    tel_ms = millis();
    char m[128];
    snprintf(m, sizeof m, "i,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f", (unsigned long)tel_ms,
             imu_roll() * IMU_RAD2DEG, imu_pitch() * IMU_RAD2DEG, imu_psi * IMU_RAD2DEG, imu_w[0], imu_w[1], imu_w[2],
             imu_alin[0], imu_alin[1], imu_alin[2]);
    Serial.println(m);
    net_send_line(m);
  }
}

void imu_cmd(char* a) {
  while (*a == ' ') ++a;
  const char sub = *a++;
  if (sub == 'X') {
    imu_ext(a);
    return;
  } else if (sub == 'T') {
    tel_hz = strtoul(a, nullptr, 10);
  } else if (sub == 'B') {
    calib_bias();
    return;
  } else if (sub == 'Z') {  // robot QUIETO sobre un piso plano: lo que sobre de (0, 0, g) es offset
    float am[3], wm[3];
    if (!average(am, wm)) { Serial.println("ERR E sin lecturas"); return; }
    aoff[0] = am[0]; aoff[1] = am[1]; aoff[2] = am[2] - IMU_G;
    prefs.putFloat("ax", aoff[0]); prefs.putFloat("ay", aoff[1]); prefs.putFloat("az", aoff[2]);
    memcpy(bias, wm, sizeof bias);
    imu_u[0] = imu_u[1] = 0; imu_u[2] = 1;
  } else if (sub == 'Q') {
    q_mount = atoi(a) & 3;
    prefs.putInt("q", q_mount);
  } else if (sub == '0') {
    imu_psi = 0;
  } else if (sub != '?') {
    Serial.println("ERR E <?|T hz|B|Z|Q 0-3|0>");
    return;
  }
  Serial.printf("OK E fuente=%s ok=%d lecturas=%lu err=%lu q=%d roll=%.2f pitch=%.2f psi=%.2f deg | sesgo=%.4f,%.4f,%.4f "
                "offset=%.3f,%.3f,%.3f | quieto=%d tel=%lu Hz\n", ext_ms ? "Pi" : "MPU local", imu_ok, (unsigned long)n_ok, (unsigned long)n_err,
                q_mount, imu_roll() * IMU_RAD2DEG, imu_pitch() * IMU_RAD2DEG, imu_psi * IMU_RAD2DEG, bias[0], bias[1],
                bias[2], aoff[0], aoff[1], aoff[2], imu_still, (unsigned long)tel_hz);
}
