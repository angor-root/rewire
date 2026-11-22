/*
 * imu_pi — IMU MPU6050 del chasis en la Raspberry Pi -> PRIZM por USB (28-oct).
 * La IMU no funciona en el puerto I2C de la PRIZM (alimentacion), en la Pi si: SDA pin 3, SCL pin 5, VCC pin 2 (5 V),
 * GND pin 6. Lee a 200 Hz, filtra con prizm_pro/gimbal_fw/imu_math.h (el mismo filtro que el firmware) y manda a la
 * PRIZM a 100 Hz:
 *   "E X ux uy uz wx wy wz ax ay az dpsi aox aoy aoz\n"   vertical, giro (rad/s, sin sesgo), aceleracion sin gravedad
 *                                             (m/s2) en el pivote del plato, en ejes del chasis (x adelante, y izquierda,
 *                                             z arriba), giro alrededor de la vertical desde la linea anterior (rad) y
 *                                             aceleracion sin gravedad en el ORIGEN (filtro de 10 ms, la que la PRIZM
 *                                             fusiona con los encoders; 28-oct)
 * Escribe en el mismo USB que ball_pixel: cada linea sale en un solo write() y el driver tty no las mezcla.
 *
 * Uso:   ~/imu_pi [--puerto /dev/ttyACM0] [--q 0-3] [--udp IP:PUERTO]
 *        q: hacia donde apunta la x del MPU (0 adelante 1 izq ...); --udp: copia de cada linea para ver en RViz
 *        (sim_kanayama/imu_rviz.launch.py en la laptop, puerto 5006)
 *        --r_imu x y z / --r_plato x y z: posicion (m) de la IMU y del pivote del plato en ejes del CHASIS (imu_math.h:
 *        origen en el medio del eje de las ruedas actuadas, en el piso; x hacia las ruedas actuadas, y izquierda, z
 *        arriba). Sin ellas (0 0 0) no se corrige el brazo de palanca: con la IMU de costado, al girar en el sitio
 *        aparece una centripeta w^2 r falsa (r 0.1 m, w 1 rad/s: 0.1 m/s2 ~ 0.6 grados de inclinacion falsa).
 *        ~/imu_pi --nivel     robot QUIETO en piso PLANO: guarda la calibracion en ~/imu_calib.txt y sale
 * Al arrancar mide el sesgo del giro durante 1 s: robot QUIETO.
 * Log (28-oct): kill -USR1 -> empieza /tmp/imu_log.csv (100 Hz, t = CLOCK_MONOTONIC, el mismo reloj que ball_pixel),
 *               kill -USR2 -> lo cierra. Columnas: t,roll,pitch,psi (grados),wx,wy,wz (rad/s),ax,ay,az (m/s2, sin
 *               gravedad, en el pivote del plato),ux,uy,uz. deploy/tools/sesion.py lo usa.
 * Compilar en la Pi:  gcc -O2 -Wall -I. -o ~/imu_pi imu_pi.c -lm   (con imu_math.h en la misma carpeta)
 */
#include <fcntl.h>
#include <glob.h>
#include <arpa/inet.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include "imu_math.h"

static int i2c = -1;
static volatile sig_atomic_t log_req = 0;  // 1 abrir, 2 cerrar (lo piden las senales, lo hace el lazo)
static void on_sig(int s) { log_req = s == SIGUSR1 ? 1 : 2; }
static int q_mount = 0;
static float cal_s = 1, cal_off[3] = {0, 0, 0};  // a_chasis = cal_s * (a_cruda - cal_off)

static int wr(uint8_t reg, uint8_t v) {
  const uint8_t b[2] = {reg, v};
  return write(i2c, b, 2) == 2;
}
static int rd(uint8_t reg, uint8_t* buf, int n) { return write(i2c, &reg, 1) == 1 && read(i2c, buf, n) == n; }

static double now_s(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}

static int mpu_open(void) {
  if (i2c >= 0) close(i2c);
  i2c = open("/dev/i2c-1", O_RDWR);
  if (i2c < 0 || ioctl(i2c, I2C_SLAVE, 0x68) < 0) return 0;
  const int ok = wr(0x6B, 0x01) & wr(0x1B, 0x08) & wr(0x1C, 0x08) & wr(0x1A, 0x03);  // PLL, +-500 deg/s, +-4 g, 44 Hz
  usleep(100000);
  return ok;
}

// lectura cruda en ejes del chasis, sin calibrar (shortcut: chip con z hacia arriba, sin montaje de cabeza)
static int mpu_read(float a[3], float w[3]) {
  uint8_t b[14];
  if (!rd(0x3B, b, 14)) return 0;
  int16_t v[7];
  for (int i = 0; i < 7; ++i) v[i] = (int16_t)(b[2 * i] << 8 | b[2 * i + 1]);
  const float ka = IMU_G / 8192.0f, kg = 0.0174533f / 65.5f;
  static const int X[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};  // x del MPU en el chasis; y_mpu = z x x_mpu
  const int* x = X[q_mount & 3];
  const float am[2] = {v[0] * ka, v[1] * ka}, wm[2] = {v[4] * kg, v[5] * kg};
  a[0] = am[0] * x[0] - am[1] * x[1];
  a[1] = am[0] * x[1] + am[1] * x[0];
  a[2] = v[2] * ka;
  w[0] = wm[0] * x[0] - wm[1] * x[1];
  w[1] = wm[0] * x[1] + wm[1] * x[0];
  w[2] = v[6] * kg;
  return 1;
}

// promedio de 'secs' con el robot quieto; devuelve el giro maximo para detectar si se movio
static float average(float secs, float am[3], float wm[3]) {
  float sa[3] = {0}, sw[3] = {0}, wmax = 0;
  int n = 0;
  for (double t0 = now_s(); now_s() - t0 < secs; usleep(5000)) {
    float a[3], w[3];
    if (!mpu_read(a, w)) continue;
    for (int k = 0; k < 3; ++k) { sa[k] += a[k]; sw[k] += w[k]; }
    for (int k = 0; k < 3; ++k) if (fabsf(w[k]) > wmax) wmax = fabsf(w[k]);
    ++n;
  }
  for (int k = 0; k < 3; ++k) { am[k] = n ? sa[k] / n : 0; wm[k] = n ? sw[k] / n : 0; }
  return n ? wmax - fmaxf(fabsf(wm[0]), fmaxf(fabsf(wm[1]), fabsf(wm[2]))) : 1e9f;
}

static const char* calib_path(void) {
  static char p[256];
  snprintf(p, sizeof p, "%s/imu_calib.txt", getenv("HOME") ? getenv("HOME") : ".");
  return p;
}

static int open_prizm(const char* want) {
  char path[128];
  if (want) snprintf(path, sizeof path, "%s", want);
  else {  // como ball_pixel: el primer /dev/ttyACM* (la PRIZM es lo unico por USB-CDC)
    glob_t g;
    if (glob("/dev/ttyACM*", 0, NULL, &g) != 0 || !g.gl_pathc) return -1;
    snprintf(path, sizeof path, "%s", g.gl_pathv[0]);
    globfree(&g);
  }
  const int fd = open(path, O_WRONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  struct termios t;
  if (tcgetattr(fd, &t) == 0) { cfmakeraw(&t); cfsetspeed(&t, B115200); tcsetattr(fd, TCSANOW, &t); }
  fprintf(stderr, "PRIZM en %s\n", path);
  return fd;
}

int main(int argc, char** argv) {
  const char* puerto = NULL;
  int nivel = 0, q_arg = -1, udp = -1;
  float r_imu[3] = {0, 0, 0}, r_plato[3] = {0, 0, 0};
  struct sockaddr_in dst = {0};
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--puerto") && i + 1 < argc) puerto = argv[++i];
    else if (!strcmp(argv[i], "--q") && i + 1 < argc) q_arg = atoi(argv[++i]) & 3;
    else if (!strcmp(argv[i], "--nivel")) nivel = 1;
    else if ((!strcmp(argv[i], "--r_imu") || !strcmp(argv[i], "--r_plato")) && i + 3 < argc) {
      float* r = !strcmp(argv[i], "--r_imu") ? r_imu : r_plato;
      for (int k = 0; k < 3; ++k) r[k] = atof(argv[i + 1 + k]);
      i += 3;
    }
    else if (!strcmp(argv[i], "--udp") && i + 1 < argc) {
      char ip[64];
      int port;
      if (sscanf(argv[++i], "%63[^:]:%d", ip, &port) != 2 || inet_pton(AF_INET, ip, &dst.sin_addr) != 1) {
        fprintf(stderr, "--udp IP:PUERTO\n");
        return 2;
      }
      dst.sin_family = AF_INET;
      dst.sin_port = htons(port);
      udp = socket(AF_INET, SOCK_DGRAM, 0);
    }
    else { fprintf(stderr, "uso: imu_pi [--puerto /dev/ttyACMx] [--q 0-3] [--nivel]\n"); return 2; }
  }
  FILE* f = fopen(calib_path(), "r");
  if (f) {
    if (fscanf(f, "%d %f %f %f %f", &q_mount, &cal_s, &cal_off[0], &cal_off[1], &cal_off[2]) != 5) {
      q_mount = 0; cal_s = 1; cal_off[0] = cal_off[1] = cal_off[2] = 0;
      fprintf(stderr, "AVISO %s ilegible: sin calibracion\n", calib_path());
    }
    fclose(f);
  } else if (!nivel) fprintf(stderr, "AVISO sin %s: correr 'imu_pi --nivel' en piso plano\n", calib_path());
  if (q_arg >= 0) q_mount = q_arg;
  if (!mpu_open()) { fprintf(stderr, "ERROR el MPU6050 no responde en /dev/i2c-1 0x68\n"); return 1; }

  float am[3], wm[3];
  if (nivel) {
    // La magnitud medida no da g (~1.14 g en este modulo, parecida en distintas orientaciones: error de ESCALA comun):
    // se corrige la escala con |a| y lo que no esta en z con el offset (supone el piso plano).
    if (average(2.0f, am, wm) > 0.05f) { fprintf(stderr, "ERROR el robot se movio: repetir\n"); return 1; }
    const float n = v3_len(am);
    cal_s = IMU_G / n;
    cal_off[0] = am[0]; cal_off[1] = am[1]; cal_off[2] = am[2] - n;
    f = fopen(calib_path(), "w");
    if (!f) { perror(calib_path()); return 1; }
    fprintf(f, "%d %.6f %.5f %.5f %.5f\n", q_mount, cal_s, cal_off[0], cal_off[1], cal_off[2]);
    fclose(f);
    printf("OK nivel: |a| cruda = %.3f m/s2 -> escala %.4f, offset %.3f %.3f %.3f (inclinacion cruda %.2f deg) -> %s\n", n,
           cal_s, cal_off[0], cal_off[1], cal_off[2], acosf(am[2] / n) * IMU_RAD2DEG, calib_path());
    return 0;
  }

  float bias[3], u[3];
  if (average(1.0f, am, bias) > 0.05f) fprintf(stderr, "AVISO el robot se movio al medir el sesgo del giro\n");
  for (int k = 0; k < 3; ++k) am[k] = cal_s * (am[k] - cal_off[k]);
  for (int k = 0; k < 3; ++k) u[k] = am[k] / v3_len(am);
  fprintf(stderr, "sesgo del giro %.4f %.4f %.4f rad/s | calibracion q=%d s=%.4f\n", bias[0], bias[1], bias[2], q_mount, cal_s);

  int ser = open_prizm(puerto);
  FILE* lg = NULL;
  signal(SIGUSR1, on_sig);
  signal(SIGUSR2, on_sig);
  float alin[3] = {0, 0, 0}, aorg[3] = {0, 0, 0}, dpsi = 0, psi = 0, still_s = 0, w_prev[3] = {0, 0, 0}, dw[3] = {0, 0, 0};
  float to_origin[3], to_plate[3];
  for (int k = 0; k < 3; ++k) { to_origin[k] = -r_imu[k]; to_plate[k] = r_plato[k] - r_imu[k]; }
  fprintf(stderr, "r_imu %.3f %.3f %.3f  r_plato %.3f %.3f %.3f m\n", r_imu[0], r_imu[1], r_imu[2], r_plato[0],
          r_plato[1], r_plato[2]);
  double next = now_s(), last = next, t_retry = 0, t_stat = next;
  int n = 0, nerr = 0, nsent = 0, fails = 0;
  for (;;) {
    next += 0.005;
    const double sl = next - now_s();
    if (sl > 0) usleep((useconds_t)(sl * 1e6));
    else if (sl < -0.05) next = now_s();  // muy atrasado: no acumular
    float a[3], w[3];
    if (!mpu_read(a, w)) {
      ++nerr;
      if (++fails >= 20) { fprintf(stderr, "AVISO MPU sin respuesta: reabriendo\n"); mpu_open(); fails = 0; }
      continue;
    }
    fails = 0;
    const double t = now_s();
    const float dt = (float)fmin(fmax(t - last, 0.001), 0.05);
    last = t;
    for (int k = 0; k < 3; ++k) { a[k] = cal_s * (a[k] - cal_off[k]); w[k] -= bias[k]; }
    // shortcut: "quieto" solo con la IMU (giro < 0.02 rad/s por 1 s); un giro real mas lento que eso se toma como sesgo.
    // Si molesta, que la PRIZM (que ve las ruedas) diga cuando esta quieto.
    const int calm = fabsf(w[0]) < 0.02f && fabsf(w[1]) < 0.02f && fabsf(w[2]) < 0.02f &&
                     fabsf(v3_len(a) - IMU_G) < 0.3f;
    still_s = calm ? still_s + dt : 0;
    if (still_s > 1.0f) for (int k = 0; k < 3; ++k) { bias[k] += w[k] * dt / 10.0f; w[k] = 0; }
    const float kd = dt / (0.02f + dt);  // aceleracion angular filtrada (la derivada del giro es ruidosa)
    for (int k = 0; k < 3; ++k) { dw[k] += kd * ((w[k] - w_prev[k]) / dt - dw[k]); w_prev[k] = w[k]; }
    float a_o[3], a_p[3];
    lever_arm(a, w, dw, to_origin, a_o);  // el filtro usa la medida en el origen (sin la centripeta del costado)
    lever_arm(a, w, dw, to_plate, a_p);   // la compensacion del plato, la del pivote del plato
    const float d = att_step(u, w, a_o, dt, 0.5f);
    dpsi += d;
    psi += d;
    const float kl = dt / (0.03f + dt);
    for (int k = 0; k < 3; ++k) alin[k] += kl * (a_p[k] - IMU_G * u[k] - alin[k]);
    const float ko = dt / (0.01f + dt);  // para la odometria: poco filtro, la PRIZM compara con los encoders a 100 Hz
    for (int k = 0; k < 3; ++k) aorg[k] += ko * (a_o[k] - IMU_G * u[k] - aorg[k]);

    if (++n % 2) continue;  // a la PRIZM a 100 Hz
    if (log_req) {
      if (lg) fclose(lg);
      lg = NULL;
      if (log_req == 1 && (lg = fopen("/tmp/imu_log.csv", "w")))
        fprintf(lg, "t,roll,pitch,psi,wx,wy,wz,ax,ay,az,ux,uy,uz\n");
      log_req = 0;
    }
    if (lg)
      fprintf(lg, "%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f\n", t, att_roll(u) * IMU_RAD2DEG,
              att_pitch(u) * IMU_RAD2DEG, psi * IMU_RAD2DEG, w[0], w[1], w[2], alin[0], alin[1], alin[2], u[0], u[1], u[2]);
    char buf[200];
    const int k = snprintf(buf, sizeof buf, "E X %.5f %.5f %.5f %.4f %.4f %.4f %.3f %.3f %.3f %.6f %.3f %.3f %.3f\n", u[0],
                           u[1], u[2], w[0], w[1], w[2], alin[0], alin[1], alin[2], dpsi, aorg[0], aorg[1], aorg[2]);
    if (udp >= 0) sendto(udp, buf, k, 0, (struct sockaddr*)&dst, sizeof dst);  // la laptop integra dpsi por su cuenta
    if (ser < 0) {
      if (t - t_retry > 1.0) { t_retry = t; ser = open_prizm(puerto); }
      dpsi = 0;  // sin PRIZM no se acumula (al volver, el myRIO no debe recibir un salto)
    } else {
      const ssize_t r = write(ser, buf, k);
      if (r == k) { dpsi = 0; ++nsent; }
      else if (r < 0) { fprintf(stderr, "AVISO se perdio la PRIZM: reabriendo\n"); close(ser); ser = -1; }
    }
    if (t - t_stat >= 1.0) {
      printf("imu %.0f Hz err=%d enviadas=%d | roll %.2f pitch %.2f psi %.1f deg | w %.3f %.3f %.3f | alin %.2f %.2f %.2f\n",
             n / (t - t_stat), nerr, nsent, att_roll(u) * IMU_RAD2DEG, att_pitch(u) * IMU_RAD2DEG, psi * IMU_RAD2DEG,
             w[0], w[1], w[2], alin[0], alin[1], alin[2]);
      fflush(stdout);
      t_stat = t;
      n = nerr = nsent = 0;
    }
  }
}
