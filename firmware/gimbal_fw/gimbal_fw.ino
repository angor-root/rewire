/*
 * gimbal_fw — control de bajo nivel del gimbal de 2 GDL (MT5001 / DSM)
 * Placa: TETRIX PRIZM Pro (ESP32-S3). Libreria: TETRIX_PRIZM_PRO v1.15.
 *
 * Conexion:
 *   Servo eje alfa (inclinacion en X del plato) -> puerto de servo 1
 *   Servo eje beta (inclinacion en Y del plato) -> puerto de servo 2
 *   Cable negro hacia el lado con muesca (el mas cercano a los puertos de motor DC).
 *   Bateria TETRIX 12 V conectada y switch encendido (los servos se alimentan de ahi).
 *
 * Arranque: al encender, la PRIZM espera el BOTON VERDE (Start) antes de
 * habilitar los servos (lo impone PrizmBegin; se usa como paso de armado).
 *
 * Protocolo por USB (CDC, /dev/ttyACM0), lineas ASCII terminadas en '\n':
 *   S <alfa> <beta>        consigna de inclinacion en grados (relativa a plato nivelado)
 *   P <canal> <raw>        posicion cruda del servo (0-180), para calibrar
 *   C <n1> <n2> <s1> <s2> <g1> <g2> <lim>
 *                          calibracion: neutro crudo (plato nivelado), signo (+1/-1),
 *                          ganancia (grados servo por grado de plato), limite |grados plato|
 *   W <ms>                 watchdog: sin 'S' en <ms> -> vuelve a nivelado (0 = desactivado)
 *   F <hz>                 frecuencia de telemetria (0 = apagada)
 *   ?                      imprime configuracion
 *   M <p1> <p2> <p3> <p4>  potencia de motores DC 1-4 (-100..100). Watchdog propio:
 *                          sin otro 'M' en motor_wd_ms (300) -> motores a 0
 *   V <w1> <w2>            DESPLIEGUE: consigna de velocidad de ruedas 1-2 en rad/s (+ = adelante). Arranca el PI
 *                          de 100 Hz (el mismo que CLPI con ventana 2, tau_m 40 ms: lo que replica la sim) sin
 *                          bloquear. Responde "v,<ms>,<w1>,<w2>,<u1>,<u2>,<c1>,<c2>" (velocidad medida en rad/s,
 *                          potencia, cuentas de encoder del ultimo paso del PI).
 *                          Watchdog: sin otro 'V' en vel_wd_ms (200) -> motores a 0. 'M', 'X' o 'I' lo cancelan.
 *   Y <ms>                 watchdog del modo 'V'
 *   --- POLITICA A BORDO (05-oct). Nucleo 0: solo calcula la red a 30 Hz (timer). Nucleo 1 (loop): todo el
 *       I/O (serie, I2C de motores/servos, PI de 100 Hz). La tarea de la politica nunca toca el I2C ni el Serial.
 *   O <seq> <bx> <by> <vis> <pitch> <roll> <gx> <gy> <gz> <vx> <wz>
 *                          entradas de la politica que no ve la PRIZM (vision de la RPi, IMU, comando):
 *                          pelota en m (frame del plato), visible 0/1, angulos del plato en rad, gravedad
 *                          proyectada unitaria, comando vx (m/s) y wz (rad/s). Se guarda la ultima.
 *   A 1 | A 0              arranca / detiene la politica. Con A 1 la red manda las ruedas (por el PI de 'V')
 *                          y el plato ('S'). Watchdog: sin 'O' en 200 ms, o inclinacion > 0.8 rad -> para todo.
 *                          Responde por paso: "a,<ms>,<k>,<seq>,<inf_us>,<w1>,<w2>,<a0>,<a1>,<a2>,<a3>"
 *                          (velocidades de rueda usadas en la obs y accion cruda de la red, AMBAS en la convencion
 *                          de la sim: rueda + = hacia ATRAS; ver WHEEL_SIGN).
 *   Q                      prueba de paridad (16 vectores de onnxruntime) y tiempo de inferencia, en el nucleo 1
 *   G                      estadisticas de tiempo real (politica y PI de 100 Hz) y las reinicia
 *   --- BASE (12-oct): cinematica diferencial + odometria de encoders + trayectorias, sobre el PI de 'V' (base_kin.c)
 *   U V <v> <w>            velocidad del cuerpo: v en m/s (+ adelante), w en rad/s (+ antihorario). Watchdog 300 ms.
 *   U C <R> <v> [dir] [vueltas]
 *                          circulo de radio R (m) a v (m/s), tangente a la pose actual. dir +1 = antihorario
 *                          (centro a la izquierda), -1 = horario. vueltas 0 = sin fin. Rampa de 0.25 m/s^2.
 *   U F                    termina el circulo frenando con la rampa       U S   parada inmediata de la base
 *   U O                    odometria a cero                               U ?   estado
 *   U G <r> <via>          geometria (m): radio de rueda y via efectiva (calibrar con un giro en el sitio)
 *   U K <kx> <ky> <kth>    ganancias del seguimiento (Kanayama)
 *   U T <hz>               telemetria de la base (0 = apagada). Mientras la base esta activa responde:
 *                          "b,<ms>,<modo>,<x>,<y>,<th>,<x_ref>,<y_ref>,<th_ref>,<v>,<w>,<wL_med>,<wR_med>"
 *   --- GAMEPAD TETRIX (receptor de 4 canales en los puertos de sensor 2 y 3, rc_input.cpp)
 *   J 1 | J 0              habilita / deshabilita el gamepad
 *   J R [s]                imprime los pulsos crudos de los 4 canales a 10 Hz durante s segundos (default 10)
 *   J C                    calibra los centros (palancas sueltas)
 *   J M <v> <w> <modo>     que canal es cada eje, con signo: "J M 3 -1 4" = v del canal 3, w del canal 1 invertido,
 *                          gestos con el canal 4
 *   J L <vmax> <wmax> <R> <v_circ> <vueltas>   limites de teleoperacion y circulo del gesto
 *   J ?                    configuracion y estado de los canales
 *   Gamepad: palanca de v adelante/atras + palanca de w izquierda/derecha = manejo directo.
 *            Palanca de MODO a fondo 1 s (las otras sueltas): derecha = circulo antihorario, izquierda = horario.
 *            Durante el circulo: mover v o w lo ABORTA (parada inmediata); modo a fondo 1 s = termina con rampa.
 *            Con la politica activa (A 1) las palancas dan el comando vx / wz de la red en vez de las ruedas.
 *            Sin pulsos del receptor (> 100 ms) -> se para lo que el gamepad haya arrancado.
 *   --- MODO DSM (16-oct): myRIO por UART en texto (Serial1, puerto de sensor 1: GPIO 6 = RX, GPIO 7 = TX, 115200 8N1)
 *       myRIO -> PRIZM:  "G gx gy gz wR_ref wL_ref\r\n"   (gravedad normalizada + consignas de rueda en rad/s)
 *       PRIZM -> myRIO:  "G dN_R dN_L run dpsi pitch roll\r\n"  una por cada linea valida; dN = cuentas desde la
 *                          respuesta anterior; dpsi = giro alrededor de la vertical desde la respuesta anterior (rad,
 *                          + = izquierda, del MPU6050; sin MPU sale de los encoders con U G); pitch (+ nariz arriba) y
 *                          roll (+ lado izquierdo arriba) del chasis en rad (0 sin MPU). 27-oct.
 *                          run = 1 si el control esta activo, 0 si esta parada ('D 0', watchdog, inclinacion): con 0 el
 *                          VI debe reiniciar t y su odometria, asi al rearmar la referencia arranca donde esta el robot
 *                          (+ = adelante, 1440 por vuelta). Las consignas van directo al PI de 100 Hz del modo 'V'.
 *       ARMADO al encender: arranca con la primera linea con consignas (y vuelve a arrancar si se corto el myRIO).
 *       Paradas: sin lineas en 200 ms, inclinacion > 0.8 rad, linea sin consignas, 'D 0' o cualquier otro modo
 *       (estos desarman: volver con 'D 1').
 *   D 1 | D 0              arma / para y desarma
 *   D Z                    encoders a cero (con las ruedas paradas)
 *   D L <wmax> <wd_ms>     limite de consigna (rad/s, default 12; proporcional, conserva el giro) y watchdog (200 ms)
 *   D T <hz>               telemetria por USB con el control activo (default 10 Hz, 0 = apagada):
 *                          "d,<ms>,<wref_der>,<wref_izq>,<w_der>,<w_izq>,<cuentas_der>,<cuentas_izq>"
 *   D T L                  LOG (19-oct): una fila por cada linea del myRIO (lo que usa dsm_log.py; 'D T 10' vuelve):
 *                          "L,<ms>,<n>,<run>,<gx>,<gy>,<gz>,<wref_rx_der>,<wref_rx_izq>,<wref_der>,<wref_izq>,
 *                           <w_der>,<w_izq>,<u_der>,<u_izq>,<cuentas_der>,<cuentas_izq>,<dN_der>,<dN_izq>,
 *                           <x>,<y>,<th>,<nx>,<x1>..<xn>"
 *                          wref_rx = la recibida, wref = la aplicada (tras D L), u = potencia del PI (%),
 *                          x y th = odometria de la PRIZM (U G, a cero al arrancar DSM o con D O).
 *                          Campos EXTRA: el myRIO puede agregar hasta 12 numeros al final de su linea
 *                          ("G gx gy gz wR wL ex ey eth ..."): no cambian el control, solo salen en el log.
 *   D O                    odometria de la PRIZM a cero
 *   D A <rad/s2>           RAMPA (28-oct): las consignas del myRIO llegan al PI con aceleracion de rueda limitada (las
 *                          dos ruedas por el mismo factor: se conserva la curvatura). 0 = escalon como antes. Las
 *                          paradas (watchdog, D 0, inclinacion) siguen siendo inmediatas. Queda en la flash.
 *   D F 1 | D F 0          FUSION (28-oct, odo_ekf.h): los dN al myRIO salen de la odometria fusionada encoders + IMU de
 *                          la Pi (rumbo del giroscopio, avance de los encoders salvo "sin contacto": en el aire o
 *                          patinando, avance de la IMU). El VI no cambia. Sin 'E X' con aceleracion -> dN crudos.
 *                          El filtro corre siempre que hay IMU (D E y el log lo muestran); D F solo elige que dN salen.
 *                          Queda en la flash.
 *   D P 0 | D P 1          (28-oct noche) dpsi de la respuesta al myRIO: 0 = de los ENCODERS (default), 1 = del giroscopio
 *                          de la Pi. Queda en la flash.
 *   D E                    estado del filtro: v_enc, v fusionada, sesgo, contacto, rechazos
 *                          Con 'D T L' cada fila L va seguida de "F,<ms>,<v_enc>,<w_enc>,<w_gyro>,<vx>,<vy>,<ba>,
 *                          <rej_v>,<rej_w>,<sin_contacto>,<nis_win>,<perdidas>,<dN_crudo_der>,<dN_crudo_izq>"
 *   --- PELOTA CON LA CAMARA (20-oct): PD sobre la pelota que manda la Pi en 'O' -> IK del plato de rotulas -> servos
 *   H 1 | H 0              activa el control / lo apaga y nivela       H ?   configuracion
 *   H K <kp> <kd> [ki] [lim]  ganancias (grados de plato por m, por m/s, por m*s) y limite (grados, default 10)
 *   H S <sx> <sy> [swap]   signos de x / y de la camara y si estan intercambiados (ver pruebas con H T)
 *   H T <roll> <pitch>     inclinacion manual con la IK (grados de plato): para probar signos
 *   H R <pA> <n1> <n2> <s1> <s2>  calibracion de rotulas: puerto del servo A, neutro y signo por puerto
 *                          Responde por cada 'O': "h,<ms>,<x>,<y>,<vx>,<vy>,<roll>,<pitch>,<roll_out>,<pitch_out>"
 *                          (roll/pitch = lo que pide el PD; *_out = lo que va a la IK tras la compensacion)
 *   H C <kg> <ka> <kw> [tau]  COMPENSACION con la IMU (27-oct), cada una de 0 (apagada, default) a 1:
 *                          kg inclinacion del chasis, ka aceleracion del carro, kw giro (centrifuga + Coriolis);
 *                          tau = adelanto con el giroscopio (s, ~retardo del servo, default 0). Se aplica a 100 Hz.
 *   H Y <q>                hacia donde apunta el eje X del plato en el chasis: 0 adelante 1 izq 2 atras 3 der
 *   --- IMU MPU6050 del chasis (27-oct, imu.cpp; puerto I2C de expansion, SDA 8 / SCL 9). Al encender mide el sesgo
 *       del giro durante 1 s tras el boton verde: robot QUIETO.
 *   E ?                    estado        E T <hz>   telemetria "i,<ms>,<roll>,<pitch>,<psi> (grados),<wx>,<wy>,<wz>
 *                          (rad/s),<ax>,<ay>,<az> (m/s2, sin gravedad)" (0 = apagada)
 *   E B                    recalibrar el sesgo del giro (quieto 1 s)    E 0   rumbo psi a cero
 *   E Z                    robot quieto en piso PLANO: guarda el offset del acelerometro (queda en la flash)
 *   E Q <q>                hacia donde apunta el eje x del MPU: 0 adelante 1 izq 2 atras 3 der (queda en la flash)
 *   E X <ux uy uz wx wy wz ax ay az dpsi>  IMU DE LA PI (28-oct, deploy/c/imu_pi.c, ~100 Hz por este USB): vertical,
 *                          giro (rad/s), aceleracion sin gravedad (m/s2) en ejes del chasis y giro alrededor de la vertical
 *                          desde la linea anterior. Manda sobre el MPU local; sin lineas en 100 ms -> dpsi de encoders.
 *   H T con H C != 0: lo que pide ball_pixel + la compensacion (100 Hz, sigue 200 ms despues de la ultima 'H T').
 *                          H C y H Y quedan en la flash.
 *   --- WiFi (20-oct, net_link.cpp): log y comandos por UDP (puerto 5005) para dsm_gui.py sin cable
 *   N S <red> <clave>      guarda la red en la flash y la activa      N 1 | N 0   activa / apaga    N ?   estado e IP
 *   D R [s]                muestra cada linea recibida ("rx") y su respuesta ("tx") durante s segundos (default 1)
 *   D ?                    estado: lineas ok / err / sin LF, Hz, acelerometro, ultima consigna
 *   X                      parada inmediata de todos los motores
 *   Z                      encoders a cero
 *   B                      benchmark: tiempo de una lectura de encoder y de una escritura de potencia (us)
 *   I <tipo> <motor> <hz> <a1> <a2> <a3> <a4>
 *                          experimento de identificacion en LAZO ABIERTO, ejecutado por el firmware
 *                          a periodo fijo (micros). motor: 1, 2 o 3 (=ambos, misma entrada).
 *                          tipos:
 *                            STEP  a1=u  a2=ms en u  (0 durante 400 ms antes, 800 ms despues)
 *                            STAIR a1=umax a2=paso a3=ms por escalon (0 -> +umax -> -umax -> 0)
 *                            PRBS  a1=u0 a2=du a3=ms por bit a4=bits  (u0 +- du, LFSR de 9 bits)
 *                          Salida: "D,<k>,<t_us>,<u1>,<u2>,<enc1>,<enc2>,<lectura_us>" por muestra,
 *                          "IBEGIN ..." al inicio y "IEND <muestras> <atrasos>" al final. 'X' aborta.
 *                          Lazo cerrado de velocidad (perfil de referencia fijo, ver ref_profile()):
 *                            CLPI  PI propio a <hz>, a1 = muestras de la ventana de velocidad (2),
 *                                  a2 = tau del modelo de referencia en ms (0 = PI de 1 GDL). Con a2 > 0
 *                                  el error se calcula contra la referencia filtrada 1/(tau_m s + 1)
 *                                  (2 GDL): el feedforward sigue usando la referencia cruda.
 *                                  a3 = separacion de la integral en rad/s: solo se integra si |e| < a3
 *                                  (0 = siempre). Evita que el integrador se cargue mientras la planta
 *                                  no puede seguir (frenado sin invertir: solo friccion).
 *                            CLINT PID interno de la PRIZM (setMotorSpeeds, grados/s)
 *                            CHIRP barrido de frecuencias de la CONSIGNA de velocidad sobre el MISMO PI que el modo 'V'
 *                                  (100 Hz recomendado, ventana 2, tau_m 40 ms, sin separacion de la integral):
 *                                  ref = a5 + a1*sin(fase), frecuencia LINEAL de a2 a a3 Hz en a4 s (como PACE:
 *                                  mismo tiempo en cada banda, asi las altas frecuencias pesan igual en el ajuste).
 *                                  Perfil: 0.5 s en 0, 1 s en a5, barrido, 0.5 s en a5, 0.5 s en 0. |ref| <= 12 rad/s.
 *                                  Ej.: I CHIRP 3 100 3 0.1 12 40 0   (amplitud 3 rad/s, 0.1-12 Hz, 40 s, offset 0)
 *                          Salida: "C,<k>,<t_us>,<ref_rad_s>,<u1>,<u2>,<enc1>,<enc2>,<w1>,<w2>,<ciclo_us>"
 *   K <m> <kp> <ki> <g+> <dz+> <g-> <dz->
 *                          ganancias del PI propio del motor m (1|2): kp [%/(rad/s)], ki [%/(rad/s)/s],
 *                          feedforward: ganancia estatica g [rad/s por %] y zona muerta dz [%] por lado
 * Respuestas:
 *   T,<ms>,<alfa_cmd>,<beta_cmd>,<raw1>,<raw2>,<vbat>,<edad_cmd_ms>,<estado>
 *   E,<ms>,<p1>,<p2>,<p3>,<p4>,<enc1>,<enc2>,<enc3>,<enc4>   (potencias y cuentas de encoder)
 *   OK ... / ERR ...
 *
 * Nota: el chip de servos de la PRIZM recibe la posicion como ENTERO en
 * grados (0-180): la resolucion minima es 1 grado de servo.
 */
#include <PRIZM_PRO.h>
#include "experiment.h"
#include "policy_mcu.h"
#include "policy_weights.h"
#include "base_kin.h"
#include "rc_input.h"
#include "net_link.h"
#include "imu.h"
#include "imu_math.h"
#include "odo_ekf.h"
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>

PRIZM prizm;

// ---------------- configuracion (modificable por 'C', 'W', 'F') ----------------
struct Axis {
  float neutral = 90.0f;  // posicion cruda con el plato nivelado
  float sign = 1.0f;      // +1 / -1 segun como quedo montado el servo
  float gain = 1.0f;      // grados de servo por grado de plato (depende del varillaje)
};
Axis axis[2];
float limit_deg = 15.0f;       // limite de inclinacion del plato (seguridad)
uint32_t watchdog_ms = 250;    // sin consignas -> nivelar
uint32_t telemetry_hz = 20;

// ---------------- estado ----------------
enum State { IDLE, RUNNING, WATCHDOG, RAW };
State state = IDLE;
float cmd[2] = {0, 0};
uint8_t raw_out[2] = {90, 90};
uint32_t last_cmd_ms = 0;
uint32_t last_tel_ms = 0;
uint32_t last_batt_ms = 0;
float vbat = 0;

// Motores DC (base / pruebas)
// Convencion del robot: potencia POSITIVA = rueda hacia ADELANTE y encoder CRECIENTE
// en ambas ruedas. El motor 1 esta montado en espejo (con potencia cruda positiva va
// hacia atras, confirmado por Caleb el 28-sep), asi que se invierte AQUI, en un solo
// lugar, para potencia y encoder a la vez. No se usa prizm.setMotorInvert(): es un
// registro del chip de motores (no documenta si invierte tambien el encoder) y ese
// chip ya mostro perder comandos por I2C.
const int8_t MOTOR_SIGN[4] = {-1, +1, +1, +1};
const int WHEEL_LEFT = 0, WHEEL_RIGHT = 1;  // VERIFICAR: motor 1 = rueda izquierda (config.yaml). Lo usan la politica y la base

void write_powers(int16_t p1, int16_t p2, int16_t p3, int16_t p4) {
  prizm.setMotorPowers(MOTOR_SIGN[0] * p1, MOTOR_SIGN[1] * p2, MOTOR_SIGN[2] * p3, MOTOR_SIGN[3] * p4);
}

int32_t read_enc(uint8_t ch) {
  return MOTOR_SIGN[ch - 1] * prizm.getEncoderCount(ch);
}

int16_t motor_power[4] = {0, 0, 0, 0};
uint32_t last_motor_ms = 0;
uint32_t motor_wd_ms = 300;
bool motors_active = false;

void set_motors(const int16_t p[4]) {
  for (int i = 0; i < 4; ++i) motor_power[i] = constrain(p[i], -100, 100);
  write_powers(motor_power[0], motor_power[1], motor_power[2], motor_power[3]);
  motors_active = motor_power[0] || motor_power[1] || motor_power[2] || motor_power[3];
}

void stop_motors() {
  const int16_t z[4] = {0, 0, 0, 0};
  set_motors(z);
}

// ---------------- identificacion de planta ----------------
// Senal de entrada del experimento en el instante t_ms (desde el inicio).

float exp_input(Experiment& e, uint32_t t_ms) {
  if (!strcmp(e.type, "STEP")) {
    return (t_ms >= 400 && t_ms < 400 + (uint32_t)e.a[1]) ? e.a[0] : 0;
  }
  if (!strcmp(e.type, "STAIR")) {
    // 0 -> +umax subiendo, +umax -> -umax bajando, -umax -> 0 subiendo
    const int n = (int)ceilf(e.a[0] / e.a[1]);  // escalones por tramo
    const int idx = t_ms / (uint32_t)e.a[2];
    float u;
    if (idx <= n) u = idx * e.a[1];
    else if (idx <= 3 * n) u = e.a[0] - (idx - n) * e.a[1];
    else u = -e.a[0] + (idx - 3 * n) * e.a[1];
    return constrain(u, -e.a[0], e.a[0]);
  }
  if (!strcmp(e.type, "PRBS")) {
    const int32_t bit_idx = t_ms / (uint32_t)e.a[2];
    if (bit_idx != e.last_bit_idx) {  // LFSR x^9 + x^5 + 1 (maxima longitud 511)
      e.last_bit_idx = bit_idx;
      const uint16_t b = ((e.lfsr >> 8) ^ (e.lfsr >> 4)) & 1;
      e.lfsr = ((e.lfsr << 1) | b) & 0x1FF;
      e.prbs_bit = b;
    }
    return e.a[0] + (e.prbs_bit ? e.a[1] : -e.a[1]);
  }
  return 0;
}

uint32_t exp_duration_ms(const Experiment& e) {
  if (!strcmp(e.type, "STEP")) return 400 + (uint32_t)e.a[1] + 800;
  if (!strcmp(e.type, "STAIR")) return (uint32_t)(4 * ceilf(e.a[0] / e.a[1]) + 1) * (uint32_t)e.a[2];
  if (!strcmp(e.type, "PRBS")) return (uint32_t)e.a[3] * (uint32_t)e.a[2];
  return 0;
}

void run_experiment(Experiment& e) {
  e.total_ms = exp_duration_ms(e);
  if (e.total_ms == 0 || e.total_ms > 120000 || e.hz < 1 || e.hz > 1000) {
    Serial.println("ERR experimento invalido (tipo/duracion/hz)");
    return;
  }
  stop_motors();
  delay(300);
  prizm.resetEncoders();
  const uint32_t period_us = (uint32_t)(1e6f / e.hz);
  Serial.printf("IBEGIN %s motor=%d hz=%.1f a=%.2f,%.2f,%.2f,%.2f dur_ms=%lu period_us=%lu\n", e.type,
                e.motor, e.hz, e.a[0], e.a[1], e.a[2], e.a[3], (unsigned long)e.total_ms,
                (unsigned long)period_us);
  const uint32_t t0 = micros();
  uint32_t next = t0;
  uint32_t k = 0, late = 0;
  int16_t last_u = 32767;
  bool aborted = false;
  while (true) {
    const uint32_t now = micros();
    if ((int32_t)(now - next) < 0) continue;  // espera activa hasta el instante de muestreo
    if (now - next > period_us / 2) ++late;    // muestra atrasada mas de medio periodo
    const uint32_t t = now - t0;
    if (t / 1000 >= e.total_ms) break;

    const int16_t u = (int16_t)lroundf(exp_input(e, t / 1000));
    if (u != last_u) {  // escribir potencia solo si cambia (I2C)
      const int16_t u1 = (e.motor & 1) ? u : 0, u2 = (e.motor & 2) ? u : 0;
      write_powers(u1, u2, 0, 0);
      motor_power[0] = u1; motor_power[1] = u2;
      last_u = u;
    }
    const uint32_t tr = micros();
    const int32_t e1 = read_enc(1);
    const int32_t e2 = read_enc(2);
    const uint32_t read_us = micros() - tr;
    Serial.printf("D,%lu,%lu,%d,%d,%ld,%ld,%lu\n", (unsigned long)k, (unsigned long)t, motor_power[0],
                  motor_power[1], (long)e1, (long)e2, (unsigned long)read_us);
    ++k;
    next += period_us;
    if (Serial.available() && Serial.peek() == 'X') { Serial.read(); aborted = true; break; }
  }
  stop_motors();
  Serial.printf("IEND %lu %lu%s\n", (unsigned long)k, (unsigned long)late, aborted ? " ABORTADO" : "");
  last_motor_ms = millis();
}

// ---------------- lazo de velocidad propio (PI + feedforward) ----------------
// Valores de la identificacion del 28-sep (resultados/modelo_motores.json):
// PI por cancelacion de polo, Ti = tau, Kp = tau / (g * Tcl), Tcl = 60 ms.
SpeedPI spi[2] = {
  // m1: lados +/- intercambiados respecto a modelo_motores.json porque la identificacion
  // se hizo en la convencion cruda del motor (antes de invertir m1).
  {4.07f, 142.0f, 0.1160f, 5.03f, 0.1184f, 5.76f},
  {2.82f, 139.0f, 0.1201f, 3.94f, 0.1191f, 3.78f},
};

// Perfil de referencia de velocidad (rad/s) para comparar controladores.
float ref_profile(float t) {
  if (t < 0.3f) return 0;
  if (t < 1.5f) return 5;
  if (t < 2.7f) return 8;
  if (t < 3.9f) return 2;
  if (t < 5.1f) return -5;
  return 0;
}
const float REF_PROFILE_S = 6.0f;

// Chirp lineal (I CHIRP): a[0] amplitud rad/s, a[1] f0 Hz, a[2] f1 Hz, a[3] duracion del barrido s, a[4] offset
float chirp_ref(const Experiment& e, float t) {
  const float amp = e.a[0], f0 = e.a[1], f1 = e.a[2], T = e.a[3], off = e.a[4];
  float r;
  if (t < 0.5f) r = 0;
  else if (t < 1.5f) r = off;
  else if (t < 1.5f + T) {
    const float tc = t - 1.5f;
    r = off + amp * sinf(6.283185f * (f0 * tc + 0.5f * (f1 - f0) / T * tc * tc));
  } else if (t < 2.0f + T) r = off;
  else r = 0;
  return constrain(r, -12.0f, 12.0f);
}

float g_int_sep = 0;  // separacion de la integral (rad/s); 0 = desactivada

float pi_step(SpeedPI& c, float ref, float ref_fb, float w, float dt) {
  // feedforward estatico: invierte la curva zona muerta + ganancia (con la referencia cruda)
  float ff = 0;
  if (ref > 0.05f) ff = ref / c.g_pos + c.dz_pos;
  else if (ref < -0.05f) ff = ref / c.g_neg - c.dz_neg;
  const float e = ref_fb - w;  // realimentacion contra la referencia del modelo (2 GDL) o la cruda
  const float u_unsat = ff + c.kp * e + c.integ;
  const float u = constrain(u_unsat, -100.0f, 100.0f);
  // anti-windup por integracion condicional: no integrar si satura y el error empuja mas alla
  const bool sep_ok = g_int_sep <= 0 || fabsf(e) < g_int_sep;
  if (sep_ok && (u == u_unsat || (u_unsat > 100.0f && e < 0) || (u_unsat < -100.0f && e > 0)))
    c.integ += c.ki * e * dt;
  return u;
}

void run_closed_loop(Experiment& e) {
  const bool internal = !strcmp(e.type, "CLINT");
  const bool chirp = !strcmp(e.type, "CHIRP");
  if (chirp && (e.a[0] <= 0 || e.a[1] <= 0 || e.a[2] <= e.a[1] || e.a[3] < 1 || e.a[3] > 110)) {
    Serial.println("ERR I CHIRP <motor> <hz> <amp> <f0> <f1> <T_s> [offset]");
    return;
  }
  // CHIRP usa los parametros del modo 'V' (VEL_WIN = 2, VEL_TAU_M = 40 ms, sin separacion): es lo que replica la sim
  const int win = chirp ? 2 : max(1, (int)e.a[0]);
  const float tau_m = chirp ? 0.040f : e.a[1] / 1000.0f;  // modelo de referencia (s); 0 = desactivado
  g_int_sep = chirp ? 0 : e.a[2];
  const float dur_s = chirp ? e.a[3] + 2.5f : REF_PROFILE_S;
  float ref_m = 0;
  if (e.hz < 10 || e.hz > 500) { Serial.println("ERR hz fuera de rango (10-500)"); return; }
  stop_motors();
  delay(300);
  prizm.resetEncoders();
  for (auto& c : spi) c.integ = 0;
  const uint32_t period_us = (uint32_t)(1e6f / e.hz);
  const float dt = 1.0f / e.hz;
  Serial.printf("IBEGIN %s motor=%d hz=%.1f ventana=%d tau_m_ms=%.0f int_sep=%.2f dur_ms=%lu period_us=%lu\n", e.type, e.motor, e.hz,
                win, tau_m * 1000, g_int_sep, (unsigned long)(dur_s * 1000), (unsigned long)period_us);
  if (chirp) Serial.printf("ICHIRP amp=%.3f f0=%.3f f1=%.3f T=%.1f offset=%.3f\n", e.a[0], e.a[1], e.a[2], e.a[3], e.a[4]);
  int32_t hist[2][16] = {{0}};  // posiciones recientes para la ventana de velocidad
  const uint32_t t0 = micros();
  uint32_t next = t0, k = 0, late = 0;
  float last_ref = 1e9f;
  bool aborted = false;
  while (true) {
    const uint32_t now = micros();
    if ((int32_t)(now - next) < 0) continue;
    if (now - next > period_us / 2) ++late;
    const float t = (now - t0) * 1e-6f;
    if (t >= dur_s) break;
    const uint32_t tc = micros();
    const float ref = chirp ? chirp_ref(e, t) : ref_profile(t);
    if (tau_m > 0) {
      const float am = expf(-dt / tau_m);
      ref_m = am * ref_m + (1 - am) * ref;
    } else {
      ref_m = ref;
    }
    const int32_t enc[2] = {read_enc(1), read_enc(2)};
    float w[2];
    for (int m = 0; m < 2; ++m) {
      for (int i = 15; i > 0; --i) hist[m][i] = hist[m][i - 1];
      hist[m][0] = enc[m];
      const int n = min<int>(win, (int)k);  // al inicio hay menos historia
      w[m] = n > 0 ? (hist[m][0] - hist[m][n]) * (6.283185f / 1440.0f) / (n * dt) : 0;
    }
    int16_t u[2] = {0, 0};
    if (internal) {
      if (ref != last_ref) {  // el PID interno recibe consigna en grados/s; solo se escribe al cambiar
        const int16_t dps = (int16_t)lroundf(ref * 57.29578f);
        // setMotorSpeeds (escritura en bloque, reg 0x33) no movio los motores en la PRIZM Pro
        // (28-sep); los ejemplos oficiales usan setMotorSpeed por canal.
        if (e.motor & 1) { prizm.setMotorSpeed(1, MOTOR_SIGN[0] * dps); delay(2); }
        if (e.motor & 2) { prizm.setMotorSpeed(2, MOTOR_SIGN[1] * dps); delay(2); }
        last_ref = ref;
      }
    } else {
      for (int m = 0; m < 2; ++m)
        if (e.motor & (1 << m)) u[m] = (int16_t)lroundf(pi_step(spi[m], ref, ref_m, w[m], dt));
      write_powers(u[0], u[1], 0, 0);
    }
    motor_power[0] = u[0]; motor_power[1] = u[1];
    const uint32_t cycle_us = micros() - tc;
    Serial.printf("C,%lu,%lu,%.3f,%d,%d,%ld,%ld,%.3f,%.3f,%lu\n", (unsigned long)k, (unsigned long)(now - t0),
                  ref, u[0], u[1], (long)enc[0], (long)enc[1], w[0], w[1], (unsigned long)cycle_us);
    ++k;
    next += period_us;
    if (Serial.available() && Serial.peek() == 'X') { Serial.read(); aborted = true; break; }
  }
  stop_motors();
  if (internal) { prizm.setMotorSpeed(1, 0); prizm.setMotorSpeed(2, 0); }
  stop_motors();
  Serial.printf("IEND %lu %lu%s\n", (unsigned long)k, (unsigned long)late, aborted ? " ABORTADO" : "");
  last_motor_ms = millis();
}

// ---------------- modo velocidad para despliegue ('V') ----------------
// Mismo PI que CLPI con los parametros que replica la sim (wheel_actuator.py): 100 Hz, ventana de 2
// muestras, modelo de referencia tau_m = 40 ms, sin separacion de la integral. No bloquea: corre
// dentro de loop(), asi que 'S' y la telemetria siguen funcionando. Cada 'V' responde con la
// velocidad medida, que la politica usa como observacion.
const float VEL_HZ = 100.0f;
const int VEL_WIN = 2;
const float VEL_TAU_M = 0.040f;
uint32_t vel_wd_ms = 200;  // sin 'V' en este tiempo -> motores a 0 y se sale del modo
bool vel_mode = false;
float vel_ref[2] = {0, 0}, vel_ref_m[2] = {0, 0}, vel_w[2] = {0, 0};
int32_t vel_hist[2][VEL_WIN + 1];
uint32_t vel_next_us = 0, vel_last_cmd_ms = 0, vel_ticks = 0;
// vel_w lo lee la tarea de la politica (nucleo 0): se escribe y se lee bajo este candado
portMUX_TYPE pol_mux = portMUX_INITIALIZER_UNLOCKED;
// odometria de la base (se integra en vel_step, a 100 Hz, mientras el modo 'V' esta activo)
BaseGeom base_geom = BASE_GEOM_DEFAULT;
Pose2 odo = {0, 0, 0};
// salud del PI de 100 Hz (comando 'G'): hueco maximo entre pasos, pasos con > 15 ms, duracion maxima
uint32_t vel_prev_us = 0, vel_gap_max_us = 0, vel_late = 0, vel_exec_max_us = 0, vel_steps = 0;

// modo DSM (myRIO por UART): definido mas abajo, lo llama vel_step
bool dsm_run = false;
bool dsm_armed = true;  // arranca con la primera linea con consignas; 'D 0' desarma, 'D 1' vuelve a armar
void dsm_wheel_refs(int32_t enc1, int32_t enc2);
void dsm_stop(const char* why);

OdoEkf ekf;
bool ekf_live = false;          // el filtro tiene IMU (si no, la odometria fusionada = encoders)
float dsm_facc[2] = {0, 0};     // cuentas fusionadas acumuladas desde la ultima respuesta [derecha, izquierda]
float ekf_venc = 0, ekf_wenc = 0;

// fusion encoders + IMU (odo_ekf.h) a 100 Hz con las mismas lecturas del PI; acumula las cuentas "fusionadas" que salen
// al myRIO con 'D F 1': avance del filtro y giro del giroscopio, pasados a cuentas por rueda con U G (r, via)
void odo_fuse_step(float w_r, float w_l, float dt) {
  const float r = base_geom.r, b = base_geom.track;
  ekf_venc = r * (w_r + w_l) / 2;
  ekf_wenc = r * (w_r - w_l) / b;
  ekf_live = imu_aorg_ok;
  float v = ekf_venc, wz = ekf_wenc;
  if (ekf_live) {
    v = ekf_step(&ekf, ekf_venc, ekf_wenc, imu_w[2], imu_aorg[0], imu_aorg[1], dt);
    wz = imu_w[2];
  } else {
    ekf.x[0] = ekf_venc;  // sin IMU: el filtro sigue a los encoders, listo para cuando vuelva
    ekf.x[1] = 0;
    ekf.nocontact = 0;
  }
  const float kc = 1440.0f / 6.283185f / r;
  dsm_facc[0] += (v + wz * b / 2) * dt * kc;  // derecha
  dsm_facc[1] += (v - wz * b / 2) * dt * kc;  // izquierda
}

void vel_start() {
  for (auto& c : spi) c.integ = 0;
  g_int_sep = 0;
  for (int m = 0; m < 2; ++m) {
    const int32_t e = read_enc(m + 1);
    for (int i = 0; i <= VEL_WIN; ++i) vel_hist[m][i] = e;
    vel_ref_m[m] = 0;
    vel_w[m] = 0;
  }
  vel_ticks = 0;
  vel_next_us = micros();
  vel_mode = true;
}

void vel_stop() {
  vel_mode = false;
  vel_ref[0] = vel_ref[1] = 0;
  vel_w[0] = vel_w[1] = 0;  // sin el PI nadie la actualiza: no dejar la ultima velocidad congelada en el log
  stop_motors();
}

void vel_step() {
  const uint32_t now = micros();
  if ((int32_t)(now - vel_next_us) < 0) return;
  const uint32_t period_us = (uint32_t)(1e6f / VEL_HZ);
  vel_next_us += period_us;
  if ((int32_t)(now - vel_next_us) > 0) vel_next_us = now + period_us;  // atrasado: no acumular
  if (vel_ticks > 0) {
    const uint32_t gap = now - vel_prev_us;
    if (gap > vel_gap_max_us) vel_gap_max_us = gap;
    if (gap > period_us * 3 / 2) ++vel_late;
  }
  vel_prev_us = now;
  const float dt = 1.0f / VEL_HZ;
  const float am = expf(-dt / VEL_TAU_M);
  int16_t u[2];
  float w[2];
  for (int m = 0; m < 2; ++m) {
    for (int i = VEL_WIN; i > 0; --i) vel_hist[m][i] = vel_hist[m][i - 1];
    vel_hist[m][0] = read_enc(m + 1);
    const int n = min<int>(VEL_WIN, (int)vel_ticks);
    w[m] = n > 0 ? (vel_hist[m][0] - vel_hist[m][n]) * (6.283185f / 1440.0f) / (n * dt) : 0;
  }
  if (dsm_run) dsm_wheel_refs(vel_hist[0][0], vel_hist[1][0]);  // modo DSM: consignas del myRIO
  for (int m = 0; m < 2; ++m) {
    vel_ref_m[m] = am * vel_ref_m[m] + (1 - am) * vel_ref[m];
    u[m] = (int16_t)lroundf(pi_step(spi[m], vel_ref[m], vel_ref_m[m], w[m], dt));
  }
  if (vel_ticks > 0) {  // odometria con el mismo par de lecturas de encoder (sin I2C extra)
    const float k = 6.283185f / 1440.0f;
    base_odom(&base_geom, &odo, (vel_hist[WHEEL_LEFT][0] - vel_hist[WHEEL_LEFT][1]) * k,
              (vel_hist[WHEEL_RIGHT][0] - vel_hist[WHEEL_RIGHT][1]) * k);
    odo_fuse_step(w[WHEEL_RIGHT], w[WHEEL_LEFT], dt);
  }
  portENTER_CRITICAL(&pol_mux);
  vel_w[0] = w[0];
  vel_w[1] = w[1];
  portEXIT_CRITICAL(&pol_mux);
  ++vel_ticks;
  const int16_t p[4] = {u[0], u[1], 0, 0};
  set_motors(p);
  last_motor_ms = millis();  // el watchdog de 'M' no aplica: 'V' tiene el suyo
  const uint32_t exec = micros() - now;
  if (exec > vel_exec_max_us) vel_exec_max_us = exec;
  ++vel_steps;
}

// ---------------- politica a bordo (nucleo 0) ----------------
const float POL_HZ = 30.0f;           // la politica se entreno a 30 Hz: NO cambiar sin reentrenar
const float WHEEL_SCALE = 10.0f;      // rad/s por unidad de accion (ActionsCfg de la sim)
const float PLATE_LIMIT_DEG = 15.0f;  // accion de plato [-1, 1] -> grados
// En la sim el eje de las juntas de rueda es +X del CAD = -Y de la base: velocidad de junta POSITIVA = rueda hacia
// ATRAS. El firmware usa + = adelante. Se invierte aqui, en la consigna y en la medicion (05-oct: con vx > 0 la red
// pedia ruedas negativas y con wz > 0 izq +, der -, coherente con esta convencion).
const float WHEEL_SIGN = -1.0f;
const float TIP_LIMIT_RAD = 0.8f;
uint32_t pol_wd_ms = 200;  // sin 'O' en este tiempo -> parar

McuPolicy pol;
bool pol_in_ram = false;
int pol_init_err = -100;
ObsHist pol_hist;
TaskHandle_t pol_task = nullptr;
esp_timer_handle_t pol_timer = nullptr;

// compartido entre nucleos (bajo pol_mux)
struct PolIn { float cam[5], grav[3], cmd[2]; uint32_t seq, ms; } pol_in = {};
struct PolOut { uint32_t k, seq, inf_us; float w[2], act[4]; } pol_out = {};
volatile bool pol_active = false, pol_reset = false, pol_out_new = false;
bool pol_have_in = false;
// estadisticas de la tarea (las escribe el nucleo 0, 'G' las lee)
volatile uint32_t pol_steps = 0, pol_inf_max_us = 0, pol_inf_sum_us = 0, pol_gap_max_us = 0, pol_gap_min_us = 0xFFFFFFFF;
volatile uint32_t pol_overruns = 0;  // pasos que el nucleo 1 no alcanzo a aplicar antes del siguiente

void IRAM_ATTR pol_timer_cb(void*) {
  // corre en la tarea esp_timer (nucleo 0): solo despierta a la tarea de la politica
  xTaskNotifyGive(pol_task);
}

void pol_task_fn(void*) {
  float act[4] = {0, 0, 0, 0}, obs[OBS_DIM], step[OBS_STEP];
  uint32_t k = 0, prev_us = 0;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!pol_active) { prev_us = 0; continue; }
    const uint32_t t0 = micros();
    PolIn in;
    float w[2];
    bool reset;
    portENTER_CRITICAL(&pol_mux);
    in = pol_in;
    w[0] = WHEEL_SIGN * vel_w[WHEEL_LEFT];  // en la convencion de la sim
    w[1] = WHEEL_SIGN * vel_w[WHEEL_RIGHT];
    reset = pol_reset;
    pol_reset = false;
    portEXIT_CRITICAL(&pol_mux);
    if (reset) {  // como tras un reset de la sim: historial lleno con la primera muestra, accion previa 0
      obs_reset(&pol_hist);
      for (float& a : act) a = 0;
      k = 0;
      prev_us = 0;
    }
    memcpy(step, in.cam, 5 * sizeof(float));
    memcpy(step + 5, in.grav, 3 * sizeof(float));
    step[8] = w[0];
    step[9] = w[1];
    step[10] = in.cmd[0];
    step[11] = 0;  // vy: la base diferencial no lo usa (la sim lo fija en 0)
    step[12] = in.cmd[1];
    memcpy(step + 13, act, 4 * sizeof(float));
    obs_push(&pol_hist, step, obs);
    mcu_policy_run(&pol, obs, act);
    const uint32_t inf = micros() - t0;

    portENTER_CRITICAL(&pol_mux);
    if (pol_out_new) ++pol_overruns;
    pol_out.k = k++;
    pol_out.seq = in.seq;
    pol_out.inf_us = inf;
    pol_out.w[0] = w[0];
    pol_out.w[1] = w[1];
    memcpy(pol_out.act, act, sizeof act);
    pol_out_new = true;
    portEXIT_CRITICAL(&pol_mux);

    ++pol_steps;
    pol_inf_sum_us += inf;
    if (inf > pol_inf_max_us) pol_inf_max_us = inf;
    if (prev_us) {
      const uint32_t gap = t0 - prev_us;
      if (gap > pol_gap_max_us) pol_gap_max_us = gap;
      if (gap < pol_gap_min_us) pol_gap_min_us = gap;
    }
    prev_us = t0;
  }
}

void pol_init() {
  const size_t bytes = sizeof(POLICY_BLOB);
  const void* src = POLICY_BLOB;
  // los pesos en flash se leen por la cache (32 KB) y no caben: copiarlos a RAM interna si hay lugar
  void* ram = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (ram) { memcpy(ram, POLICY_BLOB, bytes); src = ram; pol_in_ram = true; }
  pol_init_err = mcu_policy_init(&pol, src, bytes);
  if (pol_init_err) return;
  xTaskCreatePinnedToCore(pol_task_fn, "policy", 8192, nullptr, 5, &pol_task, 0);
  const esp_timer_create_args_t args = {.callback = pol_timer_cb, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK,
                                        .name = "policy", .skip_unhandled_events = true};
  esp_timer_create(&args, &pol_timer);
  esp_timer_start_periodic(pol_timer, (uint64_t)(1e6f / POL_HZ));
}

void pol_stop(const char* why) {
  if (!pol_active) return;
  pol_active = false;
  vel_stop();
  apply_tilt(0, 0);
  state = IDLE;
  Serial.printf("WD A politica detenida: %s\n", why);
}

void pol_apply() {  // nucleo 1: aplica la ultima salida de la red
  PolOut o;
  portENTER_CRITICAL(&pol_mux);
  o = pol_out;
  pol_out_new = false;
  portEXIT_CRITICAL(&pol_mux);
  if (!pol_active) return;
  vel_ref[WHEEL_LEFT] = constrain(WHEEL_SIGN * o.act[0] * WHEEL_SCALE, -12.0f, 12.0f);
  vel_ref[WHEEL_RIGHT] = constrain(WHEEL_SIGN * o.act[1] * WHEEL_SCALE, -12.0f, 12.0f);
  vel_last_cmd_ms = millis();
  // plato: alfa = pitch, beta = roll (VERIFICAR con el mecanismo final, igual que config.yaml)
  apply_tilt(constrain(o.act[2], -1.0f, 1.0f) * PLATE_LIMIT_DEG, constrain(o.act[3], -1.0f, 1.0f) * PLATE_LIMIT_DEG);
  last_cmd_ms = millis();
  state = RUNNING;
  Serial.printf("a,%lu,%lu,%lu,%lu,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f\n", (unsigned long)millis(), (unsigned long)o.k,
                (unsigned long)o.seq, (unsigned long)o.inf_us, o.w[0], o.w[1], o.act[0], o.act[1], o.act[2], o.act[3]);
}

void pol_selftest() {
  if (pol_init_err) { Serial.printf("ERR Q politica no cargada (%d)\n", pol_init_err); return; }
  if (pol_active) { Serial.println("ERR Q detener la politica primero (A 0)"); return; }
  const int in = pol.dims[0], out = pol.dims[pol.n_layers];
  const float* obs = (const float*)POLICY_TEST;
  const float* ref = obs + POLICY_TEST_N * in;
  float act[4];
  float max_err = 0;
  uint32_t tmin = 0xFFFFFFFF, tmax = 0, tsum = 0;
  for (int n = 0; n < POLICY_TEST_N; ++n) {
    const uint32_t t = micros();
    mcu_policy_run(&pol, obs + n * in, act);
    const uint32_t dt = micros() - t;
    tmin = min(tmin, dt); tmax = max(tmax, dt); tsum += dt;
    for (int j = 0; j < out; ++j) max_err = max(max_err, fabsf(act[j] - ref[n * out + j]));
  }
  Serial.printf("OK Q %s capas=%d,%d,%d,%d,%d pesos_en_%s max_err=%.2e inf_us min=%lu prom=%lu max=%lu heap_libre=%u\n",
                POLICY_SOURCE, pol.dims[0], pol.dims[1], pol.dims[2], pol.dims[3], pol.dims[4],
                pol_in_ram ? "RAM" : "FLASH", max_err, (unsigned long)tmin, (unsigned long)(tsum / POLICY_TEST_N),
                (unsigned long)tmax, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

void pol_stats() {
  const uint32_t n = pol_steps;
  Serial.printf("OK G pol pasos=%lu inf_prom_us=%lu inf_max_us=%lu periodo_min_us=%lu periodo_max_us=%lu sin_aplicar=%lu"
                " | PI100 pasos=%lu hueco_max_us=%lu atrasados=%lu ejec_max_us=%lu\n",
                (unsigned long)n, (unsigned long)(n ? pol_inf_sum_us / n : 0), (unsigned long)pol_inf_max_us,
                (unsigned long)(pol_gap_min_us == 0xFFFFFFFF ? 0 : pol_gap_min_us), (unsigned long)pol_gap_max_us,
                (unsigned long)pol_overruns, (unsigned long)vel_steps, (unsigned long)vel_gap_max_us,
                (unsigned long)vel_late, (unsigned long)vel_exec_max_us);
  pol_steps = pol_inf_sum_us = pol_inf_max_us = pol_gap_max_us = pol_overruns = 0;
  pol_gap_min_us = 0xFFFFFFFF;
  vel_steps = vel_gap_max_us = vel_late = vel_exec_max_us = 0;
}

// ---------------- base: velocidad del cuerpo, circulo y gamepad ('U', 'J') ----------------
// Todo pasa por el PI de 100 Hz del modo 'V': aqui solo se calculan las consignas de rueda (IK de base_kin.c) a 50 Hz.
BaseMode base_mode = BASE_OFF;
CircleTraj circ;
float circ_k[3] = {2.0f, 60.0f, 12.0f};  // kx, ky, kth (los de circle_init; 'U K' los cambia)
bool circ_by_rc = false;  // el circulo lo arranco el gamepad (se para si se pierde la senal)
float base_v = 0, base_w = 0;          // ultimo mando aplicado (tras el limitador)
float twist_v = 0, twist_w = 0;        // consigna de 'U V'
uint32_t base_next_ms = 0, twist_ms = 0, base_tel_last_ms = 0;
uint32_t base_tel_hz = 20;
const uint32_t BASE_PERIOD_MS = 20;    // 50 Hz
const uint32_t TWIST_WD_MS = 300;
const float BASE_ACC = 0.5f, BASE_ALPHA = 3.0f;  // limitador de 'U V' y teleoperacion: m/s^2, rad/s^2

// gamepad
bool rc_enabled = false;
int rc_map[3] = {3, -1, 4};            // v, w, modo: canal 1-4 con signo (Mode 2 de un TX de 4 canales: VERIFICAR con J R)
float rc_center[RC_CHANNELS] = {1500, 1500, 1500, 1500};
float rc_span_us = 400;                // us desde el centro hasta el tope de la palanca
float rc_vmax = 0.25f, rc_wmax = 1.5f; // teleoperacion
float rc_pol_vmax = 0.4f, rc_pol_wmax = 1.2f;  // comando de la politica (curriculum de la sim: 0.4 m/s, 1.2 rad/s)
float rc_circ_r = 0.5f, rc_circ_v = 0.15f, rc_circ_laps = 1;
uint32_t rc_print_until_ms = 0, rc_print_last_ms = 0;
uint32_t rc_gesture_ms = 0;            // desde cuando la palanca de modo esta a fondo
int rc_gesture_dir = 0;
bool rc_gesture_armed = true;          // hay que soltar la palanca de modo entre gestos
bool rc_have_signal = false;
bool rc_pol_cmd = false;               // la politica toma vx/wz del gamepad
float rc_pol_vx = 0, rc_pol_wz = 0;

const char* base_mode_name() {
  switch (base_mode) {
    case BASE_OFF: return "OFF";
    case BASE_TWIST: return "TWIST";
    case BASE_CIRCLE: return "CIRC";
    case BASE_TELEOP: return "RC";
  }
  return "?";
}

void base_cancel() { base_mode = BASE_OFF; base_v = base_w = 0; }  // otro modo toma las ruedas ('V', 'M', 'I', 'A')

void base_stop(const char* why) {
  if (base_mode == BASE_OFF) return;
  base_cancel();
  vel_stop();
  Serial.printf("OK U parada: %s\n", why);
}

void base_begin(BaseMode m) {
  if (pol_active) pol_stop("comando de base");
  if (dsm_run || dsm_armed) { dsm_armed = false; dsm_stop("comando de base / gamepad"); }
  if (!vel_mode) vel_start();
  if (base_mode == BASE_OFF) base_v = base_w = 0;
  base_mode = m;
  base_next_ms = millis();
}

// manda (v, w) a las ruedas por el PI de 'V'
void base_apply(float v, float w) {
  float wl, wr;
  base_ik(&base_geom, v, w, &wl, &wr);
  vel_ref[WHEEL_LEFT] = constrain(wl, -12.0f, 12.0f);
  vel_ref[WHEEL_RIGHT] = constrain(wr, -12.0f, 12.0f);
  vel_last_cmd_ms = millis();  // el watchdog de 'V' lo cubre la base (cada modo tiene el suyo)
  base_v = v;
  base_w = w;
}

void base_apply_limited(float v, float w, float dt) {
  const float dv = BASE_ACC * dt, dw = BASE_ALPHA * dt;
  base_apply(base_v + constrain(v - base_v, -dv, dv), base_w + constrain(w - base_w, -dw, dw));
}

bool start_circle(float r, float v, int dir, float laps, bool by_rc) {
  if (r < 0.1f || r > 5.0f || v <= 0 || v > 0.45f) { Serial.println("ERR U C: R 0.1-5 m, v 0-0.45 m/s"); return false; }
  base_begin(BASE_CIRCLE);
  circle_init(&circ, r, v, dir, laps);
  circ.kx = circ_k[0]; circ.ky = circ_k[1]; circ.kth = circ_k[2];
  circle_start(&circ, &odo);
  circ_by_rc = by_rc;
  Serial.printf("OK U C R=%.2f v=%.2f dir=%+d vueltas=%.1f desde x=%.3f y=%.3f th=%.3f centro=%.3f,%.3f%s\n", r, v,
                circ.dir, laps, odo.x, odo.y, odo.th, circ.cx, circ.cy, by_rc ? " (gamepad)" : "");
  return true;
}

// eje del gamepad: canal con signo -> [-1, 1] con zona muerta; NAN si el canal no tiene senal
float rc_axis(int signed_ch) {
  const int ch = abs(signed_ch) - 1;
  uint16_t us;
  if (!rc_raw(ch, &us, nullptr)) return NAN;
  float x = (us - rc_center[ch]) / rc_span_us;
  x = constrain(x, -1.0f, 1.0f);
  const float db = 0.08f;
  x = fabsf(x) < db ? 0 : (x - copysignf(db, x)) / (1 - db);
  return signed_ch < 0 ? -x : x;
}

void rc_tick(uint32_t now, float dt) {
  const float v = rc_axis(rc_map[0]), w = rc_axis(rc_map[1]), m = rc_axis(rc_map[2]);
  const bool ok = !isnan(v) && !isnan(w) && !isnan(m);
  if (!ok) {
    if (rc_have_signal) Serial.println("WD J gamepad sin senal");
    rc_have_signal = false;
    rc_pol_cmd = false;
    if (base_mode == BASE_TELEOP || (base_mode == BASE_CIRCLE && circ_by_rc)) base_stop("gamepad sin senal");
    return;
  }
  rc_have_signal = true;

  if (pol_active) {  // la red maneja las ruedas: el gamepad solo le da el comando
    rc_pol_vx = v * rc_pol_vmax;
    rc_pol_wz = w * rc_pol_wmax;
    rc_pol_cmd = true;
    portENTER_CRITICAL(&pol_mux);
    pol_in.cmd[0] = rc_pol_vx;
    pol_in.cmd[1] = rc_pol_wz;
    portEXIT_CRITICAL(&pol_mux);
    return;
  }
  rc_pol_cmd = false;

  const bool sticks_free = fabsf(v) < 0.15f && fabsf(w) < 0.15f;
  // gesto: palanca de modo a fondo durante 1 s con las otras sueltas
  const int gdir = m > 0.8f ? 1 : (m < -0.8f ? -1 : 0);
  if (gdir == 0) { rc_gesture_armed = true; rc_gesture_dir = 0; }
  else if (rc_gesture_armed && sticks_free) {
    if (gdir != rc_gesture_dir) { rc_gesture_dir = gdir; rc_gesture_ms = now; }
    else if (now - rc_gesture_ms >= 1000) {
      rc_gesture_armed = false;
      if (base_mode == BASE_CIRCLE) { circle_finish(&circ); Serial.println("OK J circulo: terminando con rampa"); }
      else start_circle(rc_circ_r, rc_circ_v, gdir, rc_circ_laps, true);
      return;
    }
  }

  if (base_mode == BASE_CIRCLE) {
    if (fabsf(v) > 0.3f || fabsf(w) > 0.3f) base_stop("circulo abortado con el gamepad");
    return;
  }
  if (base_mode == BASE_TWIST && sticks_free) return;  // no pisar 'U V' por serie si nadie toca el gamepad
  if (base_mode != BASE_TELEOP) {
    if (v == 0 && w == 0) return;  // solo toma las ruedas cuando alguien mueve una palanca
    base_begin(BASE_TELEOP);
  }
  base_apply_limited(v * rc_vmax, w * rc_wmax, dt);
}

void rc_print(uint32_t now) {
  if (!rc_print_until_ms || now - rc_print_last_ms < 100) return;
  if ((int32_t)(now - rc_print_until_ms) > 0) { rc_print_until_ms = 0; Serial.println("OK J R fin"); return; }
  rc_print_last_ms = now;
  Serial.print("r");
  for (int ch = 0; ch < RC_CHANNELS; ++ch) {
    uint16_t us; uint32_t age;
    const bool ok = rc_raw(ch, &us, &age);
    if (ok) Serial.printf(",ch%d=%u", ch + 1, us);
    else Serial.printf(",ch%d=--", ch + 1);
  }
  Serial.printf(" | v=%.2f w=%.2f modo=%.2f\n", rc_axis(rc_map[0]), rc_axis(rc_map[1]), rc_axis(rc_map[2]));
}

void base_tick() {
  const uint32_t now = millis();
  rc_print(now);
  if ((int32_t)(now - base_next_ms) < 0) return;
  base_next_ms += BASE_PERIOD_MS;
  if ((int32_t)(now - base_next_ms) > 0) base_next_ms = now + BASE_PERIOD_MS;
  const float dt = BASE_PERIOD_MS * 1e-3f;

  if (rc_enabled) rc_tick(now, dt);
  else rc_pol_cmd = false;

  if (base_mode == BASE_TWIST) {
    if (now - twist_ms > TWIST_WD_MS) { base_stop("sin 'U V' (watchdog)"); return; }
    base_apply_limited(twist_v, twist_w, dt);
  } else if (base_mode == BASE_CIRCLE) {
    float v, w;
    circle_step(&circ, &odo, dt, &v, &w);
    if (circ.done) {
      Serial.printf("OK U circulo terminado: %.2f vueltas, pose x=%.3f y=%.3f th=%.3f\n",
                    circ.s / (6.283185f * circ.radius), odo.x, odo.y, odo.th);
      base_stop("fin del circulo");
      return;
    }
    base_apply(v, w);
  }

  if (base_mode != BASE_OFF && base_tel_hz > 0 && now - base_tel_last_ms >= 1000 / base_tel_hz) {
    base_tel_last_ms = now;
    float wl, wr;
    portENTER_CRITICAL(&pol_mux);
    wl = vel_w[WHEEL_LEFT];
    wr = vel_w[WHEEL_RIGHT];
    portEXIT_CRITICAL(&pol_mux);
    const bool c = base_mode == BASE_CIRCLE;
    Serial.printf("b,%lu,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f\n", (unsigned long)now, base_mode_name(),
                  odo.x, odo.y, odo.th, c ? circ.ref.x : odo.x, c ? circ.ref.y : odo.y, c ? circ.ref.th : odo.th,
                  base_v, base_w, wl, wr);
  }
}

void base_print() {
  Serial.printf("OK U modo=%s r=%.4f via=%.4f k=%.2f,%.2f,%.2f pose=%.4f,%.4f,%.4f v=%.3f w=%.3f tel_hz=%lu\n",
                base_mode_name(), base_geom.r, base_geom.track, circ_k[0], circ_k[1], circ_k[2], odo.x, odo.y, odo.th,
                base_v, base_w, (unsigned long)base_tel_hz);
}

void rc_print_config() {
  Serial.printf("OK J %s mapa v=%d w=%d modo=%d centros=%.0f,%.0f,%.0f,%.0f span=%.0f vmax=%.2f wmax=%.2f"
                " circulo R=%.2f v=%.2f vueltas=%.1f senal=%d\n", rc_enabled ? "ON" : "OFF", rc_map[0], rc_map[1],
                rc_map[2], rc_center[0], rc_center[1], rc_center[2], rc_center[3], rc_span_us, rc_vmax, rc_wmax,
                rc_circ_r, rc_circ_v, rc_circ_laps, rc_have_signal);
  for (int ch = 0; ch < RC_CHANNELS; ++ch) {
    uint16_t us; uint32_t age;
    const bool ok = rc_raw(ch, &us, &age);
    Serial.printf("   ch%d GPIO%u %s ancho=%u us edad=%ld ms pulsos=%lu rechazados=%lu\n", ch + 1, RC_PINS[ch],
                  ok ? "OK" : "SIN_SENAL", us, age == 0xFFFFFFFF ? -1L : (long)age, (unsigned long)rc_count(ch),
                  (unsigned long)rc_rejects(ch));
  }
}

void handle_base(char* s) {
  char sub = s[0];
  while (sub == ' ') sub = *++s;
  char* a = s + 1;
  if (sub == 'V') {
    float v, w;
    if (sscanf(a, "%f %f", &v, &w) != 2) { Serial.println("ERR U V <v m/s> <w rad/s>"); return; }
    if (base_mode != BASE_TWIST) base_begin(BASE_TWIST);
    twist_v = constrain(v, -0.45f, 0.45f);
    twist_w = constrain(w, -3.0f, 3.0f);
    twist_ms = millis();
  } else if (sub == 'C') {
    float r, v, laps = 0; int dir = 1;
    if (sscanf(a, "%f %f %d %f", &r, &v, &dir, &laps) < 2) { Serial.println("ERR U C <R> <v> [dir] [vueltas]"); return; }
    start_circle(r, v, dir, laps, false);
  } else if (sub == 'F') {
    if (base_mode == BASE_CIRCLE) { circle_finish(&circ); Serial.println("OK U F frenando"); }
    else Serial.println("ERR U F: no hay circulo");
  } else if (sub == 'S') {
    if (base_mode == BASE_OFF) Serial.println("OK U S (ya detenida)");
    base_stop("comando U S");
  } else if (sub == 'O') {
    odo = {0, 0, 0};
    base_print();
  } else if (sub == 'G') {
    float r, b;
    if (sscanf(a, "%f %f", &r, &b) != 2 || r < 0.02f || r > 0.1f || b < 0.2f || b > 0.6f) {
      Serial.println("ERR U G <r 0.02-0.1> <via 0.2-0.6>"); return;
    }
    base_geom.r = r;
    base_geom.track = b;
    base_print();
  } else if (sub == 'K') {
    float kx, ky, kth;
    if (sscanf(a, "%f %f %f", &kx, &ky, &kth) != 3) { Serial.println("ERR U K <kx> <ky> <kth>"); return; }
    circ_k[0] = circ.kx = kx; circ_k[1] = circ.ky = ky; circ_k[2] = circ.kth = kth;
    base_print();
  } else if (sub == 'T') {
    base_tel_hz = strtoul(a, nullptr, 10);
    base_print();
  } else if (sub == '?') {
    base_print();
  } else {
    Serial.println("ERR U <V|C|F|S|O|G|K|T|?>");
  }
}

void handle_rc(char* s) {
  char sub = s[0];
  while (sub == ' ') sub = *++s;
  char* a = s + 1;
  if (sub == '1' || sub == '0') {
    rc_enabled = sub == '1';
    if (!rc_enabled && (base_mode == BASE_TELEOP || (base_mode == BASE_CIRCLE && circ_by_rc))) base_stop("J 0");
    rc_gesture_armed = false;  // al habilitar, no tomar como gesto una palanca que ya estaba a fondo
    rc_print_config();
  } else if (sub == 'R') {
    const int secs = atoi(a);
    rc_print_until_ms = millis() + 1000u * (secs > 0 ? secs : 10);
  } else if (sub == 'C') {
    int n_ok = 0;
    for (int ch = 0; ch < RC_CHANNELS; ++ch) {
      uint16_t us;
      if (rc_raw(ch, &us, nullptr)) { rc_center[ch] = us; ++n_ok; }
    }
    Serial.printf("%s J C %d canales calibrados\n", n_ok ? "OK" : "ERR", n_ok);
    rc_print_config();
  } else if (sub == 'M') {
    int v, w, m;
    if (sscanf(a, "%d %d %d", &v, &w, &m) != 3 || !v || !w || !m || abs(v) > 4 || abs(w) > 4 || abs(m) > 4) {
      Serial.println("ERR J M <v> <w> <modo>  (canal 1-4 con signo)"); return;
    }
    rc_map[0] = v; rc_map[1] = w; rc_map[2] = m;
    rc_print_config();
  } else if (sub == 'L') {
    float vm, wm, r, vc, laps;
    if (sscanf(a, "%f %f %f %f %f", &vm, &wm, &r, &vc, &laps) != 5) {
      Serial.println("ERR J L <vmax> <wmax> <R> <v_circ> <vueltas>"); return;
    }
    rc_vmax = constrain(vm, 0.0f, 0.45f);
    rc_wmax = constrain(wm, 0.0f, 3.0f);
    rc_circ_r = r; rc_circ_v = vc; rc_circ_laps = laps;
    rc_print_config();
  } else if (sub == '?') {
    rc_print_config();
  } else {
    Serial.println("ERR J <1|0|R|C|M|L|?>");
  }
}

// ---------------- modo DSM: myRIO por UART en texto (16-oct) ----------------
// Mismo estilo que el acelerometro validado el 12-oct (115200 8N1, MXP A del myRIO):
//   myRIO -> PRIZM:  "G gx gy gz wR_ref wL_ref\r\n"   gravedad normalizada + consignas de rueda en rad/s
//   PRIZM -> myRIO:  "G dN_R dN_L run\r\n"            una respuesta por cada linea valida (run: 1 activo, 0 parada)
// dN = cuentas de encoder desde la respuesta ANTERIOR (no por tiempo fijo): asi la suma de los dN es exactamente el
// avance de la rueda aunque los periodos varien, y la odometria del myRIO no pierde cuentas. + = adelante, 1440/vuelta.
// Las consignas van directo al PI de 100 Hz del modo 'V'. La linea termina en '\n'; si no llega '\n', la siguiente
// 'G' cierra la anterior (tolera el "\\r\\n" literal del VI, a costa de un periodo de retardo).
const int MYRIO_RX = 6, MYRIO_TX = 7;  // puerto de sensor 1: pin 1 = RX (myRIO MXP A pin 14), pin 2 = TX (MXP A pin 10)
const uint32_t MYRIO_BAUD = 115200;
const int DSM_WHEEL[2] = {WHEEL_RIGHT, WHEEL_LEFT};  // indice de la linea (0 = derecha, 1 = izquierda) -> motor

float dsm_g[3] = {0, 0, -1};
float dsm_ref[2] = {0, 0};      // ultima consigna recibida [derecha, izquierda] rad/s
float dsm_w_ref[2] = {0, 0};    // la aplicada (tras el limite)
bool dsm_have_ref = false;
int32_t dsm_prev_cnt[2] = {0, 0};
bool dsm_have_prev = false;
float dsm_wmax = 12.0f;         // rad/s = tope fisico (vacio ~11): sin saturacion artificial (09-nov); D L lo baja
float dsm_acc = 0;              // rad/s2 de rueda, rampa de las consignas ('D A'; 0 = escalon). En la flash
bool dsm_fuse = false;          // 'D F': dN fusionados al myRIO. En la flash
bool dsm_dpsi_imu = false;      // 'D P': dpsi al myRIO del giroscopio (1) o de los encoders (0, default). En la flash
Preferences dsm_prefs;
uint32_t dsm_wd_ms = 200;       // sin lineas validas -> parar
uint32_t dsm_last_ms = 0, dsm_n_ok = 0, dsm_n_err = 0, dsm_n_nolf = 0, dsm_n_tx = 0;
uint32_t dsm_win_ok = 0, dsm_win_ms = 0, dsm_bytes = 0;
float dsm_hz = 0;
char dsm_reason[64] = "esperando lineas del myRIO";
uint32_t dsm_tel_hz = 10, dsm_tel_last_ms = 0;
bool dsm_log_rows = false;       // 'D T L': una fila "L,..." por linea del myRIO
const int DSM_NX = 12;           // campos extra que el myRIO puede agregar (ex, ey, eth, ...): solo para el log
float dsm_x[DSM_NX];
int dsm_nx = 0;
uint32_t dsm_dump_until_ms = 0;  // 'D R': mostrar lo que llega
char dsm_line[192];
size_t dsm_line_len = 0;

// cuentas actuales [derecha, izquierda]: las del ultimo paso del PI si esta corriendo (sin I2C extra)
void dsm_counts(int32_t out[2]) {
  for (int i = 0; i < 2; ++i) out[i] = vel_mode ? vel_hist[DSM_WHEEL[i]][0] : read_enc(DSM_WHEEL[i] + 1);
}

void dsm_wheel_refs(int32_t, int32_t) {  // en vel_step, antes del PI
  // limite PROPORCIONAL: si una rueda pasa de wmax se escalan las dos por el mismo factor, asi se conserva el giro
  // (19-oct: recortando cada rueda por separado, con consignas de +200/+50 las dos quedaban en +6 e iba recto)
  const float m = fmaxf(fabsf(dsm_ref[0]), fabsf(dsm_ref[1]));
  // parado (11-nov): el VI manda +-0.01..0.05 rad/s "en cero"; 1 cuenta en la ventana de 20 ms ya son 0.22 rad/s, asi que
  // el PI integraba hasta vencer la friccion, saltaba 1-2 cuentas y volvia (picos). Bajo 0.1 rad/s (5 mm/s) = consigna 0.
  // El integrador NO se borra: en rampa es el que sostiene al robot.
  const float k = m < 0.1f ? 0.0f : m > dsm_wmax ? dsm_wmax / m : 1.0f;
  // rampa (28-oct): el cambio por paso de 10 ms se limita a dsm_acc/100 en la rueda que mas cambia, y la otra cambia en
  // la misma proporcion -> aceleracion constante en vez de un escalon (sin golpes a la pelota) y la IMU ve la aceleracion
  float d[2], dm = 0;
  for (int i = 0; i < 2; ++i) { d[i] = dsm_ref[i] * k - dsm_w_ref[i]; dm = fmaxf(dm, fabsf(d[i])); }
  const float lim = dsm_acc / VEL_HZ, s = dsm_acc > 0 && dm > lim ? lim / dm : 1.0f;
  for (int i = 0; i < 2; ++i) {
    dsm_w_ref[i] += d[i] * s;
    vel_ref[DSM_WHEEL[i]] = dsm_w_ref[i];
  }
  vel_last_cmd_ms = millis();
}

// respuesta por USB y por WiFi (los comandos nuevos: dsm_gui por WiFi tiene que ver la confirmacion)
void dsm_reply(const char* fmt, ...) {
  char b[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  Serial.println(b);
  net_send_line(b);
}

void dsm_set_reason(const char* why) {
  strncpy(dsm_reason, why, sizeof dsm_reason - 1);
  dsm_reason[sizeof dsm_reason - 1] = 0;
}

void dsm_stop(const char* why) {
  if (!dsm_run) return;
  dsm_run = false;
  vel_stop();
  dsm_w_ref[0] = dsm_w_ref[1] = 0;
  char b[64];
  snprintf(b, sizeof b, "detenido: %s", why);
  dsm_set_reason(b);
  char m[80];
  snprintf(m, sizeof m, "WD D %s", b);
  Serial.println(m);
  net_send_line(m);
}

void dsm_start() {
  if (dsm_run) return;
  pol_stop("modo DSM");
  base_cancel();
  vel_start();  // PI desde cero (integradores e historial de encoders)
  vel_last_cmd_ms = millis();  // si no, el watchdog de 'V' lo corta en la primera vuelta de loop()
  odo = {0, 0, 0};  // el myRIO tambien arranca su odometria en cero
  dsm_w_ref[0] = dsm_w_ref[1] = 0;  // la rampa arranca desde las ruedas quietas
  ekf_init(&ekf);
  dsm_facc[0] = dsm_facc[1] = 0;
  dsm_run = true;
  dsm_set_reason("activo");
  Serial.println("OK D control activo (consignas del myRIO)");
  net_send_line("OK D control activo (consignas del myRIO)");
}

void dsm_print() {
  int32_t enc[2];
  dsm_counts(enc);
  char m[400];
  snprintf(m, sizeof m, "OK D run=%d armado=%d | rx %.1f Hz ok=%lu err=%lu sin_LF=%lu bytes=%lu edad=%lu ms | g=%.4f,%.4f,%.4f"
                " ref=%.3f,%.3f extra=%d | respuestas=%lu cuentas=%ld,%ld | wmax=%.1f wd=%lu log=%s rampa=%.1f fusion=%d dpsi=%s bat=%.2fV | %s",
                dsm_run, dsm_armed, dsm_hz, (unsigned long)dsm_n_ok, (unsigned long)dsm_n_err,
                (unsigned long)dsm_n_nolf, (unsigned long)dsm_bytes,
                (unsigned long)(dsm_n_ok ? millis() - dsm_last_ms : 0), dsm_g[0], dsm_g[1], dsm_g[2], dsm_ref[0],
                dsm_ref[1], dsm_nx, (unsigned long)dsm_n_tx, (long)enc[0], (long)enc[1], dsm_wmax, (unsigned long)dsm_wd_ms,
                dsm_log_rows ? "filas" : "hz", dsm_acc, dsm_fuse, dsm_dpsi_imu ? "giro" : "encoders", vbat, dsm_reason);
  Serial.println(m);
  net_send_line(m);
}

void dsm_log_row(const int32_t enc[2], const long dn[2]) {
  float w[2];
  portENTER_CRITICAL(&pol_mux);
  w[0] = vel_w[DSM_WHEEL[0]]; w[1] = vel_w[DSM_WHEEL[1]];
  portEXIT_CRITICAL(&pol_mux);
  char b[400];
  int k = snprintf(b, sizeof b,
                   "L,%lu,%lu,%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%ld,%ld,%ld,%ld,%.4f,%.4f,%.4f,%d",
                   (unsigned long)millis(), (unsigned long)dsm_n_ok, dsm_run, dsm_g[0], dsm_g[1], dsm_g[2],
                   dsm_ref[0], dsm_ref[1], dsm_w_ref[0], dsm_w_ref[1], w[0], w[1],
                   motor_power[DSM_WHEEL[0]], motor_power[DSM_WHEEL[1]], (long)enc[0], (long)enc[1], dn[0], dn[1],
                   odo.x, odo.y, odo.th, dsm_nx);
  for (int i = 0; i < dsm_nx && k < (int)sizeof b - 20; ++i) k += snprintf(b + k, sizeof b - k, ",%.4f", dsm_x[i]);
  Serial.println(b);
  net_send_line(b);
}

void dsm_handle_line() {
  dsm_line[dsm_line_len] = 0;
  while (dsm_line_len > 0 && (dsm_line[dsm_line_len - 1] == '\r' || dsm_line[dsm_line_len - 1] == ' '))
    dsm_line[--dsm_line_len] = 0;
  if (dsm_line_len == 0) return;
  if (dsm_dump_until_ms) Serial.printf("rx \"%s\"\n", dsm_line);
  char* p = dsm_line;
  while (*p == ' ') ++p;
  if (*p != 'G') { ++dsm_n_err; return; }
  ++p;
  float v[5 + DSM_NX];
  int n = 0;  // numeros despues de la 'G'
  while (n < 5 + DSM_NX) {
    char* e;
    const float f = strtof(p, &e);
    if (e == p) break;
    v[n++] = f;
    p = e;
  }
  if ((n != 3 && n < 5) || !isfinite(v[0])) { ++dsm_n_err; return; }
  memcpy(dsm_g, v, sizeof dsm_g);
  dsm_have_ref = n >= 5 && isfinite(v[3]) && isfinite(v[4]);
  if (dsm_have_ref) { dsm_ref[0] = v[3]; dsm_ref[1] = v[4]; }
  dsm_nx = n > 5 ? n - 5 : 0;
  memcpy(dsm_x, v + 5, dsm_nx * sizeof(float));
  dsm_last_ms = millis();
  ++dsm_n_ok;
  ++dsm_win_ok;

  int32_t enc[2];  // respuesta: incremento de cuentas desde la respuesta anterior
  dsm_counts(enc);
  if (!dsm_have_prev) { dsm_prev_cnt[0] = enc[0]; dsm_prev_cnt[1] = enc[1]; dsm_have_prev = true; imu_take_dpsi(); }
  long dn[2] = {(long)(enc[0] - dsm_prev_cnt[0]), (long)(enc[1] - dsm_prev_cnt[1])};
  const long dn_raw[2] = {dn[0], dn[1]};
  if (dsm_fuse && dsm_run && ekf_live) {  // dN de la odometria fusionada (el resto fraccionario queda para la proxima)
    for (int i = 0; i < 2; ++i) { dn[i] = lroundf(dsm_facc[i]); dsm_facc[i] -= dn[i]; }
  } else {
    dsm_facc[0] = dsm_facc[1] = 0;
  }
  float dpsi = imu_take_dpsi();  // mismo instante que los dN (y se descarta junto con ellos tras un silencio)
  // 28-oct noche: por defecto de los ENCODERS (D P 0). En el cuadrado de las 23:33 el giroscopio no vio el giro y el
  // rumbo del VI (que usa dpsi) se congelo. Con D P 1 vuelve el del giroscopio (solo si llega la IMU).
  if (!imu_ok || !dsm_dpsi_imu) dpsi = (dn_raw[0] - dn_raw[1]) * (6.283185f / 1440.0f) * base_geom.r / base_geom.track;
  char out[80];
  snprintf(out, sizeof out, "G %ld %ld %d %.5f %.4f %.4f\r\n", dn[0], dn[1], dsm_run ? 1 : 0, dpsi,
           imu_ok ? imu_pitch() : 0.0f, imu_ok ? imu_roll() : 0.0f);  // run: 0 = PRIZM parada
  dsm_prev_cnt[0] = enc[0];
  dsm_prev_cnt[1] = enc[1];
  Serial1.print(out);
  ++dsm_n_tx;
  if (dsm_dump_until_ms) Serial.printf("tx \"%.*s\"\n", (int)strlen(out) - 2, out);
  if (dsm_log_rows) {
    dsm_log_row(enc, dn);
    char b[200];
    snprintf(b, sizeof b, "F,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%.2f,%lu,%ld,%ld", (unsigned long)millis(), ekf_venc,
             ekf_wenc, imu_w[2], ekf.x[0], ekf.x[1], ekf.x[2], ekf.rej_v, ekf.rej_w, ekf.nocontact, ekf.nis_win,
             (unsigned long)ekf.n_lost, dn_raw[0], dn_raw[1]);
    Serial.println(b);
    net_send_line(b);
  }

  if (!dsm_run) return;
  if (!dsm_have_ref) { dsm_stop("linea del myRIO sin consignas"); return; }
  const float gn = sqrtf(dsm_g[0] * dsm_g[0] + dsm_g[1] * dsm_g[1] + dsm_g[2] * dsm_g[2]);
  if (gn > 0.3f && acosf(constrain(-dsm_g[2] / gn, -1.0f, 1.0f)) > TIP_LIMIT_RAD) dsm_stop("inclinacion > limite");
}

void dsm_tick() {
  uint32_t now = millis();
  if (dsm_dump_until_ms && (int32_t)(now - dsm_dump_until_ms) >= 0) { dsm_dump_until_ms = 0; Serial.println("OK D R fin"); }
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    ++dsm_bytes;
    if (c == '\n') {
      dsm_handle_line();
      dsm_line_len = 0;
    } else if (c == 'G' && dsm_line_len > 0) {  // llega otra 'G' sin '\n' antes
      ++dsm_n_nolf;
      dsm_handle_line();
      dsm_line_len = 0;
      dsm_line[dsm_line_len++] = c;
    } else if (dsm_line_len < sizeof(dsm_line) - 1) {
      dsm_line[dsm_line_len++] = c;
    } else {  // linea demasiado larga: baudrate o cableado
      ++dsm_n_err;
      dsm_line_len = 0;
    }
  }
  now = millis();
  const bool fresh = dsm_n_ok && dsm_have_ref && now - dsm_last_ms <= dsm_wd_ms;
  if (dsm_run && now - dsm_last_ms > dsm_wd_ms) dsm_stop("sin lineas del myRIO (watchdog)");
  // myRIO callado: la primera respuesta al volver sale con dN = 0. Si no, traeria lo que giraron las ruedas mientras
  // se acomodaba el robot a mano con el VI apagado, y la odometria del VI saltaria justo despues de su Reset (20-oct)
  if (dsm_have_prev && dsm_n_ok && now - dsm_last_ms > dsm_wd_ms) dsm_have_prev = false;
  if (dsm_armed && !dsm_run && fresh) dsm_start();  // directo: arranca con la primera linea con consignas

  if (now - dsm_win_ms >= 1000) {
    dsm_hz = dsm_win_ok * 1000.0f / (now - dsm_win_ms);
    dsm_win_ok = 0;
    dsm_win_ms = now;
  }
  if (dsm_run && !dsm_log_rows && dsm_tel_hz > 0 && now - dsm_tel_last_ms >= 1000 / dsm_tel_hz) {
    dsm_tel_last_ms = now;
    int32_t enc[2];
    dsm_counts(enc);
    float w[2];
    portENTER_CRITICAL(&pol_mux);
    w[0] = vel_w[DSM_WHEEL[0]]; w[1] = vel_w[DSM_WHEEL[1]];
    portEXIT_CRITICAL(&pol_mux);
    Serial.printf("d,%lu,%.3f,%.3f,%.3f,%.3f,%ld,%ld\n", (unsigned long)now, dsm_w_ref[0], dsm_w_ref[1], w[0], w[1],
                  (long)enc[0], (long)enc[1]);
  }
}

void handle_dsm(char* s) {
  char sub = s[0];
  while (sub == ' ') sub = *++s;
  char* a = s + 1;
  if (sub == '1') { dsm_armed = true; Serial.println("OK D 1 armado (arranca con la primera linea con consignas)"); }
  else if (sub == '0') { dsm_armed = false; if (dsm_run) dsm_stop("comando D 0 (desarmado)"); else Serial.println("OK D 0 desarmado"); }
  else if (sub == 'Z') {
    if (vel_mode) { Serial.println("ERR D Z: parar las ruedas primero (D 0)"); return; }
    prizm.resetEncoders();
    dsm_have_prev = false;
    Serial.println("OK D Z encoders a cero");
  } else if (sub == 'L') {
    float wmax, wd;
    if (sscanf(a, "%f %f", &wmax, &wd) != 2 || wmax < 0.5f || wmax > 12 || wd < 50 || wd > 2000) {
      Serial.println("ERR D L <wmax 0.5-12 rad/s> <watchdog 50-2000 ms>"); return;
    }
    dsm_wmax = wmax;
    dsm_wd_ms = (uint32_t)wd;
    dsm_print();
  } else if (sub == 'T') {
    while (*a == ' ') ++a;
    dsm_log_rows = *a == 'L';
    if (!dsm_log_rows) dsm_tel_hz = strtoul(a, nullptr, 10);
    dsm_print();
  } else if (sub == 'O') {
    odo = {0, 0, 0};
    Serial.println("OK D O odometria a cero");
  } else if (sub == 'A') {
    const float acc = strtof(a, nullptr);
    if (acc < 0 || acc > 200) { Serial.println("ERR D A <aceleracion de rueda 0-200 rad/s2, 0 = escalon>"); return; }
    dsm_acc = acc;
    dsm_prefs.putFloat("acc", dsm_acc);
    dsm_reply("OK D A rampa %.1f rad/s2 de rueda = %.2f m/s2 (0 a %.1f rad/s en %.2f s)", dsm_acc,
                  dsm_acc * base_geom.r, dsm_wmax, dsm_acc > 0 ? dsm_wmax / dsm_acc : 0.0f);
  } else if (sub == 'F') {
    while (*a == ' ') ++a;
    dsm_fuse = *a == '1';
    dsm_prefs.putBool("fuse", dsm_fuse);
    dsm_reply("OK D F %d (dN al myRIO %s)", dsm_fuse, dsm_fuse ? "FUSIONADOS encoders + IMU" : "crudos de los encoders");
  } else if (sub == 'P') {
    while (*a == ' ') ++a;
    dsm_dpsi_imu = *a == '1';
    dsm_prefs.putBool("dpsi", dsm_dpsi_imu);
    dsm_reply("OK D P %d (dpsi al myRIO de %s)", dsm_dpsi_imu, dsm_dpsi_imu ? "el GIROSCOPIO" : "los ENCODERS");
  } else if (sub == 'E') {
    dsm_reply("OK D E imu=%d fusion=%d dpsi=%s | v_enc=%.3f vx=%.3f vy=%.3f m/s sesgo=%.3f m/s2 | w_enc=%.3f w_gyro=%.3f rad/s | "
                  "sin_contacto=%d nis_ventana=%.1f | rechazos v=%lu w=%lu perdidas=%lu | rampa %.1f rad/s2",
                  ekf_live, dsm_fuse, dsm_dpsi_imu ? "giro" : "encoders", ekf_venc, ekf.x[0], ekf.x[1], ekf.x[2], ekf_wenc, imu_w[2], ekf.nocontact,
                  ekf.nis_win, ekf.n_rej_v, ekf.n_rej_w, ekf.n_lost, dsm_acc);
  } else if (sub == 'R') {
    const int secs = atoi(a);
    dsm_dump_until_ms = millis() + 1000u * (secs > 0 ? secs : 1);
  } else if (sub == '?') {
    dsm_print();
  } else {
    Serial.println("ERR D <1|0|Z|L|T|R|O|A|F|P|E|?>");
  }
}

char line[224];  // la "E X" de imu_pi mide ~135 (28-oct)
size_t line_len = 0;

const char* state_name() {
  switch (state) {
    case IDLE: return "IDLE";
    case RUNNING: return "RUN";
    case WATCHDOG: return "WD";
    case RAW: return "RAW";
  }
  return "?";
}

void write_servo(int i, float raw_deg) {
  raw_deg = constrain(raw_deg, 0.0f, 180.0f);
  const uint8_t r = (uint8_t)lroundf(raw_deg);
  raw_out[i] = r;
  prizm.setServoPosition(i + 1, r);  // la libreria solo transmite si cambia
}

void apply_tilt(float a, float b) {
  cmd[0] = constrain(a, -limit_deg, limit_deg);
  cmd[1] = constrain(b, -limit_deg, limit_deg);
  for (int i = 0; i < 2; ++i)
    write_servo(i, axis[i].neutral + axis[i].sign * axis[i].gain * cmd[i]);
}

void print_config() {
  Serial.printf("OK cfg n=%.1f,%.1f s=%.0f,%.0f g=%.3f,%.3f lim=%.1f wd=%lu tel=%lu estado=%s\n",
                axis[0].neutral, axis[1].neutral, axis[0].sign, axis[1].sign, axis[0].gain,
                axis[1].gain, limit_deg, (unsigned long)watchdog_ms, (unsigned long)telemetry_hz,
                state_name());
}

// ---------------- plato de rotulas: IK del CAD + control PD de la pelota con la camara ('H', 20-oct) ----------------
// La Pi (vision_node + prizm_bridge) manda 'O' con la pelota en el frame del plato a la velocidad de la camara. Con cada
// 'O' nuevo: PD (+ I opcional) -> roll / pitch del plato -> IK exacta del CAD (plate_ik_table.h, la misma de
// plato_gui.py) -> servos. Convencion del CAD: roll + sube el borde +Y (la pelota va hacia -Y); pitch + sube el borde
// -X (la pelota va hacia +X). Los signos reales de la camara se ajustan con 'H S' (probar con 'H T' primero).
#include "plate_ik_table.h"
int rot_port_a = 2;                       // puerto del servo A (plato_calib.json)
float rot_neutral[2] = {96, 95};          // crudo con el plato nivelado, por PUERTO 1, 2 (plato_calib.json)
float rot_sign[2] = {1, 1};
const float SERVO_B_MIN = -33.0f;         // por debajo la manivela B toca la barra (README del CAD)

void plate_ik(float roll, float pitch, float out[2]) {
  roll = constrain(roll, -(float)PLATE_LIM, (float)PLATE_LIM);
  pitch = constrain(pitch, -(float)PLATE_LIM, (float)PLATE_LIM);
  const int r0 = min((int)floorf(roll) + PLATE_LIM, 2 * PLATE_LIM - 1), p0 = min((int)floorf(pitch) + PLATE_LIM, 2 * PLATE_LIM - 1);
  const float fr = roll + PLATE_LIM - r0, fp = pitch + PLATE_LIM - p0;
  for (int k = 0; k < 2; ++k) {
    const float a = PLATE_IK[r0][p0][k] * (1 - fr) + PLATE_IK[r0 + 1][p0][k] * fr;
    const float b = PLATE_IK[r0][p0 + 1][k] * (1 - fr) + PLATE_IK[r0 + 1][p0 + 1][k] * fr;
    out[k] = a * (1 - fp) + b * fp;
  }
}

void plate_set(float roll, float pitch) {
  float ab[2];
  plate_ik(roll, pitch, ab);
  ab[1] = fmaxf(ab[1], SERVO_B_MIN);
  const int pa = rot_port_a - 1;
  write_servo(pa, rot_neutral[pa] + rot_sign[pa] * ab[0]);
  write_servo(1 - pa, rot_neutral[1 - pa] + rot_sign[1 - pa] * ab[1]);
  state = RAW;  // que el watchdog de 'S' no lo nivele
}

bool h_on = false;
float h_kp = 50, h_kd = 25, h_ki = 0;     // grados de plato por m, por m/s, por m*s
float h_lim = 10;                         // |roll|, |pitch| maximos (grados)
float h_sx = 1, h_sy = 1;                 // signo de x e y de la camara respecto al plato
int h_swap = 0;                           // 1: la x de la camara es la y del plato
uint32_t h_seq = 0, h_vis_ms = 0, h_cam_ms = 0;
float h_p[2], h_v[2], h_i[2], h_out[2];
bool h_have = false;
float h_kg = 0, h_ka = 0, h_kw = 0, h_tau = 0;  // compensacion con la IMU ('H C'), apagada por defecto
int h_q = 0;                                    // eje X del plato en el chasis ('H Y')
float h_cmd[2] = {0, 0};                        // lo que va a la IK (roll, pitch)
uint32_t h_apply_ms = 0;
float h_tdes[2] = {0, 0};                       // 'H T' (PD de ball_pixel en la Pi): lo que pide, antes de compensar
uint32_t h_t_ms = 0;                            // ultima 'H T': la compensacion la sigue aplicando 200 ms
Preferences h_prefs;                            // H C y H Y quedan en la flash (la PRIZM se reinicia en cada prueba)

void h_print() {
  Serial.printf("OK H on=%d kp=%.1f kd=%.1f ki=%.1f lim=%.1f | signos x=%+.0f y=%+.0f swap=%d | rotulas puertoA=%d "
                "neutro=%.0f,%.0f signo=%+.0f,%+.0f | comp kg=%.2f ka=%.2f kw=%.2f tau=%.3f eje=%d imu=%d\n", h_on, h_kp, h_kd,
                h_ki, h_lim, h_sx, h_sy, h_swap, rot_port_a, rot_neutral[0], rot_neutral[1], rot_sign[0], rot_sign[1],
                h_kg, h_ka, h_kw, h_tau, h_q, imu_ok);
}

// Inclinacion final del plato = lo que pide el PD (h_out) + anular inclinacion y aceleracion del chasis y el giro del
// carro (imu_math.h: plate_comp). A 100 Hz, mas rapido que la camara: la IMU cambia aunque no llegue un 'O' nuevo.
void h_apply() {
  const bool pd = h_on && h_have;  // PD de la PRIZM ('H 1'); si no, lo que pidio 'H T'
  const float des[2] = {h_on ? (h_have ? h_out[0] : 0.0f) : h_tdes[0], h_on ? (h_have ? h_out[1] : 0.0f) : h_tdes[1]};
  if (!imu_ok || (h_kg == 0 && h_ka == 0 && h_kw == 0)) {
    h_cmd[0] = des[0]; h_cmd[1] = des[1];
  } else {
    float u[3];  // adelanto: donde estara la vertical dentro de tau (du/dt = u x w)
    const float* w = imu_w;
    u[0] = imu_u[0] + h_tau * (imu_u[1] * w[2] - imu_u[2] * w[1]);
    u[1] = imu_u[1] + h_tau * (imu_u[2] * w[0] - imu_u[0] * w[2]);
    u[2] = imu_u[2] + h_tau * (imu_u[0] * w[1] - imu_u[1] * w[0]);
    const float fb[2] = {h_kg * IMU_G * u[0] + h_ka * imu_alin[0], h_kg * IMU_G * u[1] + h_ka * imu_alin[1]};
    float f[3];
    body_to_plate(h_q, fb, f);
    f[2] = IMU_G * u[2] + h_ka * imu_alin[2];
    // pelota en un plato que gira w_z: centrifuga w^2 r y Coriolis (2 w vy, -2 w vx), en ejes del plato.
    // shortcut: r desde el centro del plato (no desde el eje de giro del carro) y sin el termino de Euler (dw/dt)
    const float wz = w[2], hp = pd ? 1.0f : 0.0f;  // la pelota solo se conoce con 'H 1' (con 'H T' no hay giro)
    const float fict[2] = {h_kw * hp * (wz * wz * h_p[0] + 2 * wz * h_v[1]),
                           h_kw * hp * (wz * wz * h_p[1] - 2 * wz * h_v[0])};
    plate_comp(des, f, fict, h_cmd);
  }
  plate_set(h_cmd[0], h_cmd[1]);
}

void h_tick() {
  if (!h_on) return;
  float cam[3];
  uint32_t seq, ms;
  portENTER_CRITICAL(&pol_mux);
  memcpy(cam, pol_in.cam, sizeof cam);
  seq = pol_in.seq;
  ms = pol_in.ms;
  portEXIT_CRITICAL(&pol_mux);
  const uint32_t now = millis();
  if (seq == h_seq) {
    // sin pelota (o sin 'O') por 0.5 s: plato nivelado y se olvida el estado
    if (h_have && now - h_vis_ms > 500) { h_have = false; h_i[0] = h_i[1] = 0; }  // h_apply: plato horizontal
    return;
  }
  h_seq = seq;
  if (cam[2] < 0.5f) return;  // 'O' sin pelota visible
  float p[2] = {cam[h_swap ? 1 : 0] * h_sx, cam[h_swap ? 0 : 1] * h_sy};
  const float dt = constrain((ms - h_cam_ms) * 1e-3f, 0.01f, 0.2f);
  for (int k = 0; k < 2; ++k) {
    const float v = h_have ? (p[k] - h_p[k]) / dt : 0;
    h_v[k] = h_have ? 0.6f * h_v[k] + 0.4f * v : 0;  // derivada filtrada (la camara mide a 15-30 Hz)
    if (h_ki > 0) h_i[k] = constrain(h_i[k] + p[k] * dt, -h_lim / h_ki, h_lim / h_ki);
    h_p[k] = p[k];
  }
  h_have = true;
  h_vis_ms = now;
  h_cam_ms = ms;
  // roll + empuja hacia -Y, pitch + hacia +X
  h_out[0] = constrain(h_kp * p[1] + h_kd * h_v[1] + h_ki * h_i[1], -h_lim, h_lim);
  h_out[1] = constrain(-(h_kp * p[0] + h_kd * h_v[0] + h_ki * h_i[0]), -h_lim, h_lim);
  h_apply();
  char m[128];
  snprintf(m, sizeof m, "h,%lu,%.4f,%.4f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f", (unsigned long)now, p[0], p[1], h_v[0], h_v[1],
           h_out[0], h_out[1], h_cmd[0], h_cmd[1]);
  Serial.println(m);
  net_send_line(m);
}

void handle_ball(char* a) {
  char sub = a[0];
  while (sub == ' ') sub = *++a;
  ++a;
  if (sub == '1') {
    pol_stop("modo H");
    h_on = true;
    h_have = false;
    h_i[0] = h_i[1] = 0;
    h_seq = pol_in.seq;
    plate_set(0, 0);
    Serial.println("OK H 1 control de la pelota activo (necesita 'O' de la Pi)");
  } else if (sub == '0') {
    h_on = false;
    plate_set(0, 0);
    Serial.println("OK H 0 plato nivelado");
  } else if (sub == 'K') {
    if (sscanf(a, "%f %f %f %f", &h_kp, &h_kd, &h_ki, &h_lim) < 2) { Serial.println("ERR H K <kp> <kd> [ki] [lim]"); return; }
    h_lim = constrain(h_lim, 1.0f, 15.0f);
    h_print();
  } else if (sub == 'S') {
    int sw = h_swap;
    if (sscanf(a, "%f %f %d", &h_sx, &h_sy, &sw) < 2) { Serial.println("ERR H S <signo x> <signo y> [swap 0|1]"); return; }
    h_sx = h_sx < 0 ? -1 : 1; h_sy = h_sy < 0 ? -1 : 1; h_swap = sw ? 1 : 0;
    h_print();
  } else if (sub == 'T') {  // inclinacion manual con la IK (pruebas de signo): apaga el control
    float r, pt;
    if (sscanf(a, "%f %f", &r, &pt) != 2) { Serial.println("ERR H T <roll> <pitch>"); return; }
    h_on = false;
    h_tdes[0] = r; h_tdes[1] = pt;
    h_t_ms = millis();
    h_apply();  // + compensacion con la IMU (H C); con ganancias 0 es la inclinacion pedida tal cual
    Serial.printf("OK H T roll=%.1f pitch=%.1f -> %.1f %.1f servos %u %u\n", r, pt, h_cmd[0], h_cmd[1], raw_out[0], raw_out[1]);
  } else if (sub == 'R') {
    int pa; float n1, n2, s1, s2;
    if (sscanf(a, "%d %f %f %f %f", &pa, &n1, &n2, &s1, &s2) != 5 || pa < 1 || pa > 2) {
      Serial.println("ERR H R <puerto A 1|2> <neutro1> <neutro2> <signo1> <signo2>"); return;
    }
    rot_port_a = pa; rot_neutral[0] = n1; rot_neutral[1] = n2; rot_sign[0] = s1 < 0 ? -1 : 1; rot_sign[1] = s2 < 0 ? -1 : 1;
    h_print();
  } else if (sub == 'C') {
    if (sscanf(a, "%f %f %f %f", &h_kg, &h_ka, &h_kw, &h_tau) < 3) { Serial.println("ERR H C <kg> <ka> <kw> [tau]"); return; }
    h_kg = constrain(h_kg, 0.0f, 1.0f); h_ka = constrain(h_ka, 0.0f, 1.0f); h_kw = constrain(h_kw, 0.0f, 1.0f);
    h_tau = constrain(h_tau, 0.0f, 0.3f);
    h_prefs.putFloat("kg", h_kg); h_prefs.putFloat("ka", h_ka); h_prefs.putFloat("kw", h_kw); h_prefs.putFloat("tau", h_tau);
    h_print();
  } else if (sub == 'Y') {
    h_q = atoi(a) & 3;
    h_prefs.putInt("q", h_q);
    h_print();
  } else if (sub == '?') {
    h_print();
  } else {
    Serial.println("ERR H <1|0|K|S|T|R|C|Y|?>");
  }
}

void handle_line(char* s) {
  char c = s[0];
  if (c == 'S') {
    float a, b;
    if (sscanf(s + 1, "%f %f", &a, &b) != 2) { Serial.println("ERR S <alfa> <beta>"); return; }
    apply_tilt(a, b);
    last_cmd_ms = millis();
    state = RUNNING;
  } else if (c == 'P') {
    int ch; float raw;
    if (sscanf(s + 1, "%d %f", &ch, &raw) != 2 || ch < 1 || ch > 2) { Serial.println("ERR P <1|2> <0-180>"); return; }
    write_servo(ch - 1, raw);
    state = RAW;  // el watchdog no pisa una posicion manual
    Serial.printf("OK P %d %u\n", ch, raw_out[ch - 1]);
  } else if (c == 'C') {
    float n1, n2, s1, s2, g1, g2, lim;
    if (sscanf(s + 1, "%f %f %f %f %f %f %f", &n1, &n2, &s1, &s2, &g1, &g2, &lim) != 7) {
      Serial.println("ERR C <n1> <n2> <s1> <s2> <g1> <g2> <lim>"); return;
    }
    axis[0] = {n1, s1 >= 0 ? 1.0f : -1.0f, g1};
    axis[1] = {n2, s2 >= 0 ? 1.0f : -1.0f, g2};
    limit_deg = fabsf(lim);
    print_config();
  } else if (c == 'W') {
    watchdog_ms = strtoul(s + 1, nullptr, 10);
    print_config();
  } else if (c == 'F') {
    telemetry_hz = strtoul(s + 1, nullptr, 10);
    print_config();
  } else if (c == 'M') {
    int p1, p2, p3, p4;
    if (sscanf(s + 1, "%d %d %d %d", &p1, &p2, &p3, &p4) != 4) { Serial.println("ERR M <p1> <p2> <p3> <p4>"); return; }
    const int16_t p[4] = {(int16_t)p1, (int16_t)p2, (int16_t)p3, (int16_t)p4};
    pol_stop("comando M");
    dsm_armed = false;
    dsm_stop("comando M");
    base_cancel();
    vel_mode = false;  // potencia cruda y modo velocidad son excluyentes
    set_motors(p);
    last_motor_ms = millis();
  } else if (c == 'V') {
    float w1, w2;
    if (sscanf(s + 1, "%f %f", &w1, &w2) != 2) { Serial.println("ERR V <w1> <w2>"); return; }
    pol_stop("comando V");
    dsm_armed = false;
    dsm_stop("comando V");
    base_cancel();
    if (!vel_mode) vel_start();
    vel_ref[0] = constrain(w1, -12.0f, 12.0f);  // rad/s; vacio medido ~11 rad/s a 100 %
    vel_ref[1] = constrain(w2, -12.0f, 12.0f);
    vel_last_cmd_ms = millis();
    // + cuentas de encoder del ultimo paso del PI (20-oct, para la odometria de pizarra_rl.py)
    Serial.printf("v,%lu,%.3f,%.3f,%d,%d,%ld,%ld\n", (unsigned long)vel_last_cmd_ms, vel_w[0], vel_w[1], motor_power[0],
                  motor_power[1], (long)vel_hist[0][0], (long)vel_hist[1][0]);
  } else if (c == 'Y') {
    vel_wd_ms = strtoul(s + 1, nullptr, 10);
    Serial.printf("OK Y vel_wd_ms=%lu\n", (unsigned long)vel_wd_ms);
  } else if (c == 'X') {
    pol_stop("comando X");
    dsm_armed = false;
    dsm_stop("comando X");
    base_cancel();
    vel_mode = false;
    stop_motors();
    Serial.println("OK X motores detenidos");
  } else if (c == 'B') {
    uint32_t t = micros();
    for (int i = 0; i < 20; ++i) prizm.getEncoderCount(1);
    const uint32_t enc_us = (micros() - t) / 20;
    t = micros();
    for (int i = 0; i < 20; ++i) prizm.setMotorPowers(0, 0, 0, 0);
    const uint32_t pw_us = (micros() - t) / 20;
    Serial.printf("OK B lectura_encoder_us=%lu escritura_potencia_us=%lu\n", (unsigned long)enc_us,
                  (unsigned long)pw_us);
  } else if (c == 'I') {
    Experiment e;
    int n = sscanf(s + 1, "%7s %d %f %f %f %f %f %f", e.type, &e.motor, &e.hz, &e.a[0], &e.a[1], &e.a[2], &e.a[3], &e.a[4]);
    if (n < 4 || e.motor < 1 || e.motor > 3) { Serial.println("ERR I <tipo> <motor> <hz> <a1> [a2] [a3] [a4]"); return; }
    pol_stop("comando I");
    dsm_armed = false;
    dsm_stop("comando I");
    base_cancel();
    vel_mode = false;
    if (!strcmp(e.type, "CLPI") || !strcmp(e.type, "CLINT") || !strcmp(e.type, "CHIRP")) run_closed_loop(e);
    else run_experiment(e);
  } else if (c == 'K') {
    int m; float kp, ki, gp, dp, gn, dn;
    if (sscanf(s + 1, "%d %f %f %f %f %f %f", &m, &kp, &ki, &gp, &dp, &gn, &dn) != 7 || m < 1 || m > 2) {
      Serial.println("ERR K <m> <kp> <ki> <g+> <dz+> <g-> <dz->"); return;
    }
    spi[m - 1] = {kp, ki, gp, dp, gn, dn};
    Serial.printf("OK K m%d kp=%.3f ki=%.2f g+=%.4f dz+=%.2f g-=%.4f dz-=%.2f\n", m, kp, ki, gp, dp, gn, dn);
  } else if (c == 'Z') {
    prizm.resetEncoders();
    Serial.println("OK Z encoders a cero");
  } else if (c == 'O') {
    unsigned long seq;
    float v[10];
    if (sscanf(s + 1, "%lu %f %f %f %f %f %f %f %f %f %f", &seq, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6],
               &v[7], &v[8], &v[9]) != 11) {
      Serial.println("ERR O <seq> <bx> <by> <vis> <pitch> <roll> <gx> <gy> <gz> <vx> <wz>"); return;
    }
    portENTER_CRITICAL(&pol_mux);
    memcpy(pol_in.cam, v, 5 * sizeof(float));
    memcpy(pol_in.grav, v + 5, 3 * sizeof(float));
    if (!rc_pol_cmd) {  // con el gamepad habilitado el comando vx / wz viene de las palancas (rc_tick)
      pol_in.cmd[0] = v[8];
      pol_in.cmd[1] = v[9];
    }
    pol_in.seq = seq;
    pol_in.ms = millis();
    portEXIT_CRITICAL(&pol_mux);
    pol_have_in = true;
    // gravedad (0,0,-1) = nivelado: angulo de vuelco = acos(-gz)
    if (acosf(constrain(-v[7], -1.0f, 1.0f)) > TIP_LIMIT_RAD) pol_stop("inclinacion > limite");
  } else if (c == 'A') {
    const int on = atoi(s + 1);
    if (!on) { pol_stop("comando A 0"); if (!pol_active) Serial.println("OK A 0"); return; }
    if (pol_init_err) { Serial.printf("ERR A politica no cargada (%d)\n", pol_init_err); return; }
    if (!pol_have_in || millis() - pol_in.ms > pol_wd_ms) { Serial.println("ERR A falta 'O' reciente"); return; }
    if (pol_active) { Serial.println("OK A 1 (ya activa)"); return; }
    dsm_armed = false;
    dsm_stop("comando A 1");
    base_cancel();
    if (!vel_mode) vel_start();
    vel_ref[0] = vel_ref[1] = 0;
    vel_last_cmd_ms = millis();
    portENTER_CRITICAL(&pol_mux);
    pol_reset = true;
    pol_out_new = false;
    portEXIT_CRITICAL(&pol_mux);
    pol_active = true;
    Serial.printf("OK A 1 %s %.0f Hz\n", POLICY_SOURCE, POL_HZ);
  } else if (c == 'U') {
    handle_base(s + 1);
  } else if (c == 'J') {
    handle_rc(s + 1);
  } else if (c == 'N') {
    net_cmd(s + 1);
  } else if (c == 'H') {
    handle_ball(s + 1);
  } else if (c == 'D') {
    handle_dsm(s + 1);
  } else if (c == 'E') {
    imu_cmd(s + 1);
  } else if (c == 'Q') {
    pol_selftest();
  } else if (c == 'G') {
    pol_stats();
  } else if (c == '?') {
    print_config();
  } else if (c != 0) {
    Serial.printf("ERR comando desconocido: %s\n", s);
  }
}

void read_serial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[line_len] = 0;
      handle_line(line);
      line_len = 0;
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = ch;
    } else {
      line_len = 0;  // linea demasiado larga: se descarta
      Serial.println("ERR linea larga");
    }
  }
}

void setup() {
  Serial.begin(115200);  // USB CDC: el baudrate no importa, pero se deja estandar
  Serial.setTxTimeoutMs(0);  // sin nadie leyendo el USB, los mensajes se descartan en vez de frenar el lazo (16-oct)
  delay(300);
  Serial.println("gimbal_fw: presiona el BOTON VERDE para armar los servos");
  pol_init();                         // pesos a RAM + tarea en el nucleo 0 (inactiva hasta 'A 1')
  Serial.printf("gimbal_fw: politica %s err=%d pesos_en_%s\n", POLICY_SOURCE, pol_init_err, pol_in_ram ? "RAM" : "FLASH");
  prizm.PrizmBegin();                 // bloquea hasta el boton verde
  prizm.setServoSpeeds(100, 100, 100, 100, 100, 100);  // sin rampa: la libreria usa 50% por defecto
  apply_tilt(0, 0);                   // arrancar nivelado
  imu_begin();                        // MPU6050 en el I2C de expansion: 1 s midiendo el sesgo, robot QUIETO
  h_prefs.begin("plato", false);
  h_kg = h_prefs.getFloat("kg", 0); h_ka = h_prefs.getFloat("ka", 0); h_kw = h_prefs.getFloat("kw", 0);
  h_tau = h_prefs.getFloat("tau", 0); h_q = h_prefs.getInt("q", 0);
  dsm_prefs.begin("dsm", false);
  dsm_acc = dsm_prefs.getFloat("acc", 0); dsm_fuse = dsm_prefs.getBool("fuse", false);
  dsm_dpsi_imu = dsm_prefs.getBool("dpsi", false);
  ekf_init(&ekf);
  last_cmd_ms = millis();
  state = IDLE;
  rc_begin();                         // receptor R/C en los puertos de sensor 2 y 3 (deshabilitado hasta 'J 1')
  Serial1.setRxBufferSize(1024);      // myRIO (modo DSM) en el puerto de sensor 1
  Serial1.setTxBufferSize(256);
  Serial1.begin(MYRIO_BAUD, SERIAL_8N1, MYRIO_RX, MYRIO_TX);
  net_begin();                        // WiFi (si quedo activado con 'N'): log y comandos por UDP
  Serial.println("gimbal_fw: listo");
  print_config();
}

void loop() {
  read_serial();
  net_tick(handle_line);  // comandos por WiFi (mismo interprete que el USB) y envio del log
  const uint32_t now = millis();

  if (state == RUNNING && watchdog_ms > 0 && now - last_cmd_ms > watchdog_ms) {
    apply_tilt(0, 0);  // sin consignas: nivelar
    state = WATCHDOG;
  }

  if (pol_active) {
    if (now - pol_in.ms > pol_wd_ms) pol_stop("sin 'O' (vision/RPi)");
    else if (pol_out_new) pol_apply();
  }

  imu_still = vel_mode ? fabsf(vel_ref[0]) < 0.01f && fabsf(vel_ref[1]) < 0.01f && fabsf(vel_w[0]) < 0.05f &&
                              fabsf(vel_w[1]) < 0.05f
                        : !motors_active;
  imu_tick();   // IMU del chasis a 200 Hz (antes de DSM: la respuesta al myRIO lleva su dpsi)
  dsm_tick();   // myRIO por UART (modo DSM)
  h_tick();     // pelota con la camara (modo H)
  if ((h_on || now - h_t_ms < 200) && now - h_apply_ms >= 10) { h_apply_ms = now; h_apply(); }  // compensacion 100 Hz
  base_tick();  // base (U) y gamepad (J): actualiza las consignas de rueda antes del PI

  if (vel_mode) {
    // con signo: dsm_start/base_tick/pol_apply ponen vel_last_cmd_ms DESPUES de tomar 'now' -> la resta sin signo
    // daba ~4e9 y cortaba el PI recien arrancado ("PI sin consignas" en cada arranque, 11-nov)
    if ((int32_t)(now - vel_last_cmd_ms) > (int32_t)vel_wd_ms) {
      if (dsm_run) dsm_stop("PI sin consignas (lazo atrasado)");  // si no, DSM queda 'activo' con los motores en 0
      vel_stop();  // se corto la politica: nunca dejar las ruedas con la ultima consigna
      base_cancel();
      Serial.println("WD V motores detenidos (sin comandos V)");
    } else {
      vel_step();
    }
  }

  if (!vel_mode && motors_active && (int32_t)(now - last_motor_ms) > (int32_t)motor_wd_ms) {
    stop_motors();  // se corto la comunicacion: nunca dejar motores girando
    Serial.println("WD motores detenidos (sin comandos M)");
  }

  if (now - last_batt_ms >= 500) {  // la bateria se lee por I2C: no en cada vuelta
    last_batt_ms = now;
    vbat = prizm.readBatteryVoltage();
  }
  static uint32_t last_bline_ms = 0;
  if (now - last_bline_ms >= 2000) {  // 09-nov: "B,<ms>,<V>" siempre (USB -> log de la Pi, WiFi -> dsm_gui): con la
    last_bline_ms = now;             // bateria baja la PRIZM se reinicia sola en plena prueba
    char b[32];
    snprintf(b, sizeof b, "B,%lu,%.2f", (unsigned long)now, vbat);
    Serial.println(b);
    net_send_line(b);
  }

  if (telemetry_hz > 0 && now - last_tel_ms >= 1000 / telemetry_hz) {
    last_tel_ms = now;
    Serial.printf("T,%lu,%.2f,%.2f,%u,%u,%.2f,%lu,%s\n", (unsigned long)now, cmd[0], cmd[1],
                  raw_out[0], raw_out[1], vbat, (unsigned long)(now - last_cmd_ms), state_name());
    Serial.printf("E,%lu,%d,%d,%d,%d,%ld,%ld,%ld,%ld\n", (unsigned long)now, motor_power[0],
                  motor_power[1], motor_power[2], motor_power[3], (long)read_enc(1),
                  (long)read_enc(2), (long)read_enc(3), (long)read_enc(4));
  }
}
