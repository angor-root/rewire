/*
 * Cuentas de la IMU del chasis y de la compensacion del plato (C puro: se prueba en el PC con test/test_imu.c).
 * Ejes del CHASIS (28-oct): origen en el punto medio del eje de las RUEDAS ACTUADAS, a la altura del piso;
 * x adelante = hacia el lado de las ruedas actuadas (el mismo "adelante" del myRIO y de los encoders), y izquierda,
 * z arriba. u = vector unitario "arriba" del mundo visto desde el chasis (lo que lee el acelerometro quieto,
 * normalizado): nivelado u = (0, 0, 1).
 */
#pragma once
#include <math.h>

#define IMU_G 9.80665f
#define IMU_RAD2DEG 57.29578f

static inline float v3_len(const float v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

static inline void v3_cross(const float a[3], const float b[3], float o[3]) {
  const float x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0];
  o[0] = x; o[1] = y; o[2] = z;
}

/* Brazo de palanca en un cuerpo rigido: fuerza especifica en el punto r (respecto al punto donde se midio f), con giro
 * w y aceleracion angular dw:  f_r = f + dw x r + w x (w x r).  Con r = -r_imu lleva la medida al origen (girando en el
 * sitio, la IMU de costado ve una centripeta que el centro no tiene); con r = r_plato - r_imu, al pivote del plato. */
static inline void lever_arm(const float f[3], const float w[3], const float dw[3], const float r[3], float out[3]) {
  float t[3], wr[3], wwr[3];
  v3_cross(dw, r, t);
  v3_cross(w, r, wr);
  v3_cross(w, wr, wwr);
  for (int i = 0; i < 3; ++i) out[i] = f[i] + t[i] + wwr[i];
}

/* Un paso del filtro complementario. w: giro del chasis (rad/s, sin sesgo), a: acelerometro (m/s2).
 * El mundo esta fijo y el chasis gira: du/dt = u x w. Luego u se acerca a a/|a| con constante tau (s), con un peso que
 * baja de 1 (|a| = g) a 0 (|a| a 0.3 g de g o mas): con aceleraciones fuertes el acelerometro no indica la vertical.
 * Devuelve el giro alrededor de la VERTICAL del mundo en este paso (rad, + = a la izquierda visto desde arriba):
 * en una rampa no es w_z sino la proyeccion de w sobre u. */
static inline float att_step(float u[3], const float w[3], const float a[3], float dt, float tau) {
  const float d[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]};
  const float dpsi = (w[0] * u[0] + w[1] * u[1] + w[2] * u[2]) * dt;
  for (int i = 0; i < 3; ++i) u[i] += d[i] * dt;
  const float an = v3_len(a), wgt = 1.0f - fabsf(an - IMU_G) / (0.3f * IMU_G);
  if (wgt > 0) {
    const float k = wgt * dt / tau;
    for (int i = 0; i < 3; ++i) u[i] += k * (a[i] / an - u[i]);
  }
  const float n = v3_len(u);
  for (int i = 0; i < 3; ++i) u[i] /= n;
  return dpsi;
}

/* Inclinacion del chasis desde u (rad): pitch + = nariz arriba, roll + = lado izquierdo arriba. */
static inline float att_pitch(const float u[3]) { return atan2f(u[0], sqrtf(u[1] * u[1] + u[2] * u[2])); }
static inline float att_roll(const float u[3]) { return atan2f(u[1], u[2]); }

/* Vector del plano del chasis a ejes del plato. q = hacia donde apunta el eje X del plato:
 * 0 adelante (+x), 1 izquierda (+y), 2 atras, 3 derecha. El eje Z del plato es el z del chasis. */
static inline void body_to_plate(int q, const float b[2], float p[2]) {
  switch (q & 3) {
    case 0: p[0] = b[0]; p[1] = b[1]; break;
    case 1: p[0] = b[1]; p[1] = -b[0]; break;
    case 2: p[0] = -b[0]; p[1] = -b[1]; break;
    default: p[0] = -b[1]; p[1] = b[0]; break;
  }
}

/* Inclinacion del plato respecto al CHASIS (grados, convencion del CAD: roll + empuja la pelota a -Y, pitch + a +X)
 * que le da a la pelota la aceleracion que pide el PD (des = roll, pitch que el PD daria en un plato quieto y plano)
 * anulando las perturbaciones:
 *   f[3]   fuerza especifica en ejes del plato (gravedad + aceleracion del carro: lo que leeria un acelerometro en el
 *          plato), m/s2. Quieto y plano: (0, 0, g).
 *   fict[2] aceleraciones ficticias de la pelota por el giro del carro (centrifuga + Coriolis), m/s2.
 * La pelota rueda hacia -f + fict; el plato se inclina el angulo de esa "vertical efectiva" mas lo que pide el PD. */
static inline void plate_comp(const float des[2], const float f[3], const float fict[2], float out[2]) {
  out[1] = des[1] + atan2f(f[0] - fict[0], f[2]) * IMU_RAD2DEG;
  out[0] = des[0] - atan2f(f[1] - fict[1], f[2]) * IMU_RAD2DEG;
}
