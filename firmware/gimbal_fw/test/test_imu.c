/*
 * Prueba en PC de imu_math.h (sin robot):
 *   gcc -O2 -Wall -I.. -o /tmp/test_imu test_imu.c -lm && /tmp/test_imu
 */
#include <assert.h>
#include <stdio.h>
#include "imu_math.h"

#define NEAR(a, b, tol) (fabsf((a) - (b)) < (tol))

int main(void) {
  const float D = IMU_RAD2DEG, dt = 0.005f;
  /* 1. acelerometro con el chasis en una rampa de 20 grados nariz arriba: el filtro converge a pitch 20 */
  float u[3] = {0, 0, 1}, w0[3] = {0, 0, 0};
  const float a20[3] = {IMU_G * sinf(20 / D), 0, IMU_G * cosf(20 / D)};
  for (int i = 0; i < 2000; ++i) att_step(u, w0, a20, dt, 0.5f);
  assert(NEAR(att_pitch(u) * D, 20, 0.1f) && NEAR(att_roll(u) * D, 0, 0.1f));

  /* 2. solo giroscopio (acelerometro saturado = ignorado): girar sobre +x 0.5 rad/s por 0.4 s = 0.2 rad lado izq arriba */
  float v[3] = {0, 0, 1}, wx[3] = {0.5f, 0, 0}, abig[3] = {0, 0, 3 * IMU_G};
  for (int i = 0; i < 80; ++i) att_step(v, wx, abig, dt, 0.5f);
  assert(NEAR(att_roll(v), 0.2f, 0.002f));

  /* 3. rumbo: 90 grados a la izquierda en la rampa de 20 grados. El giro del chasis alrededor de la vertical del mundo
   * se ve en el chasis como w = 1 rad/s * u; la suma de dpsi debe dar pi/2 (con w_z solo daria 6 % menos) */
  float r[3] = {sinf(20 / D), 0, cosf(20 / D)}, psi = 0;
  for (int i = 0; i < 314; ++i) {
    const float w[3] = {r[0], r[1], r[2]};
    psi += att_step(r, w, abig, dt, 0.5f);
  }
  assert(NEAR(psi, 1.57f, 0.01f) && NEAR(att_pitch(r) * D, 20, 0.1f));

  /* 4. compensacion: chasis nariz arriba 10 grados con el eje X del plato hacia adelante -> la vertical se ve hacia
   * +X y el plato debe dar pitch +10 (sube el borde -X = el de atras, el de abajo) para quedar horizontal */
  const float b[2] = {IMU_G * sinf(10 / D), 0};
  float fp[2], out[2];
  body_to_plate(0, b, fp);
  const float f[3] = {fp[0], fp[1], IMU_G * cosf(10 / D)}, des0[2] = {0, 0}, nof[2] = {0, 0};
  plate_comp(des0, f, nof, out);
  assert(NEAR(out[1], 10, 0.01f) && NEAR(out[0], 0, 0.01f));
  /* mismo chasis con el eje X del plato hacia la izquierda: ahora es roll; la vertical cae hacia -Y del plato */
  body_to_plate(1, b, fp);
  const float f1[3] = {fp[0], fp[1], IMU_G * cosf(10 / D)};
  plate_comp(des0, f1, nof, out);
  assert(NEAR(out[0], 10, 0.01f) && NEAR(out[1], 0, 0.01f));
  /* lo que pide el PD se suma */
  const float des[2] = {2, -3};
  plate_comp(des, f, nof, out);
  assert(NEAR(out[1], 7, 0.01f) && NEAR(out[0], 2, 0.01f));
  /* 5. brazo de palanca: girando en el sitio a 2 rad/s con la IMU 10 cm a la izquierda del centro, la IMU ve una
   * centripeta de w^2 r = 0.4 m/s2 hacia el centro (-y); llevada al origen debe desaparecer */
  const float wz[3] = {0, 0, 2}, dw0[3] = {0, 0, 0}, rim[3] = {0, -0.1f, 0};
  const float fi[3] = {0, -0.4f, IMU_G};
  float fo[3];
  lever_arm(fi, wz, dw0, rim, fo);
  assert(NEAR(fo[0], 0, 1e-5f) && NEAR(fo[1], 0, 1e-5f) && NEAR(fo[2], IMU_G, 1e-5f));
  /* acelerando el giro (dw = 3 rad/s2), un punto 0.1 m adelante del origen acelera +0.3 m/s2 en y (tangencial) */
  const float dw3[3] = {0, 0, 3}, w00[3] = {0, 0, 0}, rx[3] = {0.1f, 0, 0}, f0[3] = {0, 0, IMU_G};
  lever_arm(f0, w00, dw3, rx, fo);
  assert(NEAR(fo[1], 0.3f, 1e-5f) && NEAR(fo[0], 0, 1e-5f));
  /* 6. peso del acelerometro: a 1.15 g corrige la mitad de rapido que a 1 g (y a 1.3 g nada, caso 2) */
  float u1[3] = {0, 0, 1}, u2[3] = {0, 0, 1};
  const float a1[3] = {IMU_G * sinf(20 / D), 0, IMU_G * cosf(20 / D)};
  const float a2[3] = {1.15f * a1[0], 0, 1.15f * a1[2]};
  att_step(u1, w0, a1, dt, 0.5f);
  att_step(u2, w0, a2, dt, 0.5f);
  assert(NEAR(u2[0], 0.5f * u1[0], 1e-4f));
  printf("test_imu OK\n");
  return 0;
}
