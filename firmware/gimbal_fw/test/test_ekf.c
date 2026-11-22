/*
 * Prueba en PC de odo_ekf.h (sin robot): piso recto con rampa, ruedas en el aire, circulo, giro en el aire.
 *   gcc -O2 -Wall -I.. -o /tmp/test_ekf test_ekf.c -lm && /tmp/test_ekf
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "odo_ekf.h"

static float gauss(float s) {  /* Box-Muller con rand(): suficiente para una prueba */
  const float u = (rand() + 1.0f) / (RAND_MAX + 2.0f), v = (rand() + 1.0f) / (RAND_MAX + 2.0f);
  return s * sqrtf(-2 * logf(u)) * cosf(6.2831853f * v);
}

/* perfil de velocidad de las ruedas: rampa 0.4 m/s2 hasta 0.3, mantiene, rampa abajo (como con 'D A') */
static float perfil(float t, float* acc) {
  const float a = 0.4f, vm = 0.3f, tr = vm / a, th = 3.0f;
  if (t < tr) { *acc = a; return a * t; }
  if (t < tr + th) { *acc = 0; return vm; }
  if (t < 2 * tr + th) { *acc = -a; return vm - a * (t - tr - th); }
  *acc = 0;
  return 0;
}

int main(void) {
  const float dt = 0.01f, T = 5.5f;
  OdoEkf f;
  srand(1);
  setvbuf(stdout, NULL, _IONBF, 0);

  /* 1. piso, recto: ruedas = realidad, acelerometro con sesgo 0.1 m/s2 (inclinacion mal estimada ~0.6 grados) */
  ekf_init(&f);
  for (int i = 0; i < 100; ++i) ekf_step(&f, 0, 0, gauss(0.005f), 0.1f + gauss(0.1f), gauss(0.1f), dt);  /* 1 s quieto */
  float err_max = 0, dist_true = 0, dist_ekf = 0;
  for (float t = 0; t < T; t += dt) {
    float acc;
    const float v = perfil(t, &acc);
    const float vx = ekf_step(&f, v + gauss(0.01f), gauss(0.02f), gauss(0.005f), acc + 0.1f + gauss(0.1f), gauss(0.1f), dt);
    if (fabsf(vx - v) > err_max) err_max = fabsf(vx - v);
    dist_true += v * dt;
    dist_ekf += vx * dt;
  }
  printf("piso:    err max %.3f m/s  distancia %.3f vs %.3f m  rechazos v=%lu w=%lu perdidas=%lu  sesgo est. %.3f\n",
         err_max, dist_ekf, dist_true, f.n_rej_v, f.n_rej_w, f.n_lost, f.x[2]);
  assert(err_max < 0.04f && fabsf(dist_ekf - dist_true) < 0.02f && f.n_rej_v < 10 && f.n_rej_w < 10);

  /* 2. ruedas en el aire: los encoders hacen el mismo perfil, la IMU no ve nada (tras 1 s quieto, como al arrancar) */
  ekf_init(&f);
  for (int i = 0; i < 100; ++i) ekf_step(&f, 0, 0, gauss(0.005f), gauss(0.1f), gauss(0.1f), dt);
  float vmax = 0;
  dist_ekf = 0;
  int rej_hold = 0, n_hold = 0;
  for (float t = 0; t < 4.5f; t += dt) {  /* hasta la mitad de la rampa de bajada */
    float acc;
    const float v = perfil(t, &acc);
    const float vx = ekf_step(&f, v + gauss(0.01f), gauss(0.02f), gauss(0.005f), gauss(0.1f), gauss(0.1f), dt);
    if (fabsf(vx) > vmax) vmax = fabsf(vx);
    dist_ekf += vx * dt;
    if (t > 1.0f && t < 3.75f) { ++n_hold; rej_hold += f.rej_v; }
  }
  printf("aire:    |vx| max %.3f m/s  distancia %.3f m (ruedas ~1.1 m)  rechazo en meseta %d/%d  perdidas=%lu\n", vmax,
         dist_ekf, rej_hold, n_hold, f.n_lost);
  assert(vmax < 0.12f && fabsf(dist_ekf) < 0.25f && rej_hold == n_hold);  /* ventana previa + deriva de integrar la IMU */
  /* ... y al parar las ruedas vuelve a creerle a los encoders */
  for (int i = 0; i < 50; ++i) ekf_step(&f, 0, 0, 0, gauss(0.1f), 0, dt);
  for (int i = 0; i < 100; ++i) ekf_step(&f, 0.05f * i / 100.0f, 0, 0, 0.05f, 0, dt);  /* arranca apoyado, 0.05 m/s2 */
  printf("vuelta:  vx %.3f (ruedas 0.050)  rechazo=%d\n", f.x[0], f.rej_v);
  assert(fabsf(f.x[0] - 0.05f) < 0.02f && !f.rej_v);

  /* 3. circulo en el piso: v 0.2, w 0.5 -> centripeta ay = w v = 0.1 hacia la izquierda; vy debe quedar en ~0 */
  ekf_init(&f);
  for (int i = 0; i < 300; ++i) ekf_step(&f, 0.2f * fminf(1, i / 50.0f), 0, 0, i < 50 ? 0.4f : 0, 0, dt);
  float vy_max = 0;
  for (int i = 0; i < 1000; ++i) {
    ekf_step(&f, 0.2f + gauss(0.01f), 0.5f + gauss(0.02f), 0.5f + gauss(0.005f), gauss(0.1f), 0.1f + gauss(0.1f), dt);
    if (fabsf(f.x[1]) > vy_max) vy_max = fabsf(f.x[1]);
  }
  printf("circulo: vx %.3f  |vy| max %.4f  rechazos v=%lu w=%lu\n", f.x[0], vy_max, f.n_rej_v, f.n_rej_w);
  assert(fabsf(f.x[0] - 0.2f) < 0.03f && vy_max < 0.02f && f.n_rej_w < 10);

  /* 4. giro en el aire: los encoders dicen 1 rad/s, el giroscopio 0 -> giro rechazado en cada muestra */
  ekf_init(&f);
  int rw = 0;
  for (int i = 0; i < 100; ++i) { ekf_step(&f, 0, 1.0f, gauss(0.005f), gauss(0.1f), 0, dt); rw += f.rej_w; }
  printf("giro aire: rechazos de giro %d/100\n", rw);
  assert(rw == 100);
  printf("OK\n");
  return 0;
}
