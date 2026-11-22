/*
 * Prueba en PC de base_kin (sin robot). Compilar y correr:
 *   gcc -O2 -Wall -I.. -o /tmp/test_base_kin test_base_kin.c ../base_kin.c -lm && /tmp/test_base_kin [csv]
 *
 * 1. IK <-> FK son inversas y la saturacion conserva la curvatura.
 * 2. Odometria: un giro en el sitio de 2*pi con la via correcta vuelve a th = 0.
 * 3. Circulo en lazo cerrado sobre una planta "real" distinta del modelo: rueda de 1er orden (tau 60 ms, como el PI
 *    de 2 GDL de gimbal_fw), encoder de 1440 CPR, odometria a 100 Hz, control a 50 Hz como en el firmware,
 *    y el robot arrancando 3 cm y 10 grados fuera de la tangente.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "base_kin.h"

static int fails = 0;
#define CHECK(cond, ...)                \
  do {                                  \
    if (!(cond)) {                      \
      printf("FALLA: " __VA_ARGS__);    \
      printf("\n");                     \
      ++fails;                          \
    }                                   \
  } while (0)

static void test_ik_fk(void) {
  const BaseGeom g = BASE_GEOM_DEFAULT;
  float wl, wr, v, w;
  base_ik(&g, 0.2f, 0.5f, &wl, &wr);
  base_fk(&g, wl, wr, &v, &w);
  CHECK(fabsf(v - 0.2f) < 1e-5f && fabsf(w - 0.5f) < 1e-5f, "IK/FK no son inversas (%f %f)", v, w);
  // saturacion: 1 m/s no llega (10 rad/s * 0.0514 = 0.514 m/s); la curvatura w/v se conserva
  const int sat = base_ik(&g, 1.0f, 2.0f, &wl, &wr);
  base_fk(&g, wl, wr, &v, &w);
  CHECK(sat && fmaxf(fabsf(wl), fabsf(wr)) <= g.w_max + 1e-4f, "no satura");
  CHECK(fabsf(w / v - 2.0f) < 1e-4f, "la saturacion cambia la curvatura (%f)", w / v);
  printf("IK/FK ok: v=0.2 w=0.5 -> wL=%.3f wR=%.3f rad/s\n", (0.2f - 0.5f * g.track / 2) / g.r,
         (0.2f + 0.5f * g.track / 2) / g.r);
}

static void test_odom_spin(void) {
  const BaseGeom g = BASE_GEOM_DEFAULT;
  Pose2 p = {0, 0, 0};
  // una vuelta en el sitio: cada rueda recorre pi*b -> angulo de rueda pi*b/r
  const float phi = 3.14159265f * g.track / g.r;
  for (int i = 0; i < 1000; ++i) base_odom(&g, &p, -phi / 1000, phi / 1000);
  CHECK(fabsf(p.th) < 1e-3f && hypotf(p.x, p.y) < 1e-4f, "giro en el sitio no vuelve (%f %f %f)", p.x, p.y, p.th);
  printf("odometria ok: giro en el sitio de 360 grados = %.2f vueltas de rueda\n", phi / 6.2831853f);
}

/* planta: unicycle con ruedas de 1er orden y geometria REAL (puede diferir del modelo) */
typedef struct { BaseGeom g; Pose2 p; float wl, wr; double enc_l, enc_r; } Plant;

static double run_circle(float radius, float speed, int dir, float track_real, float tau, FILE* csv, double* real_r_err) {
  const BaseGeom model = BASE_GEOM_DEFAULT;
  Plant pl = {BASE_GEOM_DEFAULT, {0, 0, 0}, 0, 0, 0, 0};
  pl.g.track = track_real;
  Pose2 odo = {0, 0, 0};
  const Pose2 start = {-0.03f, 0.02f, -0.1745f};  // el circulo arranca 3 cm y 10 grados fuera de donde esta el robot
  CircleTraj c;
  circle_init(&c, radius, speed, dir, 2.0f);
  circle_start(&c, &start);
  const float dt_sim = 0.001f, dt_odo = 0.01f, dt_ctl = 0.02f;
  const float cnt = 6.2831853f / 1440.0f;
  float ref_l = 0, ref_r = 0;
  long last_l = 0, last_r = 0;
  double max_e = 0, sum_e = 0;
  int n_e = 0;
  for (int k = 0; k < 60000 && !(c.done && fabsf(pl.wl) < 0.05f && fabsf(pl.wr) < 0.05f); ++k) {
    const float t = k * dt_sim;
    if (k % (int)(dt_odo / dt_sim + 0.5f) == 0) {
      const long el = (long)floor(pl.enc_l / cnt), er = (long)floor(pl.enc_r / cnt);
      base_odom(&model, &odo, (el - last_l) * cnt, (er - last_r) * cnt);
      last_l = el;
      last_r = er;
    }
    if (k % (int)(dt_ctl / dt_sim + 0.5f) == 0) {
      float v, w;
      circle_step(&c, &odo, dt_ctl, &v, &w);
      base_ik(&model, v, w, &ref_l, &ref_r);
      if (!c.done) {
        const double e = hypot(c.ref.x - odo.x, c.ref.y - odo.y);
        if (c.s > 0.5 * radius) {  // despues de corregir el error inicial
          if (e > max_e) max_e = e;
          sum_e += e;
          ++n_e;
        }
      }
      if (csv) fprintf(csv, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", t, odo.x, odo.y, odo.th, c.ref.x, c.ref.y,
                       pl.p.x, pl.p.y, pl.p.th);
    }
    // planta
    const float a = dt_sim / tau;
    pl.wl += a * (ref_l - pl.wl);
    pl.wr += a * (ref_r - pl.wr);
    pl.enc_l += pl.wl * dt_sim;
    pl.enc_r += pl.wr * dt_sim;
    base_odom(&pl.g, &pl.p, pl.wl * dt_sim, pl.wr * dt_sim);
  }
  if (real_r_err) {
    // con la via real distinta, la odometria miente: el circulo real deriva. Error = posicion final real vs inicial
    *real_r_err = hypot(pl.p.x - start.x, pl.p.y - start.y);
  }
  printf("  R=%.2f m v=%.2f m/s dir=%+d via_real=%.3f: error de seguimiento (odometria) medio %.1f mm, max %.1f mm;"
         " termino en %s tras %.2f vueltas\n",
         radius, speed, dir, track_real, 1000 * sum_e / (n_e ? n_e : 1), 1000 * max_e, c.done ? "reposo" : "TIMEOUT",
         c.s / (6.2831853 * radius));
  CHECK(c.done, "no termino");
  return max_e;
}

int main(int argc, char** argv) {
  test_ik_fk();
  test_odom_spin();
  FILE* csv = argc > 1 ? fopen(argv[1], "w") : NULL;
  if (csv) fprintf(csv, "t,x,y,th,xr,yr,x_real,y_real,th_real\n");
  printf("circulo, 2 vueltas, arranque desplazado 3 cm / 10 grados:\n");
  double drift;
  double e = run_circle(0.5f, 0.15f, +1, 0.364f, 0.06f, csv, &drift);
  CHECK(e < 0.02, "error de seguimiento > 2 cm (%.3f)", e);
  if (csv) fclose(csv);
  e = run_circle(0.5f, 0.15f, -1, 0.364f, 0.06f, NULL, &drift);
  CHECK(e < 0.02, "error > 2 cm horario");
  e = run_circle(0.3f, 0.25f, +1, 0.364f, 0.10f, NULL, &drift);
  CHECK(e < 0.03, "error > 3 cm con R 0.3 y planta lenta");
  e = run_circle(0.5f, 0.15f, +1, 0.364f * 1.05f, 0.06f, NULL, &drift);
  printf("  -> con la via real 5 %% mayor, el seguimiento sobre la odometria sigue bien, pero en el MUNDO el robot\n"
         "     termina a %.0f mm de donde empezo: por eso hay que calibrar la via (U CAL) antes de juzgar el circulo.\n",
         1000 * drift);
  printf(fails ? "\n%d FALLAS\n" : "\nTODO OK\n", fails);
  return fails != 0;
}
