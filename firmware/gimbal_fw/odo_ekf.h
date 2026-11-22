/*
 * Fusion encoders + IMU para la base diferencial (28-oct). C puro: se prueba en el PC con test/test_ekf.c.
 *
 * Estado x = [vx, vy, ba]: velocidad del chasis en SUS ejes (x adelante, y izquierda; m/s) y sesgo del acelerometro
 * en x (m/s2). El giro no es estado: lo da el giroscopio (mejor que los encoders, que no ven el patinaje).
 *   Prediccion (IMU, aceleracion lineal sin gravedad en el origen del chasis):
 *     vx' = vx + (ax - ba + w*vy) dt        vy' = vy + (ay - w*vx) dt        ba' = ba
 *   Correcciones:
 *     vy = 0                 restriccion no holonomica (la rueda no se desliza de costado): siempre
 *     vx = r(wR + wL)/2      encoders, si pasan dos pruebas chi2 (99.9 %):
 *                            - por muestra: la innovacion (saltos bruscos)
 *                            - por ventana de 0.3 s: cambio de velocidad de las ruedas contra la integral de la IMU. Hace
 *                              falta porque en una rampa cada paso difiere poco y el filtro se dejaria arrastrar (y
 *                              culparia al sesgo). Si no coinciden -> "sin contacto": se vuelve al estado del inicio de la
 *                              ventana mas lo que integro la IMU, y los encoders se ignoran hasta que coincidan 0.2 s
 *                              con la velocidad que integro la IMU (+-5 cm/s) o hasta que las ruedas paren
 *     ruedas quietas 0.2 s   vx = 0 sin test: el robot apoyado no se mueve con las ruedas paradas (y saca al filtro de
 *                            cualquier rechazo largo en cada parada)
 * Limite que ningun filtro salva: a velocidad CONSTANTE la IMU no distingue "en el aire" de "avanzando". El rechazo se
 * decide en las aceleraciones (por eso la rampa de 'D A') y despues se mantiene enclavado: si el robot ya iba a velocidad
 * constante cuando lo levantan, no se entera hasta el siguiente cambio de velocidad o de giro.
 */
#pragma once
#include <math.h>

#define EKF_WIN 30  /* muestras de la ventana (0.3 s a 100 Hz) */

typedef struct {
  float x[3];     /* vx, vy, ba */
  float P[3][3];
  float sa, sba;  /* ruido del acelerometro (m/s2) y deriva de su sesgo (m/s2/sqrt(s)) */
  float sv, sw;   /* ruido de la velocidad de los encoders (m/s) y del giro de los encoders (rad/s, + 10 % de |w|) */
  float slat;     /* tolerancia de la restriccion lateral (m/s) */
  float gate;     /* umbral chi2 de 1 GDL */
  float still_s;  /* tiempo con las ruedas quietas */
  float nis_v;    /* ultima innovacion normalizada de vx (diagnostico) */
  float nis_win;  /* ultima prueba de la ventana (diagnostico) */
  int rej_v, rej_w;   /* 1 = esta muestra se descarto (traslacion / giro) */
  int nocontact, n_ok; /* encoders ignorados hasta n_ok aceptaciones seguidas */
  unsigned long n_rej_v, n_rej_w, n_lost;
  /* ventana: velocidad de ruedas, aceleracion corregida integrada, estado del filtro */
  float wv[EKF_WIN], wa[EKF_WIN], wx0[EKF_WIN], wb[EKF_WIN];
  int wk, wn;
} OdoEkf;

static inline void ekf_init(OdoEkf* f) {
  for (int i = 0; i < 3; ++i) {
    f->x[i] = 0;
    for (int j = 0; j < 3; ++j) f->P[i][j] = 0;
  }
  f->P[0][0] = f->P[1][1] = 1e-4f;
  f->P[2][2] = 0.01f; /* sesgo inicial ~0.1 m/s2 (inclinacion mal estimada de ~0.6 grados); con las ruedas quietas se aprende */
  f->sa = 0.3f; f->sba = 0.005f; f->sv = 0.02f; f->sw = 0.05f; f->slat = 0.01f; f->gate = 10.83f;
  f->still_s = 0; f->nis_v = f->nis_win = 0; f->rej_v = f->rej_w = 0; f->n_rej_v = f->n_rej_w = f->n_lost = 0;
  f->nocontact = f->n_ok = 0; f->wk = f->wn = 0;
}

static inline void ekf_predict(OdoEkf* f, float ax, float ay, float w, float dt) {
  const float vx = f->x[0], vy = f->x[1], ba = f->x[2];
  f->x[0] = vx + (ax - ba + w * vy) * dt;
  f->x[1] = vy + (ay - w * vx) * dt;
  const float F[3][3] = {{1, w * dt, -dt}, {-w * dt, 1, 0}, {0, 0, 1}};
  float FP[3][3], P[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) FP[i][j] = F[i][0] * f->P[0][j] + F[i][1] * f->P[1][j] + F[i][2] * f->P[2][j];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) P[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];
  const float qa = f->sa * f->sa * dt * dt;
  P[0][0] += qa;
  P[1][1] += qa;
  P[2][2] += f->sba * f->sba * dt;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) f->P[i][j] = P[i][j];
}

/* correccion escalar z = x[k] con varianza R; gate > 0: descarta si innovacion^2/S > gate. Devuelve 1 si se aplico.
 * fix_b: no tocar el sesgo (filtro de Schmidt): en movimiento el sesgo NO se aprende de los encoders, porque si no el
 * filtro "explica" con un sesgo falso cualquier aceleracion de las ruedas que la IMU no vio (ruedas en el aire). */
static inline int ekf_update1(OdoEkf* f, int k, float z, float R, float gate, float* nis, int fix_b) {
  const float y = z - f->x[k], S = f->P[k][k] + R;
  const float d = y * y / S;
  if (nis) *nis = d;
  if (gate > 0 && d > gate) return 0;
  float K[3], Pk[3], P[3][3];
  for (int i = 0; i < 3; ++i) { K[i] = f->P[i][k] / S; Pk[i] = f->P[k][i]; }
  if (fix_b) K[2] = 0;
  /* P - K Pk - Pk' K' + K S K' (con K optima es P - K Pk; con K[2] = 0 sigue siendo la covarianza correcta) */
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) P[i][j] = f->P[i][j] - K[i] * Pk[j] - Pk[i] * K[j] + K[i] * S * K[j];
  for (int i = 0; i < 3; ++i) {
    f->x[i] += K[i] * y;
    for (int j = 0; j < 3; ++j) f->P[i][j] = P[i][j];
  }
  return 1;
}

/* Un paso a la frecuencia del PI (100 Hz). v_enc, w_enc: velocidad y giro segun los encoders; w_gyro: giro del chasis
 * alrededor de su z (rad/s); ax, ay: aceleracion lineal sin gravedad en el origen (m/s2). Devuelve vx fusionada. */
static inline float ekf_step(OdoEkf* f, float v_enc, float w_enc, float w_gyro, float ax, float ay, float dt) {
  /* ventana: lo que habia hace EKF_WIN pasos (el indice wk es la muestra mas vieja cuando la ventana esta llena) */
  const int full = f->wn == EKF_WIN, k = f->wk;
  float sum_a = 0;
  for (int i = 0; i < f->wn; ++i) sum_a += f->wa[i];
  const float v_old = f->wv[k], x_old = f->wx0[k], b_old = f->wb[k];

  ekf_predict(f, ax, ay, w_gyro, dt);
  ekf_update1(f, 1, 0.0f, f->slat * f->slat, 0, 0, 1);  /* vy = 0 */
  const float sw = f->sw + 0.1f * fabsf(w_gyro);
  f->rej_w = (w_enc - w_gyro) * (w_enc - w_gyro) / (sw * sw) > f->gate;
  if (f->rej_w) ++f->n_rej_w;

  /* prueba de la ventana: cambio de velocidad segun ruedas contra segun la IMU */
  const float a_c = (ax - f->x[2] + w_gyro * f->x[1]) * dt;
  if (full && !f->nocontact) {
    const float m = (v_enc - v_old) - (sum_a + a_c - f->wa[k]);
    const float s2 = 2 * f->sv * f->sv + EKF_WIN * (f->sa * dt) * (f->sa * dt);
    f->nis_win = m * m / s2;
    if (f->nis_win > f->gate) {  /* sin contacto: volver al inicio de la ventana + lo que integro la IMU */
      f->nocontact = 1;
      f->n_ok = 0;
      ++f->n_lost;
      f->x[0] = x_old + sum_a + a_c - f->wa[k];
      f->x[2] = b_old;
    }
  }

  f->still_s = (fabsf(v_enc) < 0.005f && fabsf(w_enc) < 0.02f) ? f->still_s + dt : 0;
  if (f->still_s > 0.2f) {  /* ruedas quietas: vx = 0 sin test */
    ekf_update1(f, 0, 0.0f, 1e-6f, 0, &f->nis_v, 0);  /* aqui si se aprende el sesgo */
    f->rej_v = 0;
    f->nocontact = 0;
  } else {
    if (f->nocontact) {
      /* sin contacto los encoders no se usan. Sale cuando lo que integro la IMU coincide con las ruedas 0.2 s (p. ej.
       * al apoyarlo con las ruedas girando: acelera de verdad hasta su velocidad). Con un umbral FIJO y no con la
       * covarianza, que crece con el tiempo y terminaria aceptando las ruedas en el aire a velocidad constante. */
      f->n_ok = fabsf(v_enc - f->x[0]) < 0.05f ? f->n_ok + 1 : 0;
      if (f->n_ok >= 20) f->nocontact = 0;
      f->rej_v = 1;
    } else {
      f->rej_v = !ekf_update1(f, 0, v_enc, f->sv * f->sv, f->gate, &f->nis_v, 1);
    }
    if (f->rej_v) ++f->n_rej_v;
  }

  f->wv[k] = v_enc;  /* la nueva muestra reemplaza a la mas vieja */
  f->wa[k] = a_c;
  f->wx0[k] = f->x[0];
  f->wb[k] = f->x[2];
  f->wk = (k + 1) % EKF_WIN;
  if (f->wn < EKF_WIN) ++f->wn;
  return f->x[0];
}
