// Observacion de DSM-BallTrack-v0 (31-oct), igual que el ObservationManager de IsaacLab con history_length 5:
// cada termino con su historial (del mas viejo al mas nuevo) y los terminos concatenados en este orden.
//   camera (u, v, visible) x10 | projected_gravity 3 | wheel_vel (izq, der) x0.1 | trajectory (e_x, e_y, sin e_th,
//   cos e_th, v_r, w_r) | last_action (rueda izq, rueda der, plato pitch, plato roll)
// Al primer paso (o tras obs_bt_reset) el historial se llena con ese paso, como el CircularBuffer de IsaacLab.
#pragma once
#include <string.h>

#define OBT_STEP 18
#define OBT_HIST 5
#define OBT_DIM (OBT_STEP * OBT_HIST)

typedef struct {
  float h[OBT_HIST][OBT_STEP];  // [0] = mas viejo
  int empty;
} ObsBT;

static inline void obs_bt_reset(ObsBT* o) { o->empty = 1; }

// step sin escalar: [u v vis | gx gy gz | w_izq w_der (rad/s) | ex ey sin cos vr wr | a0 a1 a2 a3]
static inline void obs_bt_push(ObsBT* o, const float step[OBT_STEP], float out[OBT_DIM]) {
  static const int dim[5] = {3, 3, 2, 6, 4}, off[5] = {0, 3, 6, 8, 14};
  static const float sc[5] = {10.0f, 1.0f, 0.1f, 1.0f, 1.0f};
  float v[OBT_STEP];
  for (int t = 0; t < 5; ++t)
    for (int d = 0; d < dim[t]; ++d) v[off[t] + d] = step[off[t] + d] * sc[t];
  if (o->empty) {
    for (int k = 0; k < OBT_HIST; ++k) memcpy(o->h[k], v, sizeof v);
    o->empty = 0;
  } else {
    memmove(o->h[0], o->h[1], sizeof(float) * OBT_STEP * (OBT_HIST - 1));
    memcpy(o->h[OBT_HIST - 1], v, sizeof v);
  }
  int n = 0;
  for (int t = 0; t < 5; ++t)
    for (int k = 0; k < OBT_HIST; ++k)
      for (int d = 0; d < dim[t]; ++d) out[n++] = o->h[k][off[t] + d];
}
