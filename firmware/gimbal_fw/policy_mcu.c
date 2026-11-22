#pragma GCC optimize("O3")
#include "policy_mcu.h"

#include <math.h>
#include <string.h>

int mcu_policy_init(McuPolicy* p, const void* blob, size_t bytes) {
  memset(p, 0, sizeof *p);
  const uint8_t* u8 = (const uint8_t*)blob;
  uint32_t hdr[2];
  if (bytes < 12 || memcmp(u8, "DSMP", 4)) return -1;
  memcpy(hdr, u8 + 4, 8);
  if (hdr[0] != 1 || hdr[1] == 0 || hdr[1] > MCU_POLICY_MAX_LAYERS) return -2;
  p->n_layers = (int)hdr[1];
  size_t off = 12;
  for (int i = 0; i <= p->n_layers; ++i) {
    uint32_t d;
    memcpy(&d, u8 + off, 4);
    off += 4;
    if (d == 0 || d > MCU_POLICY_MAX_DIM) return -3;
    p->dims[i] = (int)d;
  }
  size_t need = off + 2 * sizeof(float) * p->dims[0];
  for (int l = 0; l < p->n_layers; ++l) need += sizeof(float) * ((size_t)p->dims[l + 1] * p->dims[l] + p->dims[l + 1]);
  if (bytes != need) return -4;
  const float* fp = (const float*)(u8 + off);  // off es múltiplo de 4
  p->mean = fp; fp += p->dims[0];
  p->std = fp; fp += p->dims[0];
  for (int l = 0; l < p->n_layers; ++l) {
    p->w[l] = fp; fp += (size_t)p->dims[l + 1] * p->dims[l];
    p->b[l] = fp; fp += p->dims[l + 1];
  }
  return 0;
}

static float buf[2][MCU_POLICY_MAX_DIM];

void mcu_policy_run(const McuPolicy* p, const float* obs, float* act) {
  float* x = buf[0];
  float* y = buf[1];
  for (int i = 0; i < p->dims[0]; ++i) x[i] = (obs[i] - p->mean[i]) / p->std[i];
  for (int l = 0; l < p->n_layers; ++l) {
    const int in = p->dims[l], out = p->dims[l + 1];
    const int last = l == p->n_layers - 1;
    float* dst = last ? act : y;
    for (int o = 0; o < out; ++o) {
      const float* w = p->w[l] + (size_t)o * in;
      // 4 acumuladores: el FPU del LX7 encadena madd.s sin esperar al resultado anterior
      float s0 = 0.f, s1 = 0.f, s2 = 0.f, s3 = 0.f;
      int i = 0;
      for (; i + 4 <= in; i += 4) {
        s0 += w[i] * x[i];
        s1 += w[i + 1] * x[i + 1];
        s2 += w[i + 2] * x[i + 2];
        s3 += w[i + 3] * x[i + 3];
      }
      for (; i < in; ++i) s0 += w[i] * x[i];
      const float s = p->b[l][o] + ((s0 + s1) + (s2 + s3));
      dst[o] = last ? s : (s > 0.f ? s : expm1f(s));  // ELU(alpha=1)
    }
    float* t = x; x = y; y = t;
  }
}

// Término -> (inicio en el paso, dimensión). Mismo orden que TERMS en obs.py.
static const int TERM_OFF[5] = {0, 5, 8, 10, 13};
static const int TERM_DIM[5] = {5, 3, 2, 3, 4};

void obs_reset(ObsHist* h) { h->empty = 1; }

void obs_push(ObsHist* h, const float step[OBS_STEP], float out[OBS_DIM]) {
  float v[OBS_STEP];
  memcpy(v, step, sizeof v);
  v[8] *= 0.1f;  // wheel_vel x0.1
  v[9] *= 0.1f;
  if (h->empty) {
    for (int k = 0; k < OBS_HISTORY; ++k) memcpy(h->hist[k], v, sizeof v);
    h->empty = 0;
  } else {
    memmove(h->hist[0], h->hist[1], sizeof(float) * OBS_STEP * (OBS_HISTORY - 1));
    memcpy(h->hist[OBS_HISTORY - 1], v, sizeof v);
  }
  // cada término aplanado del más viejo al más nuevo, luego se concatenan los términos
  int n = 0;
  for (int t = 0; t < 5; ++t)
    for (int k = 0; k < OBS_HISTORY; ++k)
      for (int d = 0; d < TERM_DIM[t]; ++d) out[n++] = h->hist[k][TERM_OFF[t] + d];
}
