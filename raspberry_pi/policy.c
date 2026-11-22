#include "policy.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int policy_load(Policy* p, const char* path, char* err, size_t err_len) {
  memset(p, 0, sizeof *p);
  FILE* f = fopen(path, "rb");
  if (!f) { snprintf(err, err_len, "no se pudo abrir %s", path); return -1; }
  fseek(f, 0, SEEK_END);
  const long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  uint8_t* blob = malloc(size);
  if (!blob || fread(blob, 1, size, f) != (size_t)size) {
    fclose(f);
    free(blob);
    snprintf(err, err_len, "lectura incompleta de %s", path);
    return -1;
  }
  fclose(f);

  uint32_t hdr[2];
  if (size < 12 || memcmp(blob, "DSMP", 4)) { free(blob); snprintf(err, err_len, "formato inválido"); return -1; }
  memcpy(hdr, blob + 4, 8);
  if (hdr[0] != 1 || hdr[1] == 0 || hdr[1] > POLICY_MAX_LAYERS) {
    free(blob);
    snprintf(err, err_len, "versión %u / capas %u no soportadas", hdr[0], hdr[1]);
    return -1;
  }
  p->n_layers = (int)hdr[1];
  size_t off = 12;
  int max_dim = 0;
  for (int i = 0; i <= p->n_layers; ++i) {
    uint32_t d;
    memcpy(&d, blob + off, 4);
    off += 4;
    p->dims[i] = (int)d;
    if ((int)d > max_dim) max_dim = (int)d;
  }
  size_t need = off + 2 * sizeof(float) * p->dims[0];
  for (int l = 0; l < p->n_layers; ++l) need += sizeof(float) * ((size_t)p->dims[l + 1] * p->dims[l] + p->dims[l + 1]);
  if ((size_t)size != need) {
    free(blob);
    snprintf(err, err_len, "tamaño %ld, se esperaban %zu bytes", size, need);
    return -1;
  }
  float* fp = (float*)(blob + off);  // off es múltiplo de 4
  p->mean = fp; fp += p->dims[0];
  p->std = fp; fp += p->dims[0];
  for (int l = 0; l < p->n_layers; ++l) {
    p->w[l] = fp; fp += (size_t)p->dims[l + 1] * p->dims[l];
    p->b[l] = fp; fp += p->dims[l + 1];
  }
  p->buf[0] = malloc(sizeof(float) * max_dim);
  p->buf[1] = malloc(sizeof(float) * max_dim);
  p->blob = blob;
  return 0;
}

void policy_run(const Policy* p, const float* obs, float* act) {
  float* x = p->buf[0];
  float* y = p->buf[1];
  for (int i = 0; i < p->dims[0]; ++i) x[i] = (obs[i] - p->mean[i]) / p->std[i];
  for (int l = 0; l < p->n_layers; ++l) {
    const int in = p->dims[l], out = p->dims[l + 1];
    const int last = l == p->n_layers - 1;
    float* dst = last ? act : y;
    for (int o = 0; o < out; ++o) {
      const float* w = p->w[l] + (size_t)o * in;
      float s = p->b[l][o];
      for (int i = 0; i < in; ++i) s += w[i] * x[i];
      dst[o] = last ? s : (s > 0.f ? s : expm1f(s));  // ELU(alpha=1)
    }
    float* t = x; x = y; y = t;
  }
}

void policy_free(Policy* p) {
  free(p->buf[0]);
  free(p->buf[1]);
  free(p->blob);
  memset(p, 0, sizeof *p);
}
