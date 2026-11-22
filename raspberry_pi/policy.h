// Runtime en C de la política (MLP con normalizador de observaciones, ELU entre capas, salida lineal).
// Los pesos vienen de tools/export_policy_bin.py; no hay dependencias externas.
#pragma once
#include <stddef.h>

#define POLICY_MAX_LAYERS 8

typedef struct {
  int n_layers;
  int dims[POLICY_MAX_LAYERS + 1];  // dims[0] = entrada, dims[n_layers] = salida
  float* mean;
  float* std;
  float* w[POLICY_MAX_LAYERS];  // [out][in]
  float* b[POLICY_MAX_LAYERS];
  float* buf[2];                // activaciones intermedias
  void* blob;
} Policy;

// Devuelve 0 si carga bien; escribe el motivo en err en caso contrario.
int policy_load(Policy* p, const char* path, char* err, size_t err_len);
// obs[dims[0]] -> act[dims[n_layers]]
void policy_run(const Policy* p, const float* obs, float* act);
void policy_free(Policy* p);
