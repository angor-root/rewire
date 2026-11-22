// Política en el ESP32-S3 de la PRIZM: MLP (normalizador + ELU, salida lineal) y la observación con
// historial. Misma matemática que deploy/c/policy.c y deploy/dsm_deploy/obs.py.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCU_POLICY_MAX_LAYERS 8
#define MCU_POLICY_MAX_DIM 256

typedef struct {
  int n_layers;
  int dims[MCU_POLICY_MAX_LAYERS + 1];  // dims[0] = entrada, dims[n_layers] = salida
  const float* mean;
  const float* std;
  const float* w[MCU_POLICY_MAX_LAYERS];  // [out][in]
  const float* b[MCU_POLICY_MAX_LAYERS];
} McuPolicy;

// blob = contenido de policy.bin (alineado a 4 bytes). Devuelve 0 si el formato es válido.
int mcu_policy_init(McuPolicy* p, const void* blob, size_t bytes);
// obs[dims[0]] -> act[dims[n_layers]]. No es reentrante (buffers estáticos): un solo llamador a la vez.
void mcu_policy_run(const McuPolicy* p, const float* obs, float* act);

// Observación: 5 pasos de historial POR TÉRMINO, como el CircularBuffer de IsaacLab.
// Un paso = [cámara 5, gravedad 3, ruedas 2 (rad/s, se escalan x0.1 aquí), comando 3, acción 4] = 17.
#define OBS_STEP 17
#define OBS_HISTORY 5
#define OBS_DIM (OBS_STEP * OBS_HISTORY)

typedef struct {
  float hist[OBS_HISTORY][OBS_STEP];  // [0] = más viejo
  int empty;
} ObsHist;

void obs_reset(ObsHist* h);
// step: valores sin escalar. Escribe la obs aplanada (85) en out.
void obs_push(ObsHist* h, const float step[OBS_STEP], float out[OBS_DIM]);

#ifdef __cplusplus
}
#endif
