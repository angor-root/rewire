// Estado de un experimento de identificacion (en header para que el
// preprocesador de Arduino declare los prototipos despues del struct).
#pragma once
#include <stdint.h>

struct Experiment {
  char type[8] = "";
  int motor = 1;
  float hz = 200;
  float a[5] = {0, 0, 0, 0, 0};
  uint32_t total_ms = 0;
  uint16_t lfsr = 0x1FF;
  int prbs_bit = 0;
  int32_t last_bit_idx = -1;
};

// PI de velocidad con feedforward estatico (ver gimbal_fw.ino)
struct SpeedPI {
  float kp, ki, g_pos, dz_pos, g_neg, dz_neg;
  float integ = 0;
};
