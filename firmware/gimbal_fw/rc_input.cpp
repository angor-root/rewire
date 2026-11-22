#include "rc_input.h"

const uint8_t RC_PINS[RC_CHANNELS] = {15, 16, 17, 18};

struct RcChan {
  volatile uint32_t rise_us;
  volatile uint16_t width_us;
  volatile uint32_t last_ms;
  volatile uint32_t count, rejects;
};
static RcChan rc[RC_CHANNELS];

static void IRAM_ATTR rc_isr(void* arg) {
  RcChan* c = (RcChan*)arg;
  const int ch = c - rc;
  const uint32_t now = micros();
  if (digitalRead(RC_PINS[ch])) {
    c->rise_us = now;
  } else {
    const uint32_t w = now - c->rise_us;
    if (w >= 800 && w <= 2200) {
      c->width_us = (uint16_t)w;
      c->last_ms = millis();
      ++c->count;
    } else {
      ++c->rejects;  // ruido, flanco perdido o un pulso que no es de servo
    }
  }
}

void rc_begin() {
  for (int i = 0; i < RC_CHANNELS; ++i) {
    rc[i] = {};
    // pull-down: sin receptor conectado el pin queda en 0 y no dispara interrupciones con ruido
    pinMode(RC_PINS[i], INPUT_PULLDOWN);
    attachInterruptArg(digitalPinToInterrupt(RC_PINS[i]), rc_isr, &rc[i], CHANGE);
  }
}

bool rc_raw(int ch, uint16_t* us, uint32_t* age_ms) {
  if (ch < 0 || ch >= RC_CHANNELS) return false;
  noInterrupts();
  const uint16_t w = rc[ch].width_us;
  const uint32_t last = rc[ch].last_ms;
  const uint32_t n = rc[ch].count;
  interrupts();
  const uint32_t age = millis() - last;
  if (us) *us = w;
  if (age_ms) *age_ms = n ? age : 0xFFFFFFFF;
  return n > 0 && age < RC_TIMEOUT_MS;
}

uint32_t rc_count(int ch) { return rc[ch].count; }
uint32_t rc_rejects(int ch) { return rc[ch].rejects; }
