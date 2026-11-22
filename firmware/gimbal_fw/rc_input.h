/*
 * rc_input — lectura del receptor R/C TETRIX (R2004GF / 42084, 2.4 GHz, 4 canales) en la PRIZM Pro.
 *
 * El receptor saca, por cada canal, un pulso de servo estandar (~1000-2000 us, centro ~1500 us, cada ~14-20 ms).
 * Cada canal entra a un pin digital de un puerto de sensor y se mide con interrupciones (flanco de subida y bajada).
 *
 *   canal 1 -> puerto de sensor 2, pin 1 = GPIO 15
 *   canal 2 -> puerto de sensor 2, pin 2 = GPIO 16
 *   canal 3 -> puerto de sensor 3, pin 1 = GPIO 17
 *   canal 4 -> puerto de sensor 3, pin 2 = GPIO 18
 *   (puerto 1 queda para la UART del myRIO: GPIO 6)
 *
 * Un canal es valido si su ultimo pulso llego hace < RC_TIMEOUT_MS y midio 800-2200 us.
 */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <Arduino.h>

#define RC_CHANNELS 4
#define RC_TIMEOUT_MS 100

extern const uint8_t RC_PINS[RC_CHANNELS];

void rc_begin();
/* ancho del ultimo pulso (us) y su edad (ms); devuelve false si el canal no es valido */
bool rc_raw(int ch, uint16_t* us, uint32_t* age_ms);
/* pulsos aceptados / rechazados por canal desde el arranque (diagnostico de cableado) */
uint32_t rc_count(int ch);
uint32_t rc_rejects(int ch);

#endif
