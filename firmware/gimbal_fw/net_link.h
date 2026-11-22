/*
 * net_link — log y comandos por WiFi (UDP), para usar dsm_gui.py sin cable (20-oct).
 *
 * La PRIZM se conecta como estacion a una red (el hotspot del celular). La red y la clave se guardan en la flash (NVS)
 * con 'N S' por USB, una sola vez. Escucha UDP en NET_PORT: cada paquete trae lineas de comando (las mismas del USB) y
 * quien lo manda queda como destino del log (la laptop se "registra" mandando cualquier comando, p. ej. 'D T L').
 * Lo que se manda por WiFi: las filas "L,..." del modo DSM y los mensajes OK D / WD D, agrupados en paquetes cada
 * ~50 ms. El control NO depende del WiFi: el lazo myRIO <-> PRIZM sigue por la UART.
 *
 * Comandos (por USB o por WiFi):
 *   N S <red> <clave>   guarda la red (sin espacios en el nombre) y la activa
 *   N 1 | N 0           activa / desactiva el WiFi (se recuerda al reiniciar)
 *   N ?                 estado: conectado, IP, senal, destino del log
 */
#pragma once

#include <stdint.h>

const uint16_t NET_PORT = 5005;

void net_begin();                               // en setup(): si estaba activado, se conecta
void net_tick(void (*handle_line)(char*));      // en loop(): comandos recibidos y envio del log
void net_send_line(const char* line);           // agrega una linea al proximo paquete (si hay destino)
void net_cmd(char* args);                       // comando 'N'
