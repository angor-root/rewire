#include "net_link.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>

static Preferences prefs;
static WiFiUDP udp;
static bool net_on = false, have_peer = false;
static IPAddress peer_ip;
static uint16_t peer_port = 0;
static char out[1400];  // un paquete UDP sin fragmentar
static size_t out_len = 0;
static uint32_t last_flush_ms = 0;

static void net_start() {
  char ssid[33] = "", pass[65] = "";
  prefs.getString("ssid", ssid, sizeof ssid);
  prefs.getString("pass", pass, sizeof pass);
  if (!ssid[0]) { Serial.println("ERR N: falta la red ('N S <red> <clave>')"); return; }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // sin ahorro de energia: menos retardo y menos paquetes perdidos
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pass);  // no bloquea: se conecta en segundo plano
  udp.begin(NET_PORT);
  net_on = true;
  Serial.printf("OK N conectando a '%s' (puerto UDP %u)\n", ssid, NET_PORT);
}

static void net_stop() {
  udp.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  net_on = have_peer = false;
}

static void flush() {
  if (out_len && have_peer && WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(peer_ip, peer_port);
    udp.write((const uint8_t*)out, out_len);
    udp.endPacket();
  }
  out_len = 0;
  last_flush_ms = millis();
}

void net_begin() {
  prefs.begin("net", false);
  if (prefs.getBool("on", false)) net_start();
}

void net_send_line(const char* line) {
  if (!net_on || !have_peer) return;
  const size_t n = strlen(line);
  if (n + 1 > sizeof out) return;
  if (out_len + n + 1 > sizeof out) flush();
  memcpy(out + out_len, line, n);
  out_len += n;
  out[out_len++] = '\n';
}

void net_tick(void (*handle_line)(char*)) {
  if (!net_on || WiFi.status() != WL_CONNECTED) return;
  int len;
  static char pkt[512];
  while ((len = udp.parsePacket()) > 0) {
    peer_ip = udp.remoteIP();  // quien manda es el destino del log
    peer_port = udp.remotePort();
    have_peer = true;
    len = udp.read(pkt, sizeof pkt - 1);
    if (len <= 0) continue;
    pkt[len] = 0;
    char* save = nullptr;
    for (char* l = strtok_r(pkt, "\r\n", &save); l; l = strtok_r(nullptr, "\r\n", &save)) handle_line(l);
  }
  if (out_len && (out_len > 1000 || millis() - last_flush_ms >= 50)) flush();
}

void net_cmd(char* a) {
  while (*a == ' ') ++a;
  if (a[0] == 'S') {
    char ssid[33], pass[65] = "";
    if (sscanf(a + 1, "%32s %64s", ssid, pass) < 1) { Serial.println("ERR N S <red> <clave>"); return; }
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.putBool("on", true);
    if (net_on) net_stop();
    net_start();
  } else if (a[0] == '1') {
    prefs.putBool("on", true);
    if (!net_on) net_start();
  } else if (a[0] == '0') {
    prefs.putBool("on", false);
    net_stop();
    Serial.println("OK N WiFi apagado");
  } else if (a[0] == '?') {
    char ssid[33] = "";
    prefs.getString("ssid", ssid, sizeof ssid);
    char msg[160];
    snprintf(msg, sizeof msg, "OK N activo=%d red='%s' conectado=%d ip=%s senal=%d dBm destino=%s:%u", net_on, ssid,
             WiFi.status() == WL_CONNECTED, WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(),
             have_peer ? peer_ip.toString().c_str() : "-", peer_port);
    Serial.println(msg);
    net_send_line(msg);
  } else {
    Serial.println("ERR N <S red clave | 1 | 0 | ?>");
  }
}
