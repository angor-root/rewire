#!/bin/bash
# Supervisor del control de la pelota (31-oct): lo corre el servicio de usuario dsm-pelota.service al encender la Pi,
# sin tocar nada. Espera la camara y la PRIZM, arranca imu_pi y ball_pixel (politica al mando + compensacion con la
# IMU) y los vigila cada 2 s:
#   - PRIZM reiniciada (bateria baja: vuelve como otro /dev/ttyACM*) o ausente al arrancar -> reinicia ball_pixel
#   - camara colgada (< 5 Hz durante ~10 s) -> usbreset de la camara y reinicia ball_pixel
#   - imu_pi o ball_pixel muertos -> los vuelve a arrancar
# Configuracion en ~/pelota.conf (VMIN, POL, IMU); log en /tmp/arranque.log.
# Parar todo: systemctl --user stop dsm-pelota     Volver a arrancar: systemctl --user start dsm-pelota
# Ojo: tras cada reinicio de la PRIZM hay que presionar su BOTON VERDE (eso no se puede automatizar).
VMIN=90; POL=1; IMU=1
[ -f "$HOME/pelota.conf" ] && . "$HOME/pelota.conf"
log() { echo "$(date +%T) $*" >> /tmp/arranque.log; }
fifo() { [ -p /tmp/ball_cmd ] && timeout 2 bash -c "echo '$1' > /tmp/ball_cmd" 2>/dev/null; }  # nunca crearla: es de ball_pixel
bp_pid() { pgrep -f "^$HOME/ball_pixel" | head -1; }
aplicar() {  # manda un comando por la FIFO y confirma en /tmp/ball.log que se aplico (hasta 5 intentos)
  for i in 1 2 3 4 5; do
    fifo "$1"; sleep 0.5
    tail -n 200 /tmp/ball.log 2>/dev/null | grep -q "$2" && return 0
  done
  log "AVISO: '$1' no se aplico"
}

arrancar_bp() {
  [ -f "$HOME/pelota.conf" ] && . "$HOME/pelota.conf"  # 02-nov: releer, si no un reinicio de la PRIZM vuelve a lo del arranque
  pkill -f "^$HOME/ball_pixel"; sleep 1
  local pol=(); [ -f "$HOME/politica.bin" ] && pol=(--policy "$HOME/politica.bin" $(cat "$HOME/balltrack_ejes.txt" 2>/dev/null))
  setsid "$HOME/ball_pixel" --vmin "$VMIN" "${pol[@]}" $(cat "$HOME/imu_ejes.txt" 2>/dev/null) \
    > /tmp/ball_pixel.out 2>&1 < /dev/null &
  # al encender la Pi la camara tarda en abrir: esperar a que ball_pixel escuche su FIFO (imprime "ball_pixel:" despues)
  for i in $(seq 30); do grep -q "^ball_pixel:" /tmp/ball_pixel.out 2>/dev/null && break; sleep 1; done
  sleep 1
  aplicar "PD 1" "OK PD on=1"
  # 09-nov: ganancias guardadas (p. ej. el LQR con retardo: "PD K ..." y "PD L ..."), una por linea
  [ -f "$HOME/pd_ganancias.txt" ] && while read -r l; do [ -n "$l" ] && aplicar "$l" "OK PD"; done < "$HOME/pd_ganancias.txt"
  [ "$POL" = 1 ] && grep -q "politica .* cargada" /tmp/ball_pixel.out && aplicar "POL 1" "OK POL on=1"
  # referencia de nivel = como esta el robot al arrancar (se supone en el piso); IMU REF por la FIFO la vuelve a tomar
  # 09-nov: con ~/imu_ref.txt ("ux uy" medidos en piso plano) la referencia es FIJA: un reinicio de la PRIZM en una
  # rampa ya no la toma inclinada (paso a las 12:11: compensaba 13.6 grados al reves)
  [ "$IMU" = 1 ] && { fifo "IMU REF $(cat "$HOME/imu_ref.txt" 2>/dev/null)"; aplicar "IMU ON" "OK IMU on=1"; }
  lento=0
  log "ball_pixel arrancado (pid $(bp_pid), PRIZM ${1:-ninguna}, POL=$POL IMU=$IMU vmin=$VMIN)"
}

log "supervisor arrancado"
systemctl --user stop cam-vision prizm-bridge 2>/dev/null  # el sistema viejo de ArUco se queda con la camara
until ls /dev/v4l/by-id/*-video-index0 >/dev/null 2>&1; do sleep 2; done
for i in $(seq 15); do ls /dev/ttyACM* >/dev/null 2>&1 && break; sleep 2; done  # hasta 30 s esperando la PRIZM
lento=0
while true; do
  if ! pgrep -x imu_pi >/dev/null; then  # la IMU solo hacia ball_pixel (UDP local); mide el sesgo del giro 1 s: QUIETO
    setsid "$HOME/imu_pi" --puerto /dev/null --udp 127.0.0.1:5007 --r_imu -0.115 0 0.045 --r_plato -0.115 0 0.155 \
      > /tmp/imu_pi.out 2>&1 < /dev/null &
    log "imu_pi arrancado"
  fi
  prizm=$(ls /dev/ttyACM* 2>/dev/null | head -1)
  pid=$(bp_pid)
  if [ -z "$pid" ]; then
    arrancar_bp "$prizm"
  elif [ -n "$prizm" ] && ! ls -l /proc/$pid/fd 2>/dev/null | grep -q "$prizm\$"; then
    log "la PRIZM esta en $prizm y ball_pixel no la tiene abierta (reinicio de la PRIZM?)"
    arrancar_bp "$prizm"
  else
    hz=$(tail -1 /tmp/ball_pixel.out 2>/dev/null | grep -o '^[0-9.]* Hz' | cut -d' ' -f1)
    # 09-nov: camara colgada sin imagenes -> ball_pixel no imprime nada y la ultima linea seguia diciendo "28 Hz"
    # (13:38:54: 80 s sin control y el supervisor no lo vio). Sin lineas nuevas en 4 s (imprime cada 2) = 0 Hz
    [ $(( $(date +%s) - $(stat -c %Y /tmp/ball_pixel.out 2>/dev/null || echo 0) )) -gt 4 ] && hz=0
    if [ -n "$hz" ] && awk "BEGIN{exit !($hz < 5)}"; then lento=$((lento + 1)); else lento=0; fi
    if [ "$lento" -ge 2 ]; then  # 09-nov: 2 vueltas (~4 s); antes 5 (~10 s)
      log "camara colgada ($hz Hz): usbreset"
      pkill -f "^$HOME/ball_pixel"; sleep 1
      sudo -n /usr/bin/usbreset "$(lsusb | grep -i webcam | awk '{print $6}')" >> /tmp/arranque.log 2>&1
      sleep 3
      arrancar_bp "$prizm"
    fi
  fi
  sleep 2
done
