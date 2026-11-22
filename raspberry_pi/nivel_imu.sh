#!/bin/bash
# Nivel de la IMU = como esta AHORA (09-nov): con el robot quieto en piso plano. Promedia ~1 s de filas i de ball_pixel,
# lo guarda en ~/imu_ref.txt (arranque.sh lo usa en cada reinicio) y lo aplica ya ("IMU REF ux uy").
#   ~/nivel_imu.sh          (tambien el boton "Nivel IMU" de dsm_gui)
sleep 1.2
r=$(grep -a "^i," /tmp/ball.log | tail -n 30 | awk -F, '{n++; x+=$3; y+=$4; xx+=$3*$3; yy+=$4*$4}
  END {if (n < 10) exit 1; sx=sqrt(xx/n-(x/n)^2); sy=sqrt(yy/n-(y/n)^2);
       printf "%.4f %.4f %.4f\n", x/n, y/n, (sx > sy ? sx : sy)}') || { echo "ERROR: no llegan datos de la IMU"; exit 1; }
set -- $r
awk "BEGIN{exit !($3 > 0.01)}" && { echo "ERROR: el robot se esta moviendo (desv. $3): dejalo quieto y repite"; exit 1; }
cp ~/imu_ref.txt ~/imu_ref.txt.ant 2>/dev/null
echo "$1 $2" > ~/imu_ref.txt
timeout 2 bash -c "echo 'IMU REF $1 $2' > /tmp/ball_cmd"; sleep 0.5
grep -a "^OK IMU" /tmp/ball.log | tail -1 | grep -o "ref=.*" | sed "s/^/nivel nuevo: /;s/\$/ (anterior $(cat ~/imu_ref.txt.ant 2>/dev/null))/"
