"""Graba y grafica una sesion del robot: pelota (camara), PD, plato (lo que va a la IK tras la compensacion), IMU del
chasis, video de la camara del plato y, con --dsm, las filas del myRIO (modo DSM de la PRIZM). 28-oct.

    python3 deploy/tools/sesion.py grabar NOMBRE [--seg 30] [--dsm] [--sin-video] [--ip 10.109.62.171]
    python3 deploy/tools/sesion.py graficar NOMBRE

grabar: en la Pi activa el video de ball_pixel (REC 1), el log de imu_pi (USR1) y, con --dsm, 'D T L' en la PRIZM;
espera --seg segundos o Ctrl+C; apaga todo y baja a logs/NOMBRE/: ball_log.txt (filas b, c, p, r de /tmp/ball.log
entre el inicio y el fin), imu_log.csv, video.avi (+ video.mp4) y meta.json. Luego grafica.
Todo lleva la hora de la Pi (steady_clock / CLOCK_MONOTONIC), asi que las curvas y el video estan sincronizados.
Necesita ball_pixel (~/pelota.sh iniciar) e imu_pi corriendo en la Pi.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'logs')


def ssh(ip, cmd, timeout=30):
    r = subprocess.run(['ssh', '-o', f'HostName={ip}', '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes', '-l', 'robot', 'dsm-pc2', cmd],
                       capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        sys.exit(f'ERROR ssh: {r.stderr.strip()}')
    return r.stdout


FIFO = "timeout 2 bash -c 'echo \"{}\" > /tmp/ball_cmd'"
NOW = "python3 -c 'import time; print(time.monotonic())'"


def ssh_ok(ip, cmd, timeout=30):
    """Como ssh() pero devuelve None en vez de salir (para la interfaz)."""
    try:
        r = subprocess.run(['ssh', '-o', f'HostName={ip}', '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes', '-l', 'robot', 'dsm-pc2', cmd],
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None
    return r.stdout if r.returncode == 0 else None


def pi_start(ip, video=True, dsm=False):
    """Empieza a grabar en la Pi (video de ball_pixel, log de imu_pi, filas DSM). Devuelve (t0 de la Pi, avisos)."""
    chk = ssh_ok(ip, 'pgrep -x ball_pixel >/dev/null && echo bp; pgrep -x imu_pi >/dev/null && echo imu')
    if chk is None:
        raise RuntimeError(f'la Pi ({ip}) no responde por ssh')
    avisos = []
    if 'bp' not in chk:
        avisos.append('ball_pixel no corre en la Pi: sin pelota, sin video y sin las lineas de la PRIZM (~/pelota.sh iniciar)')
    if 'imu' not in chk:
        avisos.append('imu_pi no corre en la Pi: sin datos de la IMU')
    start = [FIFO.format('REC 1')] if video and 'bp' in chk else []
    start += [FIFO.format('D T L')] if dsm and 'bp' in chk else []
    start += ['pkill -USR1 -x imu_pi', NOW]
    return float(ssh(ip, '; '.join(start)).split()[-1]), avisos


def pi_stop(ip, out, t0, video=True, dsm=False):
    """Termina la grabacion de la Pi y baja todo a out/: ball_log.txt (b, c, p, r entre t0 y t1: p = TODAS las lineas
    de la PRIZM, con su reloj), imu_log.csv, video.avi/.mp4, meta.json."""
    os.makedirs(out, exist_ok=True)
    stop = [FIFO.format('REC 0')] if video else []
    stop += [FIFO.format('D T 10')] if dsm else []
    stop += ['pkill -USR2 -x imu_pi', NOW]
    t1 = float(ssh(ip, '; '.join(stop)).split()[-1])
    # 09-nov: TODAS las filas con hora en la ventana (b, c, p, r + i IMU del control, s politica en sombra, tr objetivo)
    # y las confirmaciones OK (configuracion: PD/LQR, IMU, POL) de todo el log
    awk = f"awk -F, '($2+0>={t0} && $2+0<={t1}) || /^OK/' /tmp/ball.log"
    open(os.path.join(out, 'ball_log.txt'), 'w').write(ssh(ip, awk, timeout=120))
    files = ['/tmp/imu_log.csv'] + (['/tmp/ball_rec.avi'] if video else [])
    for f in files:
        dst = os.path.join(out, 'video.avi' if f.endswith('.avi') else os.path.basename(f))
        subprocess.run(['scp', '-q', '-o', f'HostName={ip}', f'dsm-pc2:{f}', dst], check=False)
    if os.path.exists(os.path.join(out, 'video.avi')):
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', os.path.join(out, 'video.avi'), '-c:v', 'libx264',
                        '-pix_fmt', 'yuv420p', os.path.join(out, 'video.mp4')], check=False)
    json.dump({'t0': t0, 't1': t1, 'ip': ip, 'dsm': dsm, 'video': video,
               'fecha': time.strftime('%Y-%m-%d %H:%M:%S')}, open(os.path.join(out, 'meta.json'), 'w'), indent=1)
    return t1


def grabar(a):
    out = os.path.join(ROOT, a.nombre)
    try:
        t0, avisos = pi_start(a.ip, not a.sin_video, a.dsm)
    except RuntimeError as ex:
        sys.exit(f'ERROR {ex}')
    for w in avisos:
        print('AVISO', w)
    print(f'grabando {a.nombre} (Pi t0 = {t0:.3f}) ... Ctrl+C para terminar' + (f' o {a.seg:.0f} s' if a.seg else ''))
    try:
        t_end = time.time() + (a.seg or 1e9)
        while time.time() < t_end:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    print('fin; bajando ...')
    t1 = pi_stop(a.ip, out, t0, not a.sin_video, a.dsm)
    print(f'{t1 - t0:.1f} s')
    graficar(a)


OKHT = re.compile(r'OK H T roll=(\S+) pitch=(\S+) -> (\S+) (\S+) servos (\d+) (\d+)')


def leer(out):
    m = json.load(open(os.path.join(out, 'meta.json')))
    t0 = m['t0']
    b, c, p, L, F = [], [], [], [], []
    for line in open(os.path.join(out, 'ball_log.txt')):
        f = line.rstrip('\n').split(',')
        try:
            if f[0] == 'b':
                b.append([float(x) for x in f[1:9]])
            elif f[0] == 'c':
                c.append([float(x) for x in f[1:8]])
            elif f[0] == 'p':
                k = OKHT.match(f[2])
                if k:
                    p.append([float(f[1])] + [float(x) for x in k.groups()])
                elif f[2] == 'L':  # fila del modo DSM (gimbal_fw 'D T L'): ver el encabezado del firmware
                    L.append([float(f[1])] + [float(x) for x in f[3:23]])
                elif f[2] == 'F':  # filtro encoders + IMU (28-oct): ms v_enc w_enc w_gyro vx vy ba rej_v rej_w
                    F.append([float(f[1])] + [float(x) for x in f[3:17]])  # sin_contacto nis perdidas dN_R dN_L
        except (ValueError, IndexError):
            pass
    arr = lambda x, n: np.array(x).reshape(-1, n)
    d = {'b': arr(b, 8), 'c': arr(c, 7), 'p': arr(p, 7), 'L': arr(L, 21), 'F': arr(F, 15)}
    for k in d:
        if len(d[k]):
            d[k][:, 0] -= t0
    imu = os.path.join(out, 'imu_log.csv')
    d['imu'] = np.loadtxt(imu, delimiter=',', skiprows=1, ndmin=2) if os.path.exists(imu) and os.path.getsize(imu) > 80 \
        else np.zeros((0, 13))
    if len(d['imu']):
        d['imu'][:, 0] -= t0
    return m, d


def graficar(a):
    out = os.path.join(ROOT, a.nombre)
    m, d = leer(out)
    b, c, p, imu, L, F = d['b'], d['c'], d['p'], d['imu'], d['L'], d['F']
    vis = b[:, 1] > 0.5 if len(b) else np.zeros(0, bool)
    res = [f'sesion {a.nombre}: {m["t1"] - m["t0"]:.1f} s']
    if len(b):
        r = np.hypot(b[vis, 4], b[vis, 5]) * 100
        res.append(f'camara {len(b) / (m["t1"] - m["t0"]):.1f} Hz, pelota vista {100 * vis.mean():.0f} %')
        if len(r):
            res.append(f'distancia al centro: media {r.mean():.2f} cm, RMS {np.sqrt((r ** 2).mean()):.2f} cm, '
                       f'max {r.max():.2f} cm, p95 {np.percentile(r, 95):.2f} cm')
    if len(c):
        sat = (np.abs(c[:, 5]) > 7.9) | (np.abs(c[:, 6]) > 7.9)
        res.append(f'PD: |roll| max {np.abs(c[:, 5]).max():.1f}, |pitch| max {np.abs(c[:, 6]).max():.1f} deg, '
                   f'saturado {100 * sat.mean():.0f} % (lim 8)')
    if len(p):
        comp = p[:, 3:5] - p[:, 1:3]
        res.append(f'compensacion IMU: |roll| max {np.abs(comp[:, 0]).max():.1f}, |pitch| max {np.abs(comp[:, 1]).max():.1f} deg')
    if len(imu):
        res.append(f'chasis: roll {imu[:, 1].min():+.1f}..{imu[:, 1].max():+.1f}, pitch {imu[:, 2].min():+.1f}..'
                   f'{imu[:, 2].max():+.1f} deg, rumbo final {imu[-1, 3]:+.1f} deg, |w| max {np.abs(imu[:, 4:7]).max():.2f} rad/s')
    print('\n'.join(res))
    open(os.path.join(out, 'resumen.txt'), 'w').write('\n'.join(res) + '\n')

    if len(F):
        res.append(f'filtro: sin contacto {100 * F[:, 10].mean():.1f} % del tiempo, perdidas de contacto '
                   f'{int(F[-1, 12] - F[0, 12])}, |v_enc - v_fusion| max {np.abs(F[:, 2] - F[:, 5]).max():.3f} m/s')
        open(os.path.join(out, 'resumen.txt'), 'w').write('\n'.join(res) + '\n')
    n = 4 + (len(L) > 0) + (len(F) > 0)
    fig, ax = plt.subplots(n, 1, figsize=(12, 2.6 * n), sharex=True)
    if len(b):
        bx = np.where(vis, b[:, 4] * 100, np.nan)
        by = np.where(vis, b[:, 5] * 100, np.nan)
        ax[0].plot(b[:, 0], bx, label='x pelota')
        ax[0].plot(b[:, 0], by, label='y pelota')
    ax[0].set_ylabel('cm'); ax[0].set_title('pelota (camara)')
    if len(c):
        ax[1].plot(c[:, 0], c[:, 5], label='roll PD')
        ax[1].plot(c[:, 0], c[:, 6], label='pitch PD')
    if len(p):
        ax[1].plot(p[:, 0], p[:, 3], '--', label='roll a la IK')
        ax[1].plot(p[:, 0], p[:, 4], '--', label='pitch a la IK')
    ax[1].set_ylabel('grados'); ax[1].set_title('plato: lo que pide el PD y lo que va a la IK (PD + compensacion)')
    if len(imu):
        ax[2].plot(imu[:, 0], imu[:, 1], label='roll chasis')
        ax[2].plot(imu[:, 0], imu[:, 2], label='pitch chasis')
        ax[3].plot(imu[:, 0], imu[:, 4:7], label=['wx', 'wy', 'wz'])
        ax3b = ax[3].twinx()
        ax3b.plot(imu[:, 0], imu[:, 7:10], ':', label=['ax', 'ay', 'az'])
        ax3b.set_ylabel('m/s2 (puntos)'); ax3b.legend(loc='lower right', fontsize=7)
    ax[2].set_ylabel('grados'); ax[2].set_title('IMU: inclinacion del chasis (+pitch nariz arriba, +roll izquierda arriba)')
    ax[3].set_ylabel('rad/s'); ax[3].set_title('IMU: giro y aceleracion sin gravedad (en el pivote del plato)')
    if len(L):  # t, ms, n, run, gx, gy, gz, wrefrx R L, wref R L, w R L, u R L, cuentas R L, dN R L, x
        ax[4].plot(L[:, 0], L[:, 9], label='w_ref der'); ax[4].plot(L[:, 0], L[:, 11], label='w der')
        ax[4].plot(L[:, 0], L[:, 10], label='w_ref izq'); ax[4].plot(L[:, 0], L[:, 12], label='w izq')
        ax[4].set_ylabel('rad/s'); ax[4].set_title('ruedas (modo DSM / myRIO)')
    if len(F):  # t, ms, v_enc, w_enc, w_gyro, vx, vy, ba, rej_v, rej_w, sin_contacto, ...
        k = 4 + (len(L) > 0)
        ax[k].plot(F[:, 0], F[:, 2], label='v encoders'); ax[k].plot(F[:, 0], F[:, 5], label='v fusionada')
        ax[k].plot(F[:, 0], F[:, 3], ':', label='w encoders'); ax[k].plot(F[:, 0], F[:, 4], ':', label='w giroscopio')
        ax[k].fill_between(F[:, 0], 0, 1, where=F[:, 10] > 0.5, color='r', alpha=0.15, lw=0,
                           transform=ax[k].get_xaxis_transform(), label='sin contacto')
        ax[k].set_ylabel('m/s, rad/s'); ax[k].set_title('fusion encoders + IMU (rojo: sin contacto)')
    for x in ax:
        x.grid(alpha=0.3)
        if x.get_lines():
            x.legend(loc='upper right', fontsize=7, ncol=4)
    ax[-1].set_xlabel('t (s)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'series.png'), dpi=110)

    fig2, ax2 = plt.subplots(1, 2, figsize=(11, 5))
    if vis.any():
        sc = ax2[0].scatter(b[vis, 4] * 100, b[vis, 5] * 100, c=b[vis, 0], s=4, cmap='viridis')
        fig2.colorbar(sc, ax=ax2[0], label='t (s)')
        ax2[1].hist(np.hypot(b[vis, 4], b[vis, 5]) * 100, bins=40)
    ax2[0].add_patch(plt.Rectangle((-10, -10), 20, 20, fill=False, ls='--'))
    ax2[0].set_aspect('equal'); ax2[0].set_xlim(-11, 11); ax2[0].set_ylim(-11, 11)
    ax2[0].set_xlabel('x (cm)'); ax2[0].set_ylabel('y (cm)'); ax2[0].set_title('pelota sobre el plato (200 x 200 mm)')
    ax2[1].set_xlabel('distancia al centro (cm)'); ax2[1].set_title('histograma')
    fig2.tight_layout()
    fig2.savefig(os.path.join(out, 'plato.png'), dpi=110)
    print(f'graficos en {out}/series.png y plato.png' + (' ; video.mp4' if os.path.exists(os.path.join(out, 'video.mp4')) else ''))
    if not a.no_mostrar:
        plt.show()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('accion', choices=['grabar', 'graficar'])
    ap.add_argument('nombre')
    ap.add_argument('--seg', type=float, default=0, help='duracion (0 = hasta Ctrl+C)')
    ap.add_argument('--dsm', action='store_true', help="tambien las filas del myRIO ('D T L' en la PRIZM)")
    ap.add_argument('--sin-video', action='store_true')
    ap.add_argument('--ip', default='10.109.62.171')
    ap.add_argument('--no-mostrar', action='store_true', help='solo guardar los PNG')
    a = ap.parse_args()
    grabar(a) if a.accion == 'grabar' else graficar(a)
