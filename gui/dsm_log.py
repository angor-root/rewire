#!/usr/bin/env python3
"""Log del modo DSM (myRIO -> PRIZM) para tunear Kanayama y el PI: graba y grafica.

    python3 dsm_log.py grabar                          # graba hasta Ctrl+C -> logs/dsm_<fecha>.csv
    python3 dsm_log.py grabar --nombre circulo_a1      # logs/dsm_<fecha>_circulo_a1.csv
    python3 dsm_log.py grabar --extras xr,yr,thr,vr,wr,ex,ey,eth,x,y,th   # nombres de los campos extra del VI
    python3 dsm_log.py graficar logs/dsm_....csv       # -> mismo nombre .png (y lo abre con --ver)
    python3 dsm_log.py graficar                        # el ultimo log

Pide al firmware 'D T L': una fila "L,..." por cada linea del myRIO (~50 Hz, la del lazo de control).
Si el VI agrega numeros al final de su linea ("G gx gy gz wR wL ex ey eth ..."), salen como columnas extra.
Al salir vuelve a 'D T 10'. Requiere gimbal_fw del 19-oct.
"""
import argparse
import csv
import glob
import math
import os
import sys
import time

import serial

LOGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
COLS = ["ms", "n", "run", "gx", "gy", "gz", "wref_rx_R", "wref_rx_L", "wref_R", "wref_L", "w_R", "w_L",
        "u_R", "u_L", "cnt_R", "cnt_L", "dN_R", "dN_L", "x", "y", "th", "nx"]
N_EXTRA = 12
# filas 'F' del filtro encoders + IMU (gimbal_fw 28-oct, odo_ekf.h): van a <log>_fusion.csv
COLS_F = ["ms", "v_enc", "w_enc", "w_gyro", "vx", "vy", "ba", "rej_v", "rej_w", "sin_contacto", "nis_win", "perdidas",
          "dN_crudo_R", "dN_crudo_L"]


def default_port():
    ports = sorted(glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else "/dev/ttyACM0"


def grabar(a):
    os.makedirs(LOGS, exist_ok=True)
    extras = [e.strip() for e in a.extras.split(",") if e.strip()] if a.extras else []
    extras += [f"x{i + 1}" for i in range(len(extras), N_EXTRA)]
    name = time.strftime("dsm_%Y%m%d_%H%M%S") + (f"_{a.nombre}" if a.nombre else "") + ".csv"
    path = os.path.join(LOGS, name)
    s = serial.Serial(a.port, 115200, timeout=0.1)
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write(b"D T L\n")
    f = open(path, "w", newline="")
    wr = csv.writer(f)
    wr.writerow(["t_pc"] + COLS + extras)
    ff = open(path[:-4] + "_fusion.csv", "w", newline="")
    wf = csv.writer(ff)
    wf.writerow(["t_pc"] + COLS_F)
    print(f"grabando en {path}  (Ctrl+C para terminar)")
    rows = n_win = 0
    t_win = time.time()
    last = None
    try:
        while True:
            line = s.readline().decode(errors="replace").strip()
            if line.startswith("L,"):
                v = line[2:].split(",")
                if len(v) < len(COLS):
                    continue
                wr.writerow([f"{time.time():.3f}"] + v)
                rows += 1
                n_win += 1
                last = v
            elif line.startswith("F,"):
                wf.writerow([f"{time.time():.3f}"] + line[2:].split(","))
            elif line and line[:2] not in ("T,", "E,", "d,"):
                print(line, flush=True)  # OK / WD / ERR del firmware
            if time.time() - t_win >= 1.0:
                hz = n_win / (time.time() - t_win)
                if last:
                    print(f"{rows:6d} filas  {hz:5.1f} Hz  run={last[2]}  wref=({last[8]},{last[9]})  "
                          f"w=({last[10]},{last[11]})  u=({last[12]},{last[13]})  extra={last[21]}", flush=True)
                else:
                    print("sin filas: ¿myRIO mandando? ¿firmware del 19-oct?", flush=True)
                n_win, t_win = 0, time.time()
                f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        s.write(b"D T 10\n")
        f.close()
        ff.close()
        s.close()
    print(f"\n{rows} filas -> {path}")
    if rows:
        graficar(argparse.Namespace(csv=path, ver=False))


def graficar(a):
    import matplotlib
    if not a.ver:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = a.csv or max(glob.glob(os.path.join(LOGS, "dsm_*.csv")), key=os.path.getmtime)
    with open(path) as f:
        rd = csv.reader(f)
        head = next(rd)
        data = [r for r in rd if len(r) >= len(COLS) + 1]
    if not data:
        sys.exit(f"{path}: vacio")
    col = {h: i for i, h in enumerate(head[:len(COLS) + 1])}  # los extras pueden repetir nombres (x, y, th)
    num = lambda h: [float(r[j]) if j < len(r) and r[j] != "" else math.nan
                     for j in [h if isinstance(h, int) else col[h]] for r in data]
    ms = num("ms")
    off = 0.0  # si la PRIZM se reinicio a mitad de la grabacion su reloj vuelve a 0: seguir contando
    for i in range(1, len(ms)):
        if ms[i] + off < ms[i - 1]:
            off = ms[i - 1] - ms[i] + 20
        ms[i] += off
    t = [(m - ms[0]) / 1000 for m in ms]
    nx = int(max(num("nx")))
    extras = head[len(COLS) + 1:len(COLS) + 1 + nx]

    fig, ax = plt.subplots(3, 2, figsize=(14, 10))
    for k, (side, lbl) in enumerate((("R", "derecha"), ("L", "izquierda"))):
        p = ax[0, k]
        p.plot(t, num(f"wref_rx_{side}"), ":", color="gray", label="ref recibida")
        p.plot(t, num(f"wref_{side}"), label="ref aplicada")
        p.plot(t, num(f"w_{side}"), label="medida")
        p.set_title(f"rueda {lbl} (rad/s)")
        p.legend(loc="upper right", fontsize=8)
        p.grid(alpha=0.3)
    p = ax[1, 0]
    p.plot(t, num("u_R"), label="u der")
    p.plot(t, num("u_L"), label="u izq")
    p.axhline(100, color="r", lw=0.5)
    p.axhline(-100, color="r", lw=0.5)
    p.set_title("potencia del PI (%)")
    p.legend(fontsize=8)
    p.grid(alpha=0.3)
    p = ax[1, 1]
    dt = [b - c for b, c in zip(ms[1:], ms[:-1])]
    p.plot(t[1:], dt, ".", ms=2)
    p.set_title(f"periodo entre lineas del myRIO (ms), mediana {sorted(dt)[len(dt) // 2] if dt else 0:.0f}")
    p.grid(alpha=0.3)
    p = ax[2, 0]
    names = {h.lower(): len(COLS) + 1 + j for j, h in enumerate(extras)}
    if all(k in names for k in ("x", "y")):
        p.plot(num(names["x"]), num(names["y"]), label="odometría myRIO")
    if all(k in names for k in ("xr", "yr")):
        p.plot(num(names["xr"]), num(names["yr"]), "--", label="referencia")
    p.set_aspect("equal", "datalim")
    p.set_title("plano XY (m)")
    p.legend(fontsize=8)
    p.grid(alpha=0.3)
    p = ax[2, 1]
    err = [h for h in extras if h.lower() in ("ex", "ey", "eth", "etheta")]
    for h in err or extras:
        p.plot(t, num(names[h.lower()]), label=h)
    p.set_title("errores de seguimiento (del VI)" if err else
                ("campos extra del VI" if extras else "sin campos extra (el VI puede mandar ex ey eth ...)"))
    if extras:
        p.legend(fontsize=8)
    p.grid(alpha=0.3)
    ax[2, 1].set_xlabel("t (s)")
    fig.suptitle(os.path.basename(path))
    fig.tight_layout()
    png = os.path.splitext(path)[0] + ".png"
    fig.savefig(png, dpi=110)
    print(f"gráfico -> {png}")
    if a.ver:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=default_port())
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("grabar")
    g.add_argument("--nombre", default="")
    g.add_argument("--extras", default="xr,yr,thr,vr,wr,ex,ey,eth,x,y,th",
                   help="nombres de los numeros extra que agrega el VI, en orden")
    p = sub.add_parser("graficar")
    p.add_argument("csv", nargs="?")
    p.add_argument("--ver", action="store_true", help="abrir la ventana además de guardar el .png")
    a = ap.parse_args()
    grabar(a) if a.cmd == "grabar" else graficar(a)


if __name__ == "__main__":
    main()
