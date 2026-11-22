"""Inclinacion de la base en vivo (09-nov), desde las filas 'i,<t>,<ux>,<uy>,<uz>,...' que ball_pixel escribe en
/tmp/ball.log (gravedad unitaria de imu_pi). El cero es la referencia de nivel de ball_pixel (IMU REF, tomada al arrancar
en el piso); el boton "Cero aqui" lo cambia por la posicion actual.

  python3 inclinacion_base.py [--pi 10.206.7.171] [--ventana 30]
"""
import argparse
import math
import re
import subprocess
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


def angles(u, ref):
    """(eje x, eje y, total) en grados respecto de ref."""
    ax = math.degrees(math.asin(max(-1, min(1, u[0]))) - math.asin(ref[0]))
    ay = math.degrees(math.asin(max(-1, min(1, u[1]))) - math.asin(ref[1]))
    rz = math.sqrt(max(0.0, 1 - ref[0] ** 2 - ref[1] ** 2))
    n = math.sqrt(sum(c * c for c in u)) or 1
    tot = math.degrees(math.acos(max(-1, min(1, (u[0] * ref[0] + u[1] * ref[1] + u[2] * rz) / n))))
    return ax, ay, tot


assert abs(angles((0, 0, 1), (0, 0))[2]) < 1e-9
assert abs(angles((math.sin(math.radians(10)), 0, math.cos(math.radians(10))), (0, 0))[0] - 10) < 1e-6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi", default="10.206.7.171")
    ap.add_argument("--ventana", type=float, default=30, help="segundos en pantalla")
    a = ap.parse_args()
    ref = [0.0, 0.0]
    out = subprocess.run(["ssh", "-o", f"HostName={a.pi}", "dsm-pc2", "grep -a 'OK IMU' /tmp/ball.log | tail -1"],
                         capture_output=True, text=True, timeout=15).stdout
    m = re.search(r"ref=(-?[\d.]+) (-?[\d.]+)", out)
    if m:
        ref = [float(m.group(1)), float(m.group(2))]
    print(f"referencia de nivel de ball_pixel: {ref}")
    data = deque()  # (t, ux, uy, uz)
    last = [None]

    def reader():
        p = subprocess.Popen(["ssh", "-o", f"HostName={a.pi}", "-o", "ServerAliveInterval=5", "dsm-pc2",
                              "tail -n 0 -F /tmp/ball.log 2>/dev/null | grep -a --line-buffered '^i,'"],
                             stdout=subprocess.PIPE, text=True)
        for line in p.stdout:
            f = line.split(",")
            try:
                r = (float(f[1]), float(f[2]), float(f[3]), float(f[4]))
            except (IndexError, ValueError):
                continue
            data.append(r)
            last[0] = r
            while data and data[-1][0] - data[0][0] > a.ventana:
                data.popleft()

    threading.Thread(target=reader, daemon=True).start()
    fig, ax = plt.subplots(figsize=(10, 5))
    fig.subplots_adjust(bottom=0.18)
    lx, = ax.plot([], [], label="eje x IMU")
    ly, = ax.plot([], [], label="eje y IMU")
    lt, = ax.plot([], [], "k", lw=2, label="total")
    ax.axhline(10, color="r", ls=":", lw=1)
    ax.axhline(-10, color="r", ls=":", lw=1)
    ax.set_xlabel("s"); ax.set_ylabel("inclinacion de la base (grados)"); ax.grid(alpha=0.3); ax.legend(loc="upper left")
    txt = ax.text(0.99, 0.95, "", transform=ax.transAxes, ha="right", va="top", fontsize=22, family="monospace")

    def cero(_):
        if last[0]:
            ref[:] = [last[0][1], last[0][2]]

    btn = Button(fig.add_axes([0.80, 0.03, 0.15, 0.07]), "Cero aqui")
    btn.on_clicked(cero)

    def update(_):
        d = list(data)
        if not d:
            txt.set_text("esperando IMU...")
            return lx, ly, lt, txt
        t0 = d[-1][0]
        ts = [r[0] - t0 for r in d]
        an = [angles(r[1:], ref) for r in d]
        for ln, k in ((lx, 0), (ly, 1), (lt, 2)):
            ln.set_data(ts, [v[k] for v in an])
        ax.set_xlim(-a.ventana, 0)
        lo = min(min(v[:2]) for v in an); hi = max(v[2] for v in an)
        ax.set_ylim(min(-12, lo - 1), max(12, hi + 1))
        x, y, tot = an[-1]
        txt.set_text(f"{tot:5.1f}°\nx {x:+5.1f}  y {y:+5.1f}")
        return lx, ly, lt, txt

    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    fig.canvas.manager.set_window_title("Inclinacion de la base")
    plt.show()


if __name__ == "__main__":
    main()
