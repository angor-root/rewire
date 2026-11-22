#!/usr/bin/env python3
"""Interfaz del modo DSM (myRIO -> PRIZM): armar/parar, ver en vivo, grabar y graficar logs.

    python3 ~/dsm/prizm_pro/tools/dsm_gui.py [--port /dev/ttyACM1 | wifi | <IP de la PRIZM>] [--pi <PI_IP>]

Pide 'D T L' al conectar (una fila por linea del myRIO). Grabar escribe las filas en logs/ con el mismo formato
que dsm_log.py; "Graficar" hace el .png del ultimo log.
28-oct: rampa ('D A'), fusion encoders + IMU ('D F', filas 'F'), geometria r/b ('U G', la misma del VI: la fusion
pasa la odometria a cuentas con ella). Al grabar, ademas del CSV de filas L:
  logs/dsm_<fecha>_<nombre>_fusion.csv   filas F del filtro
  logs/dsm_<fecha>_<nombre>_todo.txt     TODAS las lineas recibidas, crudas, con la hora de la laptop
  ~/dsm/logs/<fecha>_<nombre>/           con "Pi" marcado (PRIZM conectada a la Pi): imu_log.csv (100 Hz),
                                         ball_log.txt (pelota, PD y TODAS las lineas de la PRIZM con la hora de la Pi),
                                         video, series.png / plato.png / resumen.txt (deploy/tools/sesion.py)
"""
import argparse
import math
import csv
import sys
import glob
import os
import queue
import socket
import subprocess
import threading
import time
import tkinter as tk
import webbrowser
from tkinter import messagebox, simpledialog, ttk

import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import dsm_log

import sesion  # noqa: E402  grabacion en la Pi (IMU, pelota, video, lineas de la PRIZM)
from inclinacion_base import angles  # noqa: E402  inclinacion de la base respecto de la referencia de nivel

IX = {h: i for i, h in enumerate(dsm_log.COLS)}
IF = {h: i for i, h in enumerate(dsm_log.COLS_F)}
EXTRAS = "xr,yr,thr,vr,wr,ex,ey,eth,x,y,th"  # orden de la trama del VI (19-oct)
# m: radio de rueda y via por defecto: poner LOS MISMOS QUE EL VI (se mandan con 'U G' al conectar).
# 13-nov: 0.0514/0.335 (medidos 11-nov) = r/via real con 0.2 % segun el giroscopio de un celular en una vuelta de 360
R_W, B_W = 0.0514, 0.335
MAX_PTS = 4000  # puntos por curva al dibujar (el historial completo se guarda; solo se submuestrea el dibujo)


class UdpLink:
    """Misma interfaz que serial.Serial (write / readline / close) pero por WiFi: firmware net_link.cpp, UDP 5005.
    host = 'wifi' busca la PRIZM por broadcast; o la IP que da 'N ?'. La PRIZM le manda el log a quien le escribe."""

    def __init__(self, host):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", 0))
        self.sock.settimeout(0.1)
        self.addr = ("255.255.255.255" if host == "wifi" else host, 5005)
        self.lines, self.is_open = [], True

    def write(self, data):
        self.sock.sendto(data, self.addr)

    def readline(self):
        if not self.lines:
            try:
                data, a = self.sock.recvfrom(2048)
            except socket.timeout:
                return b""
            if a[1] == 5005:
                self.addr = a  # tras el broadcast queda fija la IP de la PRIZM
            self.lines = data.split(b"\n")
        return self.lines.pop(0) + b"\n"

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False
        self.sock.close()


def is_net(port):
    return port == "wifi" or port.replace(".", "").isdigit()


class App:
    def __init__(self, root, port, pi_ip):
        self.root, self.ser, self.q = root, None, queue.Queue()
        self.rec = self.rec_f = None
        self.rec_ff = self.rec_fw = self.rec_raw = None
        self.rec_ball = None  # _pelota.csv: filas b, de ball_pixel por el ssh de ball_reader (30-oct)
        # 09-nov: copia EN VIVO de la Pi (sobrevive a que la apaguen antes de bajar): _pi.txt = todo /tmp/ball.log menos
        # las filas p (la PRIZM ya llega por WiFi) y _imu.csv = /tmp/imu_log.csv de imu_pi (120 Hz)
        self.rec_pi = self.imu_proc = None
        self.rows_f = []  # filas F del filtro
        self.pi_t0 = None
        self.last_rx = time.time()
        self.ms_off = 0.0
        self.rows = []  # todo desde que se conecto (o desde "Limpiar" / el inicio de una grabacion)
        self.n_win, self.t_win = 0, time.time()
        root.title("DSM — myRIO ↔ PRIZM")

        top = ttk.Frame(root, padding=4)
        top.pack(fill="x")
        self.port = tk.StringVar(value=port)
        ttk.Entry(top, textvariable=self.port, width=14).pack(side="left")
        ttk.Button(top, text="Conectar", command=self.connect).pack(side="left")
        # cuentas y odometria a cero = reiniciar la PRIZM antes de cada prueba (la interfaz se reconecta sola)
        for txt, cmd in (("Armar (D 1)", "D 1"), ("PARAR (D 0)", "D 0"), ("Estado", "D ?")):
            ttk.Button(top, text=txt, command=lambda c=cmd: self.send(c)).pack(side="left")
        ttk.Button(top, text="Configurar WiFi", command=self.wifi_setup).pack(side="left")
        self.wmax = tk.StringVar(value="6")
        ttk.Label(top, text="  wmax").pack(side="left")
        ttk.Entry(top, textvariable=self.wmax, width=5).pack(side="left")
        ttk.Button(top, text="Aplicar", command=lambda: self.send(f"D L {self.wmax.get()} 200")).pack(side="left")

        fus = ttk.Frame(root, padding=4)  # 28-oct: rampa, fusion, geometria
        fus.pack(fill="x")
        self.acc = tk.StringVar(value="8")
        ttk.Label(fus, text="rampa (rad/s² de rueda, 0 = escalón)").pack(side="left")
        ttk.Entry(fus, textvariable=self.acc, width=5).pack(side="left")
        ttk.Button(fus, text="Aplicar", command=lambda: self.send(f"D A {self.acc.get()}")).pack(side="left")
        self.fuse = tk.BooleanVar(value=False)
        ttk.Checkbutton(fus, text="dN fusionados al myRIO (D F)", variable=self.fuse,
                        command=lambda: self.send(f"D F {int(self.fuse.get())}")).pack(side="left", padx=8)
        self.dpsi_imu = tk.BooleanVar(value=False)
        ttk.Checkbutton(fus, text="rumbo del giroscopio (D P)", variable=self.dpsi_imu,
                        command=lambda: self.send(f"D P {int(self.dpsi_imu.get())}")).pack(side="left", padx=8)
        ttk.Button(fus, text="Filtro (D E)", command=lambda: self.send("D E")).pack(side="left")
        self.r_w, self.b_w = tk.StringVar(value=str(R_W)), tk.StringVar(value=str(B_W))
        ttk.Label(fus, text="   r").pack(side="left")
        ttk.Entry(fus, textvariable=self.r_w, width=7).pack(side="left")
        ttk.Label(fus, text="vía").pack(side="left")
        ttk.Entry(fus, textvariable=self.b_w, width=6).pack(side="left")
        ttk.Button(fus, text="= VI (U G)", command=self.send_geom).pack(side="left")
        self.fstat = tk.StringVar(value="")
        ttk.Label(fus, textvariable=self.fstat, font=("monospace", 10)).pack(side="left", padx=8)

        rec = ttk.Frame(root, padding=4)
        rec.pack(fill="x")
        self.name = tk.StringVar(value="prueba")
        self.extras = tk.StringVar(value=EXTRAS)
        ttk.Label(rec, text="nombre").pack(side="left")
        ttk.Entry(rec, textvariable=self.name, width=16).pack(side="left")
        ttk.Label(rec, text=" extras del VI").pack(side="left")
        ttk.Entry(rec, textvariable=self.extras, width=34).pack(side="left")
        self.rec_btn = ttk.Button(rec, text="● Grabar", command=self.toggle_rec)
        self.rec_btn.pack(side="left")
        ttk.Button(rec, text="Graficar último", command=self.plot_last).pack(side="left")
        ttk.Button(rec, text="Limpiar gráficos", command=lambda: (self.rows.clear(), self.rows_f.clear())).pack(side="left")
        self.pi_on, self.pi_video = tk.BooleanVar(value=True), tk.BooleanVar(value=False)
        self.pi_ip = tk.StringVar(value=pi_ip)
        ttk.Checkbutton(rec, text="Pi (IMU, pelota, todo)", variable=self.pi_on).pack(side="left", padx=(10, 0))
        ttk.Checkbutton(rec, text="video", variable=self.pi_video).pack(side="left")
        ttk.Entry(rec, textvariable=self.pi_ip, width=14).pack(side="left")
        # 30-oct: ON arranca el detector si no corre (pelota.sh iniciar), OFF nivela el plato (el detector sigue)
        ttk.Button(rec, text="Pelota ON", command=lambda: self.pi_sh("iniciar")).pack(side="left")
        ttk.Button(rec, text="Pelota OFF", command=lambda: self.pi_sh("parar")).pack(side="left")
        # 09-nov: nivel de la IMU = como esta ahora (robot quieto en plano); queda guardado en la Pi (~/imu_ref.txt)
        ttk.Button(rec, text="Nivel IMU", command=lambda: self.pi_sh("", "~/nivel_imu.sh")).pack(side="left")
        ttk.Button(rec, text="Ver cámara", command=lambda: webbrowser.open(
            f"http://{self.pi_ip.get()}:8080")).pack(side="left")  # video en vivo de ball_pixel

        self.status = tk.StringVar(value="sin conexión")
        st = ttk.Frame(root)
        st.pack(fill="x")
        # 09-nov: bateria de la PRIZM como icono de pila (lineas B cada 2 s)
        self.bat_cv = tk.Canvas(st, width=270, height=26, highlightthickness=0)
        self.bat_cv.pack(side="left", padx=4)
        ttk.Label(st, textvariable=self.status, font=("monospace", 10), padding=4).pack(side="left", fill="x")
        self.draw_battery(None)
        # 11-nov: inclinacion de la base en vivo (IMU de la Pi menos el nivel de "Nivel IMU"): robot de perfil + grados
        self.incl_cv = tk.Canvas(st, width=330, height=64, highlightthickness=0)
        self.incl_cv.pack(side="right", padx=4)
        self.ball = tk.StringVar(value="pelota: sin datos de la Pi")
        ttk.Label(root, textvariable=self.ball, font=("monospace", 11, "bold"), padding=(4, 0)).pack(fill="x")
        self.ball_rows = []  # (t, vista, x, y) de las filas b, de /tmp/ball.log
        self.imu_ref, self.incl = [0.0, 0.0], None
        self.vbat = None  # (t, voltios) de las lineas B de la PRIZM  # inclinacion de la base: (t, (eje x, eje y, total) grados)
        threading.Thread(target=self.ball_reader, daemon=True).start()
        self.vi = tk.StringVar(value="")  # los campos extra del VI, por nombre
        ttk.Label(root, textvariable=self.vi, font=("monospace", 10), padding=(4, 0)).pack(fill="x")

        self.fig = Figure(figsize=(14, 8.5))
        self.axs = [self.fig.add_subplot(4, 3, i + 1) for i in range(12)]
        self.canvas = FigureCanvasTkAgg(self.fig, root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        bot = ttk.Frame(root, padding=4)
        bot.pack(fill="x")
        self.log = tk.Text(bot, height=6, font=("monospace", 9))
        self.log.pack(fill="x")
        self.cmd = tk.StringVar()
        e = ttk.Entry(bot, textvariable=self.cmd)
        e.pack(fill="x")
        e.bind("<Return>", lambda _: (self.send(self.cmd.get()), self.cmd.set("")))
        root.bind("<Escape>", lambda _: self.send("D 0"))
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.connect()
        self.root.after(100, self.poll)
        self.root.after(300, self.redraw)

    def draw_battery(self, v):
        """Pila: TETRIX 12 V NiMH, 11.0 V = vacia, 13.5 V = llena; bajo 11.5 V la PRIZM se reinicia con los picos."""
        c = self.bat_cv
        c.delete("all")
        c.create_rectangle(2, 3, 92, 23, width=2)
        c.create_rectangle(92, 9, 97, 17, fill="black")
        if v is None:
            c.create_text(104, 13, text="batería: sin dato", anchor="w", font=("monospace", 9))
            return
        f = min(max((v - 11.0) / 2.5, 0.0), 1.0)
        col = "#2e9d3a" if v >= 12.2 else "#e0a800" if v >= 11.5 else "#d62828"
        if f > 0:
            c.create_rectangle(4, 5, 4 + 86 * f, 21, fill=col, outline="")
        c.create_text(47, 13, text=f"{100 * f:.0f} %", font=("monospace", 9, "bold"))
        c.create_text(104, 13, text=f"{v:.2f} V" + ("  ⚠ CAMBIAR" if v < 11.5 else ""), anchor="w",
                      font=("monospace", 10, "bold"), fill=col if v < 11.5 else "black")

    def draw_incl(self):
        """Robot de perfil girado lo que mide la IMU en el eje x (adelante/atras; + = frente arriba, como al subir la
        rampa del 10-nov) y el numero grande; el eje y (lateral) va en chico."""
        c, inc = self.incl_cv, self.incl
        c.delete("all")
        c.create_line(8, 40, 122, 40, fill="#bbb", dash=(3, 3))  # horizontal
        if not inc or time.time() - inc[0] > 1:
            c.create_text(135, 32, anchor="w", text="sin IMU", font=("monospace", 14, "bold"), fill="#888")
            return
        ax, ay, _ = inc[1]
        a = math.radians(ax)
        ca, sa = math.cos(a), math.sin(a)
        rot = lambda x, y: (65 + x * ca + y * sa, 40 - x * sa + y * ca)  # frente a la derecha, pantalla con y hacia abajo
        col = "#2e7d32" if abs(ax) < 3 else "#f9a825" if abs(ax) < 8 else "#c62828"
        c.create_line(*rot(-55, 12), *rot(55, 12), fill="#555", width=2)                      # piso
        c.create_polygon(*rot(-38, 2), *rot(38, 2), *rot(38, -12), *rot(-38, -12), fill=col, outline="")  # chasis
        c.create_line(*rot(-20, -12), *rot(-20, -24), *rot(20, -24), *rot(20, -12), fill="#222", width=2)  # plato
        for wx in (-26, 26):
            x, y = rot(wx, 4)
            c.create_oval(x - 8, y - 8, x + 8, y + 8, fill="#222", outline="")
        c.create_line(*rot(40, -5), *rot(54, -5), arrow="last", fill=col, width=2)             # frente
        c.create_text(135, 24, anchor="w", text=f"{ax:+5.1f}°", font=("monospace", 22, "bold"), fill=col)
        c.create_text(135, 52, anchor="w", text=f"adelante/atrás   lat {ay:+.1f}°", font=("monospace", 9))

    def wifi_setup(self):
        """Guarda la red en la PRIZM ('N S red clave', por el cable USB) y pregunta el estado ('N ?')."""
        if not self.ser:
            self.say("primero conecta la PRIZM por el cable USB")
            return
        red = simpledialog.askstring("WiFi de la PRIZM", "Nombre de la red (hotspot), SIN espacios:", parent=self.root)
        if not red:
            return
        if " " in red.strip():
            messagebox.showerror("WiFi", "El nombre de la red no puede tener espacios: cámbialo en el celular.")
            return
        clave = simpledialog.askstring("WiFi de la PRIZM", f"Clave de '{red.strip()}':", parent=self.root, show="*")
        if clave is None:
            return
        self.ser.write(f"N S {red.strip()} {clave.strip()}\n".encode())  # sin say(): no mostrar la clave
        self.say(f"> N S {red.strip()} ****")
        self.root.after(6000, lambda: self.send("N ?"))  # unos segundos para conectarse
        self.say("conectando la PRIZM a la red... en 6 s pregunto el estado (debe decir conectado=1 y una ip)")

    def say(self, s):
        self.log.insert("end", s + "\n")
        self.log.see("end")

    def send_geom(self):
        """La PRIZM pasa la odometria fusionada a cuentas con r y via: tienen que ser los del VI."""
        self.send(f"U G {self.r_w.get()} {self.b_w.get()}")

    def pi_cmd(self, c):
        """Comando a ball_pixel por su FIFO (PD 1 / PD 0) o a la PRIZM a traves de el; muestra la respuesta (OK/ERR que
        ball_pixel guardo en /tmp/ball.log)."""
        def run():
            out = sesion.ssh_ok(self.pi_ip.get(), sesion.FIFO.format(c) + "; sleep 0.6; tail -n 3000 /tmp/ball.log | "
                                "grep -a -E '^p,[0-9.]+,(OK|ERR|WD|AVISO)' | tail -n 2 | cut -d, -f3-")
            if out is None:
                self.q.put(f"Pi> {c}  (ERROR: ¿Pi apagada o ball_pixel parado?)")
                return
            self.q.put(f"Pi> {c}")
            for l in out.strip().splitlines():
                self.q.put("  " + l)
        threading.Thread(target=run, daemon=True).start()

    def pi_sh(self, arg, cmd=None):
        """~/pelota.sh iniciar | parar (o cmd) en la Pi; muestra lo que contesta."""
        cmd = cmd or f"~/pelota.sh {arg}"

        def run():
            out = sesion.ssh_ok(self.pi_ip.get(), cmd + " 2>&1; true", timeout=20)
            self.q.put(f"Pi> {cmd}" + ("  (ERROR: ¿Pi apagada?)" if out is None else ""))
            for l in (out or "").strip().splitlines():
                self.q.put("  " + l)
        threading.Thread(target=run, daemon=True).start()

    def ball_reader(self):
        """Filas 'b,<t>,<vista>,<u>,<v>,<x m>,<y m>,...' de ball_pixel por un ssh que queda abierto (tail -F sigue al
        log aunque se reinicie el detector); si se corta, reintenta cada 2 s."""
        while True:
            if not self.pi_on.get():
                time.sleep(1)
                continue
            # 09-nov: tambien las filas i (gravedad de imu_pi) y OK IMU (referencia de nivel) -> inclinacion de la base
            p = subprocess.Popen(["ssh", "-o", f"HostName={self.pi_ip.get()}", "-o", "ConnectTimeout=5",
                                  "-o", "ServerAliveInterval=2", "-o", "BatchMode=yes", "-l", "robot", "dsm-pc2",
                                  "grep -a 'OK IMU' /tmp/ball.log | tail -1; "
                                  "tail -n 0 -F /tmp/ball.log 2>/dev/null | grep -a --line-buffered -v '^p,'"],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            for line in p.stdout:
                v = line.split(",")
                fp = self.rec_pi
                try:
                    if fp:
                        fp.write(f"{time.time():.3f} {line}")
                except ValueError:  # se cerro justo ahora
                    pass
                if v[0] not in ("b", "i") and not line.startswith("OK IMU"):  # lo demas solo va al archivo
                    continue
                if line.startswith("OK IMU"):
                    k = line.split("ref=")
                    if len(k) == 2:
                        self.imu_ref = [float(x) for x in k[1].split()[:2]]
                    continue
                if v[0] == "i":
                    try:
                        self.incl = (time.time(), angles([float(x) for x in v[2:5]], self.imu_ref))
                    except (IndexError, ValueError):
                        pass
                    continue
                try:
                    self.ball_rows.append((time.time(), int(v[2]), float(v[5]), float(v[6])))
                except (IndexError, ValueError):
                    continue
                f = self.rec_ball
                try:
                    if f:  # t_pc, t de la Pi, vista, u, v (px), x, y (m), diametro (px), relleno
                        f.write(f"{time.time():.3f},{line.strip()[2:]}\n")
                except ValueError:  # se cerro justo ahora
                    pass
                del self.ball_rows[:-200]
            p.wait()
            time.sleep(2)

    def ball_text(self):
        r = [b for b in self.ball_rows if time.time() - b[0] < 2]
        if not r:
            return "pelota: sin datos (¿Pi apagada o detector parado? → Pelota ON)"
        seen = [b for b in r if b[1]]
        pos = (f"x {100 * seen[-1][2]:+5.1f} cm   y {100 * seen[-1][3]:+5.1f} cm   dist. al centro "
               f"{100 * (seen[-1][2] ** 2 + seen[-1][3] ** 2) ** 0.5:4.1f} cm" if seen else "NO SE VE")
        inc = self.incl
        base = (f"   |   BASE {inc[1][2]:4.1f}° (x {inc[1][0]:+5.1f}  y {inc[1][1]:+5.1f})"
                if inc and time.time() - inc[0] < 1 else "   |   base: sin IMU")
        return f"pelota: {pos}   |   vista {100 * len(seen) / len(r):3.0f} %  ({len(r) / 2:.0f} Hz)" + base

    def connect(self, quiet=False):
        try:
            if self.ser:
                self.ser.close()
            self.ser = UdpLink(self.port.get()) if is_net(self.port.get()) else serial.Serial(self.port.get(), 115200,
                                                                                               timeout=0.1)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            threading.Thread(target=self.reader, args=(self.ser,), daemon=True).start()
            self.send("D T L")
            self.send_geom()
            self.say(f"conectado a {self.port.get()}")
        except (serial.SerialException, OSError) as ex:
            self.ser = None
            if not quiet:
                self.say(f"ERROR: {ex}")

    def reconnect(self):
        """Tras reiniciar la PRIZM el USB desaparece unos segundos (y puede volver como otro ttyACM)."""
        if self.ser:
            return
        ports = sorted(glob.glob("/dev/ttyACM*"))
        if ports:
            self.port.set(ports[0])
            self.connect(quiet=True)
        if not self.ser:
            self.root.after(1000, self.reconnect)

    def reader(self, s):
        while s.is_open:
            try:
                line = s.readline().decode(errors="replace").strip()
            except (serial.SerialException, OSError, TypeError):
                if s is self.ser:  # si es un enlace viejo (Conectar de nuevo), no es una desconexion
                    self.q.put("__PERDIDO__")
                return
            if line:
                self.q.put(line)

    def send(self, c):
        if not c.strip():
            return
        if not self.ser:
            self.say("sin conexión")
            return
        self.ser.write((c.strip() + "\n").encode())
        self.say("> " + c.strip())

    def toggle_rec(self):
        if self.rec:
            ball, self.rec_ball = self.rec_ball, None
            fpi, self.rec_pi = self.rec_pi, None
            for f in (self.rec_f, self.rec_ff, self.rec_raw, ball, fpi):
                f.close()
            if self.imu_proc:
                self.imu_proc.terminate()
                self.imu_proc = None
            self.say(f"guardado {self.rec_path} (+ _fusion.csv, _todo.txt)")
            self.rec = self.rec_f = self.rec_ff = self.rec_fw = self.rec_raw = None
            self.rec_btn.config(text="● Grabar")
            if self.pi_t0 is not None:
                t0, self.pi_t0 = self.pi_t0, None
                threading.Thread(target=self.pi_finish, args=(t0, self.pi_dir, self.pi_vid)).start()
            return
        os.makedirs(dsm_log.LOGS, exist_ok=True)
        ex = [e.strip() for e in self.extras.get().split(",") if e.strip()]
        ex += [f"x{i + 1}" for i in range(len(ex), dsm_log.N_EXTRA)]
        stamp = time.strftime("%Y%m%d_%H%M%S") + (f"_{self.name.get()}" if self.name.get() else "")
        self.rec_path = os.path.join(dsm_log.LOGS, f"dsm_{stamp}.csv")
        self.rec_f = open(self.rec_path, "w", newline="")
        self.rec = csv.writer(self.rec_f)
        self.rec.writerow(["t_pc"] + dsm_log.COLS + ex)
        self.rec_ff = open(self.rec_path[:-4] + "_fusion.csv", "w", newline="")
        self.rec_fw = csv.writer(self.rec_ff)
        self.rec_fw.writerow(["t_pc"] + dsm_log.COLS_F)
        self.rec_raw = open(self.rec_path[:-4] + "_todo.txt", "w")
        ball = open(self.rec_path[:-4] + "_pelota.csv", "w")
        ball.write("t_pc,t_pi,vista,u,v,x,y,diam,relleno\n")
        self.rec_ball = ball
        self.rec_pi = open(self.rec_path[:-4] + "_pi.txt", "w")  # t_pc + linea cruda de /tmp/ball.log
        if self.pi_on.get():  # el log de imu_pi empieza con la grabacion (USR1 en pi_begin): seguirlo desde el inicio
            self.imu_proc = subprocess.Popen(
                ["ssh", "-o", f"HostName={self.pi_ip.get()}", "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=2",
                 "-o", "BatchMode=yes", "-l", "robot", "dsm-pc2", "sleep 2; tail -n +1 -F /tmp/imu_log.csv 2>/dev/null"],
                stdout=open(self.rec_path[:-4] + "_imu.csv", "w"), stderr=subprocess.DEVNULL)
        self.rec_btn.config(text="■ Detener")
        self.rows.clear()  # el grafico arranca con la grabacion: lo que se ve = lo que queda en el CSV
        self.rows_f.clear()
        self.say(f"grabando {self.rec_path} (+ _pi.txt y _imu.csv en vivo desde la Pi)")
        self.send("D ?")  # el estado y la configuracion quedan al principio de _todo.txt
        if self.pi_on.get():
            self.pi_dir = os.path.join(sesion.ROOT, stamp)
            self.pi_vid = self.pi_video.get()
            self.pi_t0 = -1.0  # pendiente: el hilo pone el t0 real
            threading.Thread(target=self.pi_begin, daemon=True).start()

    def pi_begin(self):
        try:
            t0, avisos = sesion.pi_start(self.pi_ip.get(), self.pi_vid)
        except (RuntimeError, SystemExit, ValueError) as ex:
            self.pi_t0 = None
            self.q.put(f"Pi: NO graba ({ex})")
            return
        for w in avisos:
            self.q.put("Pi: AVISO " + w)
        if self.pi_t0 is None:  # se detuvo antes de que la Pi contestara
            threading.Thread(target=self.pi_finish, args=(t0, self.pi_dir, self.pi_vid)).start()
        else:
            self.pi_t0 = t0
            self.q.put(f"Pi: grabando (t0 {t0:.2f})")

    def pi_finish(self, t0, out, video):
        while t0 is not None and t0 < 0:  # todavia arrancando
            time.sleep(0.2)
            t0 = self.pi_t0 if self.pi_t0 is not None else t0
        try:
            self.q.put("Pi: bajando datos ...")
            t1 = sesion.pi_stop(self.pi_ip.get(), out, t0, video)
            # en otro proceso: pyplot desde un hilo choca con Tk
            subprocess.run([sys.executable, sesion.__file__, "graficar", os.path.basename(out), "--no-mostrar"],
                           env=dict(os.environ, MPLBACKEND="Agg"), capture_output=True, timeout=300)
            self.q.put(f"Pi: {t1 - t0:.1f} s guardados en {out} (series.png, plato.png, resumen.txt)")
        except (RuntimeError, SystemExit, ValueError, OSError) as ex:
            self.q.put(f"Pi: ERROR al terminar ({ex})")

    def plot_last(self):
        try:
            dsm_log.graficar(argparse.Namespace(csv=None, ver=False))
            png = max((os.path.join(dsm_log.LOGS, f) for f in os.listdir(dsm_log.LOGS) if f.endswith(".png")),
                      key=os.path.getmtime)
            os.startfile(png) if sys.platform == "win32" else subprocess.Popen(["xdg-open", png])
        except (ValueError, SystemExit) as ex:
            self.say(f"no hay log para graficar ({ex})")

    def poll(self):
        if self.ser and isinstance(self.ser, UdpLink) and time.time() - self.last_rx > 2:
            self.ser.write(b"D T L\n")  # WiFi sin datos (PRIZM reiniciada o sin destino): volver a registrarse
            self.last_rx = time.time()
        while not self.q.empty():
            line = self.q.get()
            self.last_rx = time.time()
            if line == "__PERDIDO__":
                self.ser = None
                self.say("PRIZM desconectada (¿reinicio?): reconectando sola... (después, botón verde)")
                self.root.after(1000, self.reconnect)
                continue
            if self.rec_raw:
                self.rec_raw.write(f"{time.time():.3f} {line}\n")
            if line.startswith("B,"):  # 09-nov: bateria cada 2 s (queda en _todo.txt)
                try:
                    self.vbat = (time.time(), float(line.split(",")[2]))
                except (IndexError, ValueError):
                    pass
                continue
            if line[:2] in ("T,", "E,", "d,", "h,"):  # telemetria: solo al archivo crudo
                continue
            if line.startswith("F,"):
                v = line[2:].split(",")
                if len(v) >= len(dsm_log.COLS_F):
                    try:
                        self.rows_f.append([float(x) for x in v])
                    except ValueError:
                        continue
                    if self.rec_fw:
                        self.rec_fw.writerow([f"{time.time():.3f}"] + v)
                continue
            if not line.startswith("L,"):
                self.say(line)
                continue
            v = line[2:].split(",")
            if len(v) < len(dsm_log.COLS):
                continue
            row = [float(x) for x in v]
            if self.rows and row[0] + self.ms_off < self.rows[-1][0]:  # PRIZM reiniciada: su reloj volvio a 0
                self.ms_off = self.rows[-1][0] - row[0] + 20
            row[0] += self.ms_off
            self.rows.append(row)
            self.n_win += 1
            if self.rec:
                self.rec.writerow([f"{time.time():.3f}"] + v)
        dt = time.time() - self.t_win
        if dt >= 1:
            hz, self.n_win, self.t_win = self.n_win / dt, 0, time.time()
            self.ball.set(self.ball_text())
            self.draw_incl()
            r = self.rows[-1] if self.rows else None
            self.draw_battery(self.vbat[1] if self.vbat and time.time() - self.vbat[0] < 6 else None)
            self.status.set(f"{hz:5.1f} Hz  " + (
                f"run={int(r[IX['run']])}  wref=({r[IX['wref_R']]:+.2f}, {r[IX['wref_L']]:+.2f})  "
                f"w=({r[IX['w_R']]:+.2f}, {r[IX['w_L']]:+.2f})  u=({r[IX['u_R']]:+.0f}, {r[IX['u_L']]:+.0f})  "
                f"extra={int(r[IX['nx']])}"
                + ("   ● GRABANDO" if self.rec else "") if r and hz > 0 else
                "sin filas del myRIO (¿VI corriendo?)"))
            names = [e.strip() for e in self.extras.get().split(",") if e.strip()]
            nx = int(r[IX["nx"]]) if r else 0
            vals = r[len(dsm_log.COLS):] if r else []
            f = self.rows_f[-1] if self.rows_f else None
            nl = sum(1 for x in self.rows_f[-500:] if x[IF["sin_contacto"]] > 0.5)
            self.fstat.set(f"v enc {f[IF['v_enc']]:+.3f}  fus {f[IF['vx']]:+.3f} m/s  "
                           f"{'SIN CONTACTO' if f[IF['sin_contacto']] > 0.5 else 'contacto'}  "
                           f"pérdidas {int(f[IF['perdidas']])}" if f else
                           "sin filas F (¿IMU de la Pi? ¿firmware 28-oct?)")
            self.vi.set("VI: " + "  ".join(f"{n}={x:+.3f}" for n, x in zip(names, vals)) +
                        (f"   ⚠ llegan {nx} de {len(names)} extras" if r and nx != len(names) else ""))
        self.root.after(50, self.poll)

    def redraw(self):
        if self.rows:
            rows = self.rows[::max(1, len(self.rows) // MAX_PTS)]
            if rows[-1] is not self.rows[-1]:
                rows.append(self.rows[-1])
            col = lambda h: [r[IX[h]] for r in rows]
            names = [e.strip() for e in self.extras.get().split(",")]
            nan = float("nan")

            def ext(h):  # campo extra del VI (NaN en las filas que no lo traen)
                if h not in names:
                    return None
                k = len(dsm_log.COLS) + names.index(h)
                v = [r[k] if len(r) > k else nan for r in rows]
                return v if any(x == x for x in v) else None

            t = [(m - rows[0][0]) / 1000 for m in col("ms")]
            wR, wL = col("w_R"), col("w_L")
            v_med = [R_W * (a + b) / 2 for a, b in zip(wR, wL)]
            w_med = [R_W * (a - b) / B_W for a, b in zip(wR, wL)]
            dt = [b - a for a, b in zip(col("ms")[:-1], col("ms")[1:])]
            for ax in self.axs:
                ax.clear()
                ax.grid(alpha=0.3)
                ax.ticklabel_format(useOffset=False)
            panels = [  # (titulo, [(x, y, etiqueta, estilo)])
                ("rueda derecha (rad/s)", [(t, col("wref_R"), "ref", "-"), (t, wR, "medida", "-")]),
                ("rueda izquierda (rad/s)", [(t, col("wref_L"), "ref", "-"), (t, wL, "medida", "-")]),
                ("plano XY (m)", [(ext("x"), ext("y"), "odometría myRIO", "-"),
                                  (ext("xr"), ext("yr"), "ref", "--")]),
                ("errores del VI", [(t, ext(h), h, "-") for h in ("ex", "ey", "eth")]),
                ("potencia del PI (%)", [(t, col("u_R"), "der", "-"), (t, col("u_L"), "izq", "-")]),
                ("periodo entre líneas del myRIO (ms)", [(t[1:], dt, "", ".")]),
                ("v (m/s)", [(t, ext("vr"), "ref", "--"), (t, v_med, "medida (ruedas)", "-")]),
                ("w (rad/s)", [(t, ext("wr"), "ref", "--"), (t, w_med, "medida (ruedas)", "-")]),
                ("θ (rad)", [(t, ext("thr"), "ref", "--"), (t, ext("th"), "myRIO", "-")]),
            ]
            fr = self.rows_f[::max(1, len(self.rows_f) // MAX_PTS)]
            fcol = lambda h: [r[IF[h]] for r in fr]  # noqa: E731
            tf = [(r[0] + self.ms_off - rows[0][0]) / 1000 for r in fr]
            panels += [
                ("fusión: v (m/s)", [(tf, fcol("v_enc"), "encoders", "-"), (tf, fcol("vx"), "fusionada", "-")]
                 if fr else []),
                ("giro (rad/s)", [(tf, fcol("w_enc"), "encoders", "-"), (tf, fcol("w_gyro"), "giroscopio", "-")]
                 if fr else []),
                ("contacto / ventana χ²", [(tf, [10 * x for x in fcol("sin_contacto")], "sin contacto ×10", "-"),
                                          (tf, fcol("nis_win"), "χ² ventana (umbral 10.8)", "-")] if fr else []),
            ]
            for ax, (title, lines) in zip(self.axs, panels):
                ax.set_title(title, fontsize=9)
                for x, y, lbl, st in lines:
                    if x is not None and y is not None:
                        ax.plot(x, y, st, label=lbl, ms=2)
                if any(lbl for *_, lbl, _ in lines) and ax.lines:
                    ax.legend(loc="upper left", fontsize=7)
            self.axs[2].set_aspect("equal", "datalim")
            self.axs[4].axhline(100, color="r", lw=0.5)
            self.axs[4].axhline(-100, color="r", lw=0.5)
            self.axs[11].axhline(10.83, color="r", lw=0.5)
            self.fig.tight_layout()
            self.canvas.draw_idle()
        self.root.after(300, self.redraw)

    def close(self):
        if self.rec:
            self.toggle_rec()  # la bajada de la Pi sigue en un hilo no-daemon: el programa espera a que termine
        if self.ser:
            self.ser.write(b"D T 10\n")
            self.ser.close()
        self.root.destroy()


def pi_default():
    """La .171 de la subred por la que sale la laptop (no manda nada: UDP connect solo elige la ruta)."""
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as k:
            k.connect(("8.8.8.8", 80))
            return k.getsockname()[0].rsplit(".", 1)[0] + ".171"
    except OSError:
        return "10.217.151.171"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=dsm_log.default_port())
    ap.add_argument("--pi", default=pi_default(), help="IP de la Pi (grabacion de IMU, pelota y video); por defecto la .171 "
                    "de la red de la laptop (el hotspot cambia la subred, la Pi se queda en .171)")
    a = ap.parse_args()
    root = tk.Tk()
    App(root, a.port, a.pi)
    root.mainloop()
