"""Probeta de tolerancia de la rótula impresa (imprimir ANTES que las bielas).

Una pieza con:
  - 3 casquillos iguales al de la biela pero con cavidad Ø14.3 / Ø14.5 / Ø14.7
    (holgura radial 0.15 / 0.25 / 0.35 mm). Se distinguen por 1, 2 o 3
    muescas en el borde (nada de texto chico que no se imprime bien).
  - 1 perno con bola Ø14 sobre una base de 8 mm, con el mismo alma M3.

Cómo usarla: calzar la bola en cada casquillo. El bueno es el que entra con
presión de dedos y gira suave sin juego. Si es el de 1 o 3 muescas, cambiar
CAV_D en common/rotula.py (14.3 o 14.7) y regenerar biela.py.
Imprimir plana (casquillos) con el mismo material, capa y velocidad que la
biela; el perno de pie como la manivela.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))

from build123d import Box, Compound, Cylinder, Pos  # noqa: E402

import dfm  # noqa: E402
from rotula import (BALL_D, ROD_W, SOCKET_OD, SOCKET_T, ball_stud, ball_stud_cut, socket_blank,  # noqa: E402
                    socket_cut)

CAVS = (14.3, 14.5, 14.7)
PITCH = 28.0
BASE_T = 8.0


def gen_sockets():
    n = len(CAVS)
    part = Pos((n - 1) * PITCH / 2, 0, 0) * Box((n - 1) * PITCH, ROD_W, SOCKET_T)
    for i in range(n):
        part += Pos(i * PITCH, 0, 0) * socket_blank()
    part, _, _ = dfm.fillet_concave(part, 3.0)
    for i, cav in enumerate(CAVS):
        part -= Pos(i * PITCH, 0, 0) * socket_cut(slot_dir=(0, 1), cav_d=cav)
        for k in range(i + 1):   # muescas de identificación en el borde -Y
            part -= Pos(i * PITCH - 3 + 3 * k, -SOCKET_OD / 2, 0) * Cylinder(0.9, SOCKET_T + 2)
    part.label = "probeta_casquillos_14.3_14.5_14.7"
    return part


def gen_stud():
    base = Pos(0, 0, -BASE_T / 2) * Box(22, 22, BASE_T)
    base = base.fillet(2.0, base.edges().filter_by(lambda e: abs(e.tangent_at(0.5).Z) > 0.9))
    part = base + ball_stud()
    part -= ball_stud_cut(BASE_T)
    part.label = f"probeta_perno_bola{BALL_D:.0f}"
    return part.moved(Pos(-40, 0, BASE_T))


def gen_step():
    return Compound(children=[gen_sockets(), gen_stud()], label="probeta_rotula")
