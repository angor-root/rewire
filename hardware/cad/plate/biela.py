"""Biela (x2, idéntica para A y B): dos casquillos de rótula a 85 mm centro a
centro, ejes de casquillo paralelos (Z local). Rev. 2026-09-30.

Cuerpo 6.0 mm (espesor del casquillo) x 10 mm, unido a los casquillos con
filetes R4 (antes esquina a 90°). Cavidad Ø14.5 para bola Ø14 (0.25 mm de
holgura radial), boca Ø13.2 a presión con ranura de 1.2 mm (common/rotula.py).
Pandeo: Euler ~ 700 N >> 17 N que empuja el servo.
Marco: casquillo 1 en el origen, casquillo 2 en +X = ROD_L.
Impresión: plana, sin soportes (el techo de la cavidad es un anillo corto;
capa 0.2 mm). PETG.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))

from build123d import Box, Pos  # noqa: E402

import dfm  # noqa: E402
import kinematics as K  # noqa: E402
from rotula import ROD_W, SOCKET_OD, SOCKET_T, socket_blank, socket_cut  # noqa: E402

L = K.ROD_L
FIL = 4.0


def gen_step():
    body = Pos(L / 2, 0, 0) * Box(L - SOCKET_OD + 2, ROD_W, SOCKET_T)
    part = body + socket_blank() + Pos(L, 0, 0) * socket_blank()
    part, _, _ = dfm.fillet_concave(part, FIL)
    part -= socket_cut(slot_dir=(-1, 0))                 # ranuras hacia afuera
    part -= Pos(L, 0, 0) * socket_cut(slot_dir=(1, 0))
    part.label = f"biela_{L:.0f}mm"
    return part
