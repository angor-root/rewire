"""Araña del plato: una sola pieza que se atornilla bajo el plato y lleva
  - las orejas del eje Y del cardán (pitch), a y = ±(7.3 .. 11.3),
  - los dos pernos de rótula del plato, en Q_A y Q_B de kinematics.py.
Así la relación centro-de-giro <-> rótulas la fija la impresión, no el
taladrado del plato. 4 autorroscantes M3 x 10 avellanados desde el plato
(la araña sirve de plantilla para marcar los 4 agujeros).

Rev. 2026-09-30:
  - Brazos de 12 mm con extremos redondos (antes polígonos que dejaban
    escalones y astillas donde tocaban los soportes).
  - Soportes de bola de 8 mm (antes 6) y 16 mm de ancho, con 2 cartelas cada
    uno por detrás (la biela empuja la bola: el soporte trabaja en flexión).
  - Filetes en todas las uniones: R2..6 en la placa, R2.5 en oreja-cubo,
    brazo-soporte y cartelas.
  - Pernos de rótula Ø8 con alma de tornillo M3 x 25 y tuerca en trampa por
    detrás del soporte (common/rotula.py).
  - Tornillos al plato donde hay material debajo (sobre las orejas y sobre
    los soportes), no en el centro hueco.
  - Agujeros de las orejas Ø3.5: la araña gira sobre los M3 del bloque.

Marco: origen = centro del cardán, cara superior (contacto con el plato) en
z = PLATE_OFF. Imprimir boca abajo (cara del plato sobre la cama) con
soporte SOLO bajo las dos bolas (pernos horizontales: así las capas quedan a
lo largo del perno, que es la dirección fuerte).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))

import numpy as np  # noqa: E402
from build123d import Cylinder, Pos, Rot  # noqa: E402

import cardan_lib as C  # noqa: E402
import dfm  # noqa: E402
import kinematics as K  # noqa: E402
from dfm import box, cyl_y  # noqa: E402
from rotula import STUD_OFFSET, ball_stud, ball_stud_cut  # noqa: E402

PLATE_OFF = K.PLATE_BOTTOM_Z - float(K.PIVOT[2])    # 12.5
T = 4.0
Z_BOT = PLATE_OFF - T
EAR_Y_IN = C.BLOCK[1] / 2 + 0.3
EAR_T = 4.0
EAR_X = 5.0
EAR_R = 5.0
ARM_W = 12.0
BRK_T = 8.0
BRK_W = 16.0
BRK_DROP = 8.0          # el soporte baja 8 mm bajo el centro de la bola
GUS_T = 3.0
GUS_L = 8.0             # cateto horizontal de la cartela
GUS_H = 12.0            # cateto vertical
SCREW_PILOT = dfm.PILOT_M3
FIL = 2.5
HUB_X = 8.0             # medio ancho del cubo central (los brazos entran lejos de sus esquinas)

QA = K.q_A
QB = K.q_B


def bar(p0, p1, w, z0, z1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    n = np.array([-d[1], d[0]]) / np.linalg.norm(d) * w / 2
    return dfm.prism_xy([tuple(p0 + n), tuple(p1 + n), tuple(p1 - n), tuple(p0 - n)], z0, z1)


def gen_step():
    hub_y = EAR_Y_IN + EAR_T
    ya = QA[1] - STUD_OFFSET                 # cara del soporte A (perno hacia +Y)
    xb = QB[0] - STUD_OFFSET                 # cara del soporte B (perno hacia +X)
    a_top = (QA[0], ya - BRK_T / 2)
    b_top = (xb - BRK_T / 2, QB[1])
    # 1) placa: cubo + 2 brazos que terminan dentro de los soportes
    start = (0.0, 7.0)          # arranque alto: el brazo A pasa por encima de la oreja -X de la base
    slab = box(-HUB_X, HUB_X, -hub_y, hub_y + 1.5, Z_BOT, PLATE_OFF)
    # los brazos terminan con punta redonda centrada en la cara trasera de cada
    # soporte (la punta queda dentro del ancho del soporte)
    a_end = (QA[0], ya - BRK_T)
    b_end = (xb - BRK_T, QB[1])
    for p in (a_end, b_end):
        slab += bar(start, p, ARM_W, Z_BOT, PLATE_OFF) + Pos(p[0], p[1], (Z_BOT + PLATE_OFF) / 2) * Cylinder(ARM_W / 2, T)
    slab += box(QA[0] - BRK_W / 2, QA[0] + BRK_W / 2, ya - BRK_T, ya, Z_BOT, PLATE_OFF)
    slab += box(xb - BRK_T, xb, QB[1] - BRK_W / 2, QB[1] + BRK_W / 2, Z_BOT, PLATE_OFF)
    slab = slab.clean()
    part, n1, t1 = dfm.fillet_concave(slab, 6.0, radii=(6.0, 4.0, 3.0, 2.0, 1.0))
    part = dfm.round_convex(part, 3.0, axis=(0, 0, 1))
    # 2) soportes de las bolas (bajan de la placa)
    part += box(QA[0] - BRK_W / 2, QA[0] + BRK_W / 2, ya - BRK_T, ya, QA[2] - BRK_DROP, Z_BOT + 0.01)
    part += box(xb - BRK_T, xb, QB[1] - BRK_W / 2, QB[1] + BRK_W / 2, QB[2] - BRK_DROP, Z_BOT + 0.01)
    # 3) cartelas detrás de cada soporte (a los lados de la trampa de tuerca)
    for dx in (-(BRK_W / 2 - GUS_T / 2), BRK_W / 2 - GUS_T / 2):
        yb = ya - BRK_T
        pts = [(yb + 0.01, Z_BOT + 0.01), (yb - GUS_L, Z_BOT + 0.01), (yb + 0.01, Z_BOT - GUS_H)]
        part += dfm.prism_yz(pts, QA[0] + dx - GUS_T / 2, QA[0] + dx + GUS_T / 2)
        xbb = xb - BRK_T
        pts = [(xbb + 0.01, Z_BOT + 0.01), (xbb - GUS_L, Z_BOT + 0.01), (xbb + 0.01, Z_BOT - GUS_H)]
        part += dfm.prism_xz(pts, QB[1] + dx - GUS_T / 2, QB[1] + dx + GUS_T / 2)
    # 4) orejas del pitch (eje Y), cabeza redonda alrededor del eje
    for s in (-1, 1):
        y0, y1 = sorted((s * EAR_Y_IN, s * hub_y))
        part += box(-EAR_X, EAR_X, y0, y1, 0, Z_BOT + 0.01)
        part += cyl_y(EAR_R, y0, y1)
    # pernos de rótula (antes del filete: así la unión perno-soporte es un
    # booleano simple; las aristas del perno se excluyen del filete)
    loc_a = Pos(QA[0], ya, QA[2]) * Rot(-90, 0, 0)
    loc_b = Pos(xb, QB[1], QB[2]) * Rot(0, 90, 0)
    part += loc_a * ball_stud()
    part += loc_b * ball_stud()
    balls = [np.array(tuple((loc * Pos(0, 0, STUD_OFFSET)).position)) for loc in (loc_a, loc_b)]

    def keep(e):
        p = np.array(tuple(e.position_at(0.5)))
        if abs(p[0]) < EAR_X + 1.5 and abs(p[1]) < EAR_Y_IN - 0.1 and p[2] < Z_BOT - 2.5:
            return False           # caras internas de las orejas: el bloque de la cruceta pasa a 0.3 mm
        return min(np.linalg.norm(p - b) for b in balls) > 8.5

    part, n2, t2 = dfm.fillet_concave(part, FIL, keep=keep)
    n3 = 0
    n2, t2 = n2 + n3, t2
    print(f"[arana] filetes placa {n1}/{t1}, uniones {n2}/{t2} valid={part.is_valid}", file=sys.stderr)
    part -= loc_a * ball_stud_cut(BRK_T)
    part -= loc_b * ball_stud_cut(BRK_T)
    # ejes de las orejas (giran sobre los M3 del bloque)
    for s in (-1, 1):
        part -= cyl_y(C.PIVOT_D / 2, *sorted((s * (EAR_Y_IN - 1), s * (hub_y + 1))))
    # tornillos al plato (donde hay material debajo)
    holes = [(0.0, 9.3), (0.0, -9.3), a_top, b_top]
    for x, y in holes:
        # si el piloto cae en una costura de filete, correrlo hasta 1 mm
        for dx, dy in ((0, 0), (0.5, 0), (-0.5, 0), (0, 0.5), (0, -0.5), (1, 0), (-1, 0), (0, 1), (0, -1)):
            cand = part - Pos(x + dx, y + dy, PLATE_OFF - 4) * Cylinder(SCREW_PILOT / 2, 8.02)
            if cand.is_valid:
                part = cand
                break
        else:
            raise RuntimeError(f"piloto en {x:.1f},{y:.1f}: ningún corte válido")
    part.label = "arana_plato"
    return part


HOLES_NOTE = "4 x M3 x 10 avellanado; posiciones = agujeros piloto de la araña"
