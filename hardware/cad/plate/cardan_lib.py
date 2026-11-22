"""Cardán central (reemplaza la media bola "placeholder"): fija el centro de
giro del plato, bloquea el yaw y deja roll/pitch libres ±20°.  Rev. 2026-09-30.

Piezas (marco: origen = centro del cardán PIVOT, ejes = mundo):
  cardan_base   abrazadera en U que abraza el extremo del canal de 96 mm por
                3 lados (la cara abierta +Y de la C queda libre para meter las
                tuercas kep). 6 tornillos 6-32 x 5/16" a los agujeros Ø3.7 del
                canal en z = 107 y z = 91 (línea media de cada cara).
  cardan_cruceta barra Ø8 (eje X, roll) + bloque 10 x 14 x 9 (eje Y, pitch).
                Eje X: 1 tornillo M3 x 50 + tuerca autoblocante a través de
                oreja-barra-oreja (la barra GIRA sobre el tornillo: Ø3.5).
                Eje Y: 2 tornillos M3 x 8 desde las orejas de la araña a
                agujeros ciegos Ø2.7 del bloque (la araña gira sobre ellos).

Rev. 2026-09-30 (observaciones de impresión):
  - Orejas de la base: antes un escalón a 90° (ancho 16 -> 9 mm) justo bajo
    el eje; ahora se afinan con tangentes rectas hasta un cubo redondo R5
    alrededor del eje (sin escalón), 5 mm de espesor (antes 4).
  - Filetes R2 en las esquinas internas de la U (el canal tiene esquinas
    exteriores R2.5, así que entran) y en las uniones oreja-respaldo.
  - Cruceta: barra Ø8 (antes Ø7: pared de 1.75 mm alrededor del tornillo),
    transición cónica a 45° barra-bloque (antes esquina a 90°), aristas del
    bloque redondeadas R1. Se imprime DE PIE (barra vertical): el cono hace
    que la cara inferior del bloque no quede volada.
  - Holguras de giro: barra 0.4 mm por lado entre orejas; agujeros de giro Ø3.5.
El bloque y las orejas de la araña bajan DENTRO del hueco del canal al girar
(por eso el centro puede estar a solo 4.5 mm sobre el extremo del canal).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))

from build123d import Box, Cylinder, Pos, Rot  # noqa: E402

import dfm  # noqa: E402
from dfm import box, cyl_x, cyl_y  # noqa: E402
from dsm_lib import TOWER_OUT, TOWER_HOLES_Z, SCREW_632_CLR  # noqa: E402
import kinematics as K  # noqa: E402

PZ = float(K.PIVOT[2])
CLR = 0.2
EAR_IN = TOWER_OUT / 2 + CLR          # 16.25
EAR_T = 5.0
EAR_OUT = EAR_IN + EAR_T
EAR_TOP_R = 5.0                        # cubo redondo alrededor del eje X
EAR_W = 16.0
TAPER_Z = -20.0                        # bajo este z la oreja tiene el ancho completo
BACK_T = 3.0
BACK_TOP_Z = -8.0
BOT_Z = 85.0 - PZ
HOLE_Z = [z - PZ for z in TOWER_HOLES_Z if z > 85.0]   # 91, 107 -> -28.5, -12.5
SCREW_D = dfm.HOLE_CLR_M3              # tornillo fijo en las orejas
PIVOT_D = dfm.HOLE_PIVOT_M3            # la barra gira sobre el tornillo
FIL = 2.0

BAR_D = 8.0
BAR_GAP = 0.4                          # holgura axial por lado entre barra y oreja
BLOCK = (10.0, 14.0, 9.0)
COLLAR = 3.0                           # transición cónica 45° barra -> bloque
BLIND_D, BLIND_DEPTH = dfm.PILOT_M3, 5.0
PIN_D = PIVOT_D                        # (compat: lo usa la araña para sus orejas)


def gen_base():
    y0 = -EAR_IN - BACK_T
    prof = dfm.lug_profile(y0, EAR_W / 2, BOT_Z, TAPER_Z, (0.0, 0.0), EAR_TOP_R)
    part = None
    for s in (-1, 1):
        x0, x1 = sorted((s * EAR_IN, s * EAR_OUT))
        ear = dfm.prism_yz(prof, x0, x1) + cyl_x(EAR_TOP_R, x0, x1)
        part = ear if part is None else part + ear
    part += box(-EAR_OUT, EAR_OUT, y0, -EAR_IN, BOT_Z, BACK_TOP_Z)
    # filetes en todas las esquinas internas (U + transición respaldo-oreja)
    part, _, _ = dfm.fillet_concave(part, FIL)
    part = dfm.round_convex(part, 1.5, axis=(0, 0, 1))
    # agujeros después de filetear
    for s in (-1, 1):
        xc = s * (EAR_IN + EAR_OUT) / 2
        part -= cyl_x(SCREW_D / 2, xc - EAR_T, xc + EAR_T)
        for hz in HOLE_Z:
            part -= cyl_x(SCREW_632_CLR / 2, xc - EAR_T, xc + EAR_T, 0, hz)
    for hz in HOLE_Z:
        part -= cyl_y(SCREW_632_CLR / 2, y0 - 1, -EAR_IN + 1, 0, hz)
    part.label = "cardan_base"
    return part


def gen_cross():
    from build123d import Cone
    bar_l = 2 * (EAR_IN - BAR_GAP)
    part = cyl_x(BAR_D / 2, -bar_l / 2, bar_l / 2)
    blk = Box(*BLOCK)
    blk = blk.fillet(1.0, blk.edges())
    part += blk
    env = Box(BLOCK[0] + 2 * COLLAR + 1, BLOCK[1] - 0.5, BLOCK[2] - 0.5)
    for s in (-1, 1):
        # cono a 45°: de Ø(8+2·3) en la cara del bloque a Ø8 a 3 mm, recortado al bloque
        c = Pos(s * (BLOCK[0] / 2 + COLLAR / 2 - 0.01), 0, 0) * Rot(0, 90 * s, 0) * \
            Cone(BAR_D / 2 + COLLAR, BAR_D / 2, COLLAR)
        part += c & env
    part -= cyl_x(PIVOT_D / 2, -bar_l, bar_l)
    for s in (-1, 1):
        part -= Pos(0, s * (BLOCK[1] / 2 - BLIND_DEPTH / 2 + 0.01), 0) * Rot(90, 0, 0) * Cylinder(BLIND_D / 2, BLIND_DEPTH)
    part.label = "cardan_cruceta"
    return part
