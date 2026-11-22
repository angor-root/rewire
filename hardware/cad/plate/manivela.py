"""Manivela (x2, idéntica para servo A y B) sobre el horn TETRIX del HS-485HB.
Rev. 2026-09-30.

Marco local del servo (common/dsm_lib): z = 0 en la cara exterior del horn,
+Z sale del horn. La manivela apunta a +X local (neutro = horizontal).
- Faldón Ø22.6 que centra la pieza en el horn Ø22.2.
- 4 agujeros Ø3.8 a r = 8 mm (patrón del horn TETRIX) con alojamiento para
  la cabeza del 6-32 del kit; agujero central Ø9 para el tornillo del horn.
- Perno de rótula a r = 35 mm (kinematics.py) con ALMA M3: tornillo
  M3 x 25 avellanado por el centro de la bola y tuerca en la trampa hexagonal
  de la cara del horn (ver common/rotula.py).
- Rev. 2026-09-30: filetes R4 brazo-cubo y brazo-bola (antes 90°), cubo de la
  bola Ø16 (antes Ø14, alrededor de la trampa de tuerca), perno Ø8 con filete.
Impresión: faldón sobre la cama y perno hacia arriba (sin soportes; el
interior del faldón es un puente de 22.6 mm). PETG, 4 perímetros, 40 %.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))

from build123d import Box, Cylinder, Pos  # noqa: E402

import dfm  # noqa: E402
import kinematics as K  # noqa: E402
from dsm_lib import SV_HORN_BC_R, SV_HORN_D  # noqa: E402
from rotula import ball_stud, ball_stud_cut  # noqa: E402

T = K.CRANK_T
R = K.CRANK_R
HUB_D = 30.0
BOSS_D = 16.0
ARM_W = 10.0
SKIRT_H = 2.0
SKIRT_ID = SV_HORN_D + 0.4
HOLE_D = 3.8
HEAD_D = 6.4
HEAD_DEPTH = 3.0
CENTER_D = 9.0
FIL = 4.0


def gen_step():
    # brazo angosto (10 mm): con el servo B a 16.6 mm de la barra 739271, un brazo
    # más ancho la toca a -33° (ver README, chequeo de choques)
    arm = Pos(R / 2, 0, T / 2) * Box(R, ARM_W, T)
    part = Pos(0, 0, T / 2) * Cylinder(HUB_D / 2, T) + Pos(R, 0, T / 2) * Cylinder(BOSS_D / 2, T) + arm
    part, _, _ = dfm.fillet_concave(part, FIL)
    # faldón de centrado sobre el horn
    part += Pos(0, 0, -SKIRT_H / 2) * (Cylinder(HUB_D / 2, SKIRT_H) - Cylinder(SKIRT_ID / 2, SKIRT_H))
    # perno de rótula (con filete propio en el cuello)
    part += Pos(R, 0, T - 0.01) * ball_stud()
    # tornillos al horn
    for x, y in ((SV_HORN_BC_R, 0), (0, SV_HORN_BC_R), (-SV_HORN_BC_R, 0), (0, -SV_HORN_BC_R)):
        part -= Pos(x, y, T / 2) * Cylinder(HOLE_D / 2, T + 6)
        part -= Pos(x, y, T - HEAD_DEPTH / 2 + 0.01) * Cylinder(HEAD_D / 2, HEAD_DEPTH)
    part -= Pos(0, 0, T / 2) * Cylinder(CENTER_D / 2, T + 6)
    part -= Pos(R, 0, T) * ball_stud_cut(T)
    part.label = "manivela_r35"
    return part
