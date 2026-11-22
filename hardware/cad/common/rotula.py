"""Rótula impresa (bola + casquillo a presión) de la propuesta 2. Rev. 2026-09-30.

Cambios frente a la rev. anterior (bola Ø11 / cuello Ø6 / cavidad Ø11.2):
  - La holgura radial bola-cavidad era 0.1 mm: en FDM eso sale pegado o duro.
    Ahora 0.25 mm (cavidad Ø14.5 para bola Ø14), verificable con la probeta
    plato_rotulas/probeta_rotula (3 cavidades: 14.3 / 14.5 / 14.7).
  - El cuello Ø6 impreso de pie (capas perpendiculares al cuello) se corta
    por la línea de capa. Ahora:
      * cuello Ø8 con filete R2 contra la cara de apoyo (sin esquina a 90°);
      * ALMA DE ACERO: tornillo M3 x 25 avellanado (ISO 10642) que atraviesa
        bola + cuello + pieza base y cierra con tuerca en una trampa
        hexagonal por detrás. El plástico del cuello queda comprimido y el
        tornillo toma la flexión. La cabeza queda embutida en una cara plana
        en la punta de la bola (fuera de la zona de contacto con el casquillo).
  - Casquillo: boca Ø13.2 (0.4 mm de presión por lado) + ranura de 1.2 mm
    para que abra al calzar; pared 2.75 mm. Espesor 6.0 mm.

Cono libre ≈ ±32° (cuello contra el borde de la boca).
Montaje: calzar a presión con los dedos (si está muy duro: agua caliente
~70 °C 1 min sobre el casquillo PETG, o subir la cavidad a Ø14.7).
Alternativa comprada (misma geometría de montaje): rótula metálica M3 de
modelismo (ball link) con bola de 4.8 mm; exige rehacer la biela.
"""
import math

from build123d import Box, Cone, Cylinder, Plane, Pos, RegularPolygon, Rot, Sphere, Torus, extrude

BALL_D = 14.0
NECK_D = 8.0
NECK_FILLET = 2.0
STUD_OFFSET = 9.0            # cara de apoyo -> centro de bola
BALL_FLAT = 6.0              # la bola se corta plana a 6 mm sobre su centro (asiento de la cabeza)
CAV_D = 14.5
OPEN_D = 13.2
SOCKET_OD = 20.0
SOCKET_T = 2 * math.sqrt((CAV_D / 2) ** 2 - (OPEN_D / 2) ** 2)   # 6.0 mm
SLOT_W = 1.2
ROD_W = 10.0

# alma M3
CORE_D = 3.4
CSK_D, CSK_H = 6.8, 1.9      # ISO 10642 M3: dk 6.72, k 1.86
NUT_AF, NUT_H = 5.7, 3.0     # trampa para tuerca M3 (5.5 AF + 0.2)
CORE_SCREW = "M3 x 25 avellanado ISO 10642 + tuerca M3"


def ball_stud():
    """Perno en +Z desde z=0 (cara de apoyo). Centro de bola en z=STUD_OFFSET.
    Incluye el filete del cuello contra la cara de apoyo."""
    neck = Pos(0, 0, STUD_OFFSET / 2) * Cylinder(NECK_D / 2, STUD_OFFSET)
    ball = Pos(0, 0, STUD_OFFSET) * Sphere(BALL_D / 2)
    ball -= Pos(0, 0, STUD_OFFSET + BALL_FLAT + 5) * Box(20, 20, 10)
    rf = NECK_D / 2 + NECK_FILLET
    ring = Pos(0, 0, NECK_FILLET / 2) * Cylinder(rf, NECK_FILLET)
    ring -= Pos(0, 0, NECK_FILLET) * Torus(rf, NECK_FILLET)
    embed = Pos(0, 0, -0.3) * Cylinder(rf, 0.6)      # se hunde 0.3 mm en la pieza base (unión robusta)
    return neck + ball + ring + embed


def ball_stud_cut(base_t):
    """Agujero del alma M3 + avellanado en la punta + trampa de tuerca en la
    cara trasera de la pieza base (base_t = espesor de la pieza bajo el perno)."""
    top = STUD_OFFSET + BALL_FLAT
    bore = Pos(0, 0, (top - base_t) / 2) * Cylinder(CORE_D / 2, top + base_t + 0.2)
    csk = Pos(0, 0, top - CSK_H / 2 + 0.01) * Cone(CORE_D / 2, CSK_D / 2, CSK_H)
    nut = extrude(Plane.XY.offset(-base_t - 0.01) * RegularPolygon(NUT_AF / math.sqrt(3), 6), NUT_H + 0.01)
    return bore + csk + nut


def core_screw_len(base_t):
    return STUD_OFFSET + BALL_FLAT + base_t


def socket_blank():
    """Cuerpo del casquillo centrado en el origen, eje del perno = Z."""
    return Cylinder(SOCKET_OD / 2, SOCKET_T)


def socket_cut(slot_dir=(1, 0), cav_d=CAV_D):
    """Cavidad + ranura (la ranura sale hacia slot_dir, lejos de la biela)."""
    cav = Sphere(cav_d / 2)
    # boca: la cavidad ya abre a Ø OPEN_D en las dos caras (SOCKET_T la fija)
    ang = math.degrees(math.atan2(slot_dir[1], slot_dir[0]))
    slot = Rot(0, 0, ang) * Pos(SOCKET_OD / 4 + 1, 0, 0) * Box(SOCKET_OD / 2 + 2, SLOT_W, SOCKET_T + 2)
    return cav + slot


def cone_limit_deg():
    rim_ang = math.atan2(OPEN_D / 2, SOCKET_T / 2)
    return math.degrees(rim_ang - math.asin((NECK_D / 2) / (CAV_D / 2)))


if __name__ == "__main__":
    print(f"T casquillo = {SOCKET_T:.2f} mm, cono libre = ±{cone_limit_deg():.1f}°, "
          f"holgura radial = {(CAV_D - BALL_D) / 2:.2f} mm, presión boca = {(BALL_D - OPEN_D) / 2:.2f} mm/lado")
