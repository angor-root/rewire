"""Medidas oficiales / medidas del CAD del kit compartidas por todas las piezas DSM.

Fuentes (todas verificadas, no "a ojo"):
  - TETRIX MAX: medido en ~/Downloads/Car.step (piezas Pitsco del carro).
    Patrón: agujero grande Ø8 cada 16 mm; círculo de 4 agujeros Ø3.7 a r = 8 mm
    alrededor de cada agujero grande "de grupo" (cada 32 mm). Tornillería
    imperial 6-32 (Ø3.5) + tuerca kep, o remache plástico Quick Rivet W39277.
  - Servo TETRIX MAX 180° (Pitsco 39197 = Hitec HS-485HB): datasheet Hitec
    (39.8 x 19.8 x 38.0 mm, 45 g, 6.0 kgf·cm @6 V, 0.18 s/60°, juego 0.5°,
    estría 24T) + STEP del servo en Car.step (posiciones de orejas y horn).
  - Raspberry Pi 4 B: dibujo mecánico oficial (85 x 56, agujeros 58 x 49, M2.5)
    + STEP raspberry_pi_4_model_b de step.parts (posiciones de conectores).
"""
from build123d import Location, Plane, Vector

# ------------------------------------------------------------------ TETRIX
TX_PITCH = 16.0          # paso de la rejilla
TX_BIG_D = 8.0           # agujero grande
TX_SMALL_D = 3.7         # agujero chico (6-32 / Quick Rivet)
TX_BC_R = 8.0            # radio del círculo de 4 agujeros chicos
SCREW_632_CLR = 3.8      # pasante 6-32 en pieza impresa
SCREW_632_HEAD_D = 5.6   # cabeza SHCS 6-32 (7/64" hex): Ø 0.226" = 5.74 -> alojamiento 6.2
KIT_PLATE_T = 2.5        # espesor de pared de canal/placa TETRIX: la oreja impresa usa
                         # el mismo espesor donde va el Quick Rivet (agarre del kit)
M3_CLR = 3.4
M3_TAP = 2.6             # autorroscante M3 en PLA/PETG
M3_NUT_AF = 5.5
M25_TAP = 2.2            # autorroscante M2.5 (RPi)
M25_CLR = 2.8

# ------------------------------------------------------------ HS-485HB
# Marco local del servo: origen = centro de la cara EXTERIOR del horn TETRIX,
# +Z = eje de salida (sale del horn), +X = hacia el lado largo del cuerpo,
# +Y = ancho. Valores medidos en el STEP del servo del carro.
SV_HORN_D = 22.2
SV_HORN_T = 6.2
SV_HORN_BC_R = 8.0       # 4 agujeros Ø3.5 a 0/90/180/270° -> patrón TETRIX
SV_HORN_HOLE_D = 3.5
SV_BODY_TOP_Z = -9.04    # cara superior del cuerpo (por donde sale el eje)
SV_BODY_BOT_Z = -46.44
SV_BODY_X = (-10.03, 29.97)   # 40.0 largo; eje a 10.03 del extremo cercano
SV_BODY_W = 20.0
SV_EAR_Z = (-19.84, -17.34)   # orejas 2.5 mm
SV_EAR_X = (-20.0, 36.6)
SV_EAR_HOLES_X = (-13.8, 33.7)  # separación 47.5
SV_EAR_HOLES_Y = (-5.0, 5.0)
SV_EAR_HOLE_D = 4.3             # (goma) -> tornillo M3 / 6-32
SV_MASS_G = 45.0
SV_TORQUE_NM_6V = 0.588
SV_SPEED_S60_6V = 0.18

# Pose de los servos en el carro (Car.step): Plane(origin, x_dir, z_dir) del marco local
SERVO_A_PLANE = Plane(origin=(11.1, 31.8, 33.6), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
SERVO_B_PLANE = Plane(origin=(78.95, -35.6, 35.6), x_dir=(0, 1, 0), z_dir=(-1, 0, 0))

# ------------------------------------------------------------ Carro / plato
PIVOT_XY = (13.6, -34.2)       # eje de la torre 96 mm (y del agujero central de la barra 739271)
TOWER_TOP_Z = 115.0            # extremo superior del canal de 96 mm
TOWER_OUT = 32.1               # canal TETRIX (exterior)
TOWER_WALL = 2.5
TOWER_X = (-2.5, 29.6)
TOWER_Y = (-50.2, -18.1)       # lado abierto de la C hacia +Y
TOWER_HOLES_Z = (27.0, 43.0, 59.0, 75.0, 91.0, 107.0)  # Ø3.7 en la línea media de cada cara
FLATBAR_TOP_Z = 19.0           # barra plana 739271 (bajo la torre)
PLATE_SIZE = 200.0
PLATE_T = 4.0
PLATE_BOTTOM_Z = 131.0         # cara inferior del plato (CAD actual y sim)


def servo_loc(plane: Plane, angle_deg: float = 0.0) -> Location:
    """Location que lleva geometría del marco local del servo al mundo.
    angle_deg gira las piezas montadas en el horn alrededor de +Z local."""
    from build123d import Rot
    return Location(plane) * Rot(0, 0, angle_deg)


def load_servo_local():
    """STEP real del servo (Car.step) llevado a su marco local."""
    from pathlib import Path
    from build123d import import_step
    here = Path(__file__).resolve().parent.parent / "ref"
    s = import_step(str(here / "servo_A_hs485hb.step"))
    return s.moved(Location(SERVO_A_PLANE).inverse())
