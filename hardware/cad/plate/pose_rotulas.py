"""Coloca todas las piezas de la propuesta de rótulas para una pose (roll, pitch)
usando la cinemática inversa de kinematics.py. Lo usan el ensamble y el
chequeo de choques, así el CAD y la cinemática no se separan."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))

import numpy as np  # noqa: E402
from build123d import Box, Compound, Location, Plane, Pos, Rot, import_step  # noqa: E402

import arana_plato  # noqa: E402
import biela  # noqa: E402
import cardan_lib  # noqa: E402
import kinematics as K  # noqa: E402
import manivela  # noqa: E402
from dsm_lib import PLATE_SIZE, PLATE_T, SERVO_A_PLANE, SERVO_B_PLANE, servo_loc  # noqa: E402

REF = Path(__file__).resolve().parent.parent / "ref"
P = Location(tuple(K.PIVOT))

_cache = {}


def parts():
    if not _cache:
        _cache.update(
            base=cardan_lib.gen_base(), cruz=cardan_lib.gen_cross(), arana=arana_plato.gen_step(),
            manivela=manivela.gen_step(), biela=biela.gen_step(),
            plato=Pos(0, 0, arana_plato.PLATE_OFF + PLATE_T / 2) * Box(PLATE_SIZE, PLATE_SIZE, PLATE_T),
        )
    return _cache


def kit_static():
    """Piezas del kit que no se mueven (del Car.step). La torre y la barra se mantienen."""
    out = {}
    for n in ("servo_A_hs485hb", "servo_B_hs485hb", "servo_brackets", "torre_96mm", "barra_plana_739271"):
        out[n] = import_step(str(REF / f"{n}.step"))
    return out


def _rod_loc(C, Q, stud_axis):
    x = (Q - C) / np.linalg.norm(Q - C)
    z = stud_axis - (stud_axis @ x) * x
    z /= np.linalg.norm(z)
    return Location(Plane(origin=tuple(C), x_dir=tuple(x), z_dir=tuple(z)))


def place(roll_deg=0.0, pitch_deg=0.0):
    """Devuelve dict nombre -> shape en mundo, y el reporte cinemático."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    pa, pb, (QA, QB, CA, CB, R) = K.ik(r, p)
    if pa is None or pb is None:
        raise ValueError(f"pose sin solución: roll={roll_deg} pitch={pitch_deg}")
    pp = parts()
    # ángulo de montaje sobre el horn (marco local del servo). A: X_l = +X, Y_l = +Z mundo;
    # B: X_l = +Y, Y_l = -Z mundo. La manivela neutra apunta a SIGN * u0.
    aA = math.degrees(math.atan2(math.sin(pa), K.SIGN_A * math.cos(pa)))
    aB = math.degrees(math.atan2(-math.sin(pb), K.SIGN_B * math.cos(pb)))
    Rx = Rot(roll_deg, 0, 0)
    Rxy = Rot(roll_deg, 0, 0) * Rot(0, pitch_deg, 0)      # R = Rx(roll) @ Ry(pitch)
    out = {
        "cardan_base": pp["base"].moved(P),
        "cardan_cruceta": pp["cruz"].moved(P * Rx),
        "arana_plato": pp["arana"].moved(P * Rxy),
        "plato_200x200": pp["plato"].moved(P * Rxy),
        "manivela_A": pp["manivela"].moved(servo_loc(SERVO_A_PLANE, aA)),
        "manivela_B": pp["manivela"].moved(servo_loc(SERVO_B_PLANE, aB)),
        "biela_A": pp["biela"].moved(_rod_loc(CA, QA, np.array([0, 1.0, 0]))),
        "biela_B": pp["biela"].moved(_rod_loc(CB, QB, np.array([1.0, 0, 0]))),
    }
    # control: la bola de la manivela colocada por el CAD == C de la cinemática
    for tag, plane, ang, C in (("A", SERVO_A_PLANE, aA, CA), ("B", SERVO_B_PLANE, aB, CB)):
        ball = servo_loc(plane, ang) * Location((K.CRANK_R, 0, K.CRANK_T + K.STUD_OFFSET))
        err = np.linalg.norm(np.array(tuple(ball.position)) - C)
        assert err < 1e-6, f"manivela {tag}: bola a {err:.4f} mm de la cinemática"
    for tag, Q, loc in (("A", QA, P * Rxy * Location(tuple(K.q_A))), ("B", QB, P * Rxy * Location(tuple(K.q_B)))):
        err = np.linalg.norm(np.array(tuple(loc.position)) - Q)
        assert err < 1e-6, f"araña {tag}: bola a {err:.4f} mm de la cinemática"
    return out, (math.degrees(pa), math.degrees(pb))


def labeled(d):
    return [Compound(list(v.solids()), label=k) for k, v in d.items()]
