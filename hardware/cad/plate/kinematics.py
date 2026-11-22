"""Cinemática del plato con rótulas (propuesta 2): 2 cadenas R-S-S
(servo -> manivela -> biela con rótula en ambos extremos -> plato) y un
cardán central (U) que fija el centro de giro y bloquea el yaw.

Marco: el mismo del CAD del carro (Car.step, mm). Todo lo que se usa aquí
lo importan los generadores CAD (params compartidos) para que el modelo y
la cinemática no se separen.

Uso:  python kinematics.py        -> tabla de verificación + figura
      from kinematics import ik   -> (phi_A, phi_B) [rad] para (roll, pitch)
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))
import rotula  # noqa: E402  (geometría real de la rótula impresa)

# ---- Datos del carro (medidos en Car.step) ----
PIVOT = np.array([13.6, -34.2, 119.5])      # centro del cardán (la sim usaba 115 con el domo: ver README)
PLATE_BOTTOM_Z = 132.0                       # cara inferior del plato 200x200x4 (+1 mm vs CAD actual)
HORN_A = np.array([11.1, 31.8, 33.6])        # cara exterior del horn del servo A, eje saliente -Y
HORN_B = np.array([78.95, -35.6, 35.6])      # cara exterior del horn del servo B (78.96 en el STEP), eje saliente -X

# ---- Parámetros de diseño ----
CRANK_R = 35.0          # radio de la manivela (eje del servo -> centro de la bola)
CRANK_T = 6.0           # espesor de la manivela
STUD_OFFSET = rotula.STUD_OFFSET   # cara de la manivela -> centro de la bola (cuello + R)
ROD_L = 85.0            # biela, centro a centro de las bolas
SIGN_A = -1             # manivela A apunta a -X en neutro (separa las rótulas del plato)
SIGN_B = +1             # manivela B apunta a +Y en neutro
PHI0_A = 0.0            # ángulo de la manivela con el plato horizontal (grados, + = bola hacia arriba)
PHI0_B = 0.0            # el eje del servo B está a solo 16.6 mm de la barra 739271: ver README

# centros de bola en neutro (manivela horizontal, biela vertical)
Y_A = HORN_A[1] - CRANK_T - STUD_OFFSET                      # plano de la cadena A
X_B = HORN_B[0] - CRANK_T - STUD_OFFSET                      # plano de la cadena B
_ca, _sa = math.cos(math.radians(PHI0_A)), math.sin(math.radians(PHI0_A))
_cb, _sb = math.cos(math.radians(PHI0_B)), math.sin(math.radians(PHI0_B))
C_A0 = np.array([HORN_A[0] + SIGN_A * CRANK_R * _ca, Y_A, HORN_A[2] + CRANK_R * _sa])
C_B0 = np.array([X_B, HORN_B[1] + SIGN_B * CRANK_R * _cb, HORN_B[2] + CRANK_R * _sb])
# INSET: la bola del plato se corre hacia el centro (la biela sale del plano de la
# manivela; la rótula lo admite). Menos palanca -> menos giro de servo.
INSET_A = 12.0
INSET_B = 12.0
Q_A0 = C_A0 + [0, -INSET_A, math.sqrt(ROD_L**2 - INSET_A**2)]   # bola del plato A (plato horizontal)
Q_B0 = C_B0 + [-INSET_B, 0, math.sqrt(ROD_L**2 - INSET_B**2)]
q_A = Q_A0 - PIVOT                     # en el marco del plato
q_B = Q_B0 - PIVOT

TILT_WORK = math.radians(15)   # rango de trabajo pedido (roll y pitch simultáneos)
TILT_HARD = math.radians(20)   # tope mecánico del cardán
CONE_LIMIT = rotula.cone_limit_deg()


def rot(roll, pitch):
    """R = Ry(pitch) @ Rx(roll): cardán con cruceta; eje X fijo al chasis (roll),
    eje Y en el plato (pitch)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Rx @ Ry


def _crank_angle(Q, horn, axis, plane_coord, r, L, sign, u0):
    """Resuelve phi tal que |C(phi)-Q| = L. C gira en el plano normal a 'axis'.
    u0 = dirección neutra de la manivela; v = z. Devuelve phi y C."""
    v = np.array([0, 0, 1.0])
    O = horn.copy()
    O[axis] = plane_coord
    d = Q - O
    a, b = d @ u0, d @ v
    c = (d @ d + r * r - L * L) / (2 * r)
    rho = math.hypot(a, b)
    if abs(c) > rho:
        return None, None
    base = math.atan2(b, a)
    phi = base - math.acos(c / rho)        # rama con la biela arriba
    cands = [base - math.acos(c / rho), base + math.acos(c / rho)]
    phi = min(cands, key=abs)
    C = O + r * (math.cos(phi) * u0 + math.sin(phi) * v)
    return phi, C


def ik(roll, pitch):
    R = rot(roll, pitch)
    QA = PIVOT + R @ q_A
    QB = PIVOT + R @ q_B
    pa, CA = _crank_angle(QA, HORN_A, 1, Y_A, CRANK_R, ROD_L, SIGN_A, np.array([SIGN_A, 0, 0.0]))
    pb, CB = _crank_angle(QB, HORN_B, 0, X_B, CRANK_R, ROD_L, SIGN_B, np.array([0, SIGN_B, 0.0]))
    return pa, pb, (QA, QB, CA, CB, R)


def joint_report(roll, pitch):
    pa, pb, (QA, QB, CA, CB, R) = ik(roll, pitch)
    if pa is None or pb is None:
        return None
    out = {"phiA": math.degrees(pa), "phiB": math.degrees(pb)}
    for tag, Q, C, stud_crank, stud_plate, phi, u0 in (
        ("A", QA, CA, np.array([0, -1.0, 0]), R @ np.array([0, 1.0, 0]), pa, np.array([SIGN_A, 0, 0.0])),
        ("B", QB, CB, np.array([-1.0, 0, 0]), R @ np.array([1.0, 0, 0]), pb, np.array([0, SIGN_B, 0.0])),
    ):
        rod = (Q - C) / np.linalg.norm(Q - C)
        # cono de la rótula = desviación de la biela respecto al plano normal al perno
        out["cone_crank_" + tag] = abs(math.degrees(math.asin(abs(rod @ stud_crank))))
        out["cone_plate_" + tag] = abs(math.degrees(math.asin(abs(rod @ stud_plate))))
        arm = math.cos(phi) * u0 + math.sin(phi) * np.array([0, 0, 1.0])
        # ángulo de transmisión: 90° = ideal (biela perpendicular a la manivela)
        out["trans_" + tag] = math.degrees(math.acos(abs(rod @ arm)))
    return out


def sweep(limit, n=41):
    rows = []
    for r in np.linspace(-limit, limit, n):
        for p in np.linspace(-limit, limit, n):
            rep = joint_report(r, p)
            rows.append((r, p, rep))
    return rows


def summary(limit):
    rows = sweep(limit)
    bad = [r for r in rows if r[2] is None]
    reps = [r[2] for r in rows if r[2] is not None]
    key = lambda k: (min(x[k] for x in reps), max(x[k] for x in reps))
    s = {k: key(k) for k in reps[0]}
    return s, len(bad)


def export_table(path="ik_tabla.csv", lim_deg=15, step=1.0):
    """Tabla (roll, pitch) -> ángulo de cada servo respecto de su neutro [°].
    Para gimbal_fw: interpolar bilinealmente, o ajustar un polinomio de 2º grado."""
    import csv
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["roll_deg", "pitch_deg", "servoA_deg", "servoB_deg"])
        for r in np.arange(-lim_deg, lim_deg + 1e-9, step):
            for p in np.arange(-lim_deg, lim_deg + 1e-9, step):
                pa, pb, _ = ik(math.radians(r), math.radians(p))
                w.writerow([f"{r:.1f}", f"{p:.1f}", f"{math.degrees(pa):.3f}", f"{math.degrees(pb):.3f}"])
    return path


if __name__ == "__main__":
    if "--tabla" in sys.argv:
        print("escrita", export_table())
    print(f"Q_A (plato) = {q_A.round(2)}  Q_B = {q_B.round(2)}")
    print(f"brazo efectivo A = {math.hypot(q_A[1], q_A[2]):.1f} mm, B = {math.hypot(q_B[0], q_B[2]):.1f} mm")
    print(f"límite de cono de la rótula impresa = ±{CONE_LIMIT:.1f}°")
    for name, lim in (("trabajo ±15°", TILT_WORK), ("tope ±20°", TILT_HARD)):
        s, nbad = summary(lim)
        print(f"\n== {name}: poses sin solución = {nbad}")
        for k, (lo, hi) in s.items():
            print(f"  {k:14s} {lo:7.1f} .. {hi:7.1f}")
    # sensibilidad en neutro (resolución): d(tilt)/d(phi)
    e = 1e-4
    J = np.array([[(ik(e, 0)[0] - ik(-e, 0)[0]) / (2 * e), (ik(0, e)[0] - ik(0, -e)[0]) / (2 * e)],
                  [(ik(e, 0)[1] - ik(-e, 0)[1]) / (2 * e), (ik(0, e)[1] - ik(0, -e)[1]) / (2 * e)]])
    print("\nJacobiano d(phi)/d(roll,pitch) en neutro:\n", J.round(3))
    print("banda muerta HS-485HB 3 us ~ 0.3° servo ->", round(0.3 / abs(J[0, 0]), 3), "° de plato (roll)")
