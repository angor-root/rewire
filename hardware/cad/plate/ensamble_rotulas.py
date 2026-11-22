"""Ensamble de la propuesta 2 (rótulas) en el marco del carro, SOLO para ver
y verificar (no imprimir). Piezas del kit (servos HS-485HB, soportes, torre de
96 mm, barra 739271) salen tal cual de Car.step.

Pose: variable de entorno DSM_POSE="roll,pitch" en grados (neutro por defecto),
p. ej. DSM_POSE=15,15 para ver la esquina del rango de trabajo.
"""
import os

from build123d import Compound

import pose_rotulas as PR


def gen_step():
    roll, pitch = (float(v) for v in os.environ.get("DSM_POSE", "0,0").split(","))
    mov, _ = PR.place(roll, pitch)
    kit = PR.kit_static()
    impresas = Compound(children=PR.labeled({k: v for k, v in mov.items() if k != "plato_200x200"}),
                        label="impresas_PETG")
    kit_c = Compound(children=PR.labeled(kit), label="kit_TETRIX_Car_step")
    plato = Compound(list(mov["plato_200x200"].solids()), label="plato_200x200x4")
    kids = [impresas, kit_c] + ([] if os.environ.get("DSM_NOPLATE") else [plato])   # DSM_NOPLATE=1 -> sin plato
    return Compound(children=kids, label=f"plato_rotulas_roll{roll:+.0f}_pitch{pitch:+.0f}")
