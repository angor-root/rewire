"""Reglas de diseño para impresión FDM (PETG, boquilla 0.4, capa 0.2) usadas por
todas las piezas DSM, y utilidades para cumplirlas en el CAD.

Reglas (revisión 2026-09-30, observaciones de Caleb):
  1. Sin esquinas internas de 90°: toda arista cóncava lleva filete
     (`fillet_concave`). Concentran tensión y ahí se rompe la pieza.
  2. Donde la carga cambia de dirección (pared -> ala, brazo -> cubo) va una
     cartela (`gusset_*`) o un disco/cubo redondo, no un escalón.
  3. Pared mínima 2.4 mm (6 líneas de 0.4) en piezas estructurales, 2.0 mm en
     cajas; nada más fino que 1.6 mm en ninguna pieza (se despega o se rompe).
  4. Holguras de impresión (medidas en PETG; ajustar con la probeta de
     plato_rotulas/probeta_rotula):
       - agujero de giro para eje M3 (pasador que gira):    Ø3.5
       - agujero pasante de tornillo M3 fijo:              Ø3.4
       - piloto para autorroscante M3 en PETG:             Ø2.7
       - rendija entre caras que giran una contra otra:    0.5 mm (+ arandela)
       - esfera en casquillo (radial):                     0.25 mm
  5. Voladizos <= 45° respecto de la vertical en la orientación de impresión
     indicada en cada pieza; puentes <= 25 mm.
"""
import math

from build123d import (Axis, Box, Cylinder, Location, Plane, Polygon, Pos, Rot, Solid, Vector,
                       extrude)

# ---------------------------------------------------------------- holguras
HOLE_PIVOT_M3 = 3.5
HOLE_CLR_M3 = 3.4
PILOT_M3 = 2.7
GAP_ROT = 0.5
MIN_WALL = 2.4


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, x1 - x0)


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, y1 - y0)


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, z1 - z0)


# ------------------------------------------------------------ cartelas
def gusset(plane: Plane, a: float, b: float, t: float):
    """Cartela triangular: catetos a (sobre x_dir del plano) y b (sobre y_dir),
    espesor t centrado en el plano (a lo largo de su normal)."""
    tri = Polygon((0, 0), (a, 0), (0, b), align=None)
    sk = plane * tri
    return extrude(sk, amount=t / 2, both=True)


def _prism(pl, pts, a0, a1, axis):
    """Extruye el polígono pts (coordenadas locales del plano pl) entre a0 y a1
    sobre el eje global `axis` (0=X, 1=Y, 2=Z), sin importar el sentido del polígono."""
    face = pl * Polygon(*pts, align=None)
    h = a1 - a0
    s = extrude(face, amount=h)
    bb = s.bounding_box()
    lo = (bb.min.X, bb.min.Y, bb.min.Z)[axis]
    d = [0.0, 0.0, 0.0]
    d[axis] = a0 - lo
    return s.moved(Location(tuple(d)))


def prism_xz(pts, y0, y1):
    """Extruye un polígono dado en coordenadas (x, z) desde y0 hasta y1."""
    return _prism(Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)), pts, y0, y1, 1)


def prism_yz(pts, x0, x1):
    """Extruye un polígono dado en coordenadas (y, z) desde x0 hasta x1."""
    return _prism(Plane(origin=(0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)), pts, x0, x1, 0)


def prism_xy(pts, z0, z1):
    return _prism(Plane.XY, pts, z0, z1, 2)


# ------------------------------------------------------- filetes cóncavos
def _is_concave(shape, edge, faces, eps=0.05):
    """Arista cóncava: el punto que entra en ambas caras (bisectriz de las
    tangentes interiores) queda FUERA del sólido."""
    try:
        p = edge.position_at(0.5)
        t = edge.tangent_at(0.5)
        acc = Vector(0, 0, 0)
        for f in faces:
            n = f.normal_at(p)
            d = n.cross(t)
            if d.length < 1e-9:
                return False
            d = d.normalized()
            q = p + d * eps
            if f.distance_to(q) > eps * 0.3:
                d = -d
            acc += d
        if acc.length < 1e-6:
            return False
        q = p + acc.normalized() * eps
        return not shape.is_inside(q)
    except Exception:
        return False


def _edge_face_map(solid):
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
    from build123d import Edge, Face
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(solid.wrapped, TopAbs_EDGE, TopAbs_FACE, m)
    out = []
    for i in range(1, m.Extent() + 1):
        e = Edge(m.FindKey(i))
        lst = m.FindFromIndex(i)
        fs, seen = [], []
        for f in lst:
            if not any(f.IsSame(g) for g in seen):
                seen.append(f)
                fs.append(Face(f))
        out.append((e, fs))
    return out


def concave_edges(shape, min_len=0.3):
    solid = shape.solids()[0] if len(shape.solids()) == 1 else shape
    out = []
    for e, fs in _edge_face_map(solid):
        if e.length < min_len or len(fs) != 2:
            continue
        if _is_concave(solid, e, fs):
            out.append(e)
    return out


def fillet_concave(part, r, min_len=0.3, keep=None, radii=None):
    """Filetea todas las aristas cóncavas con radio r (o el mayor que OCC acepte,
    bajando hasta r/4 arista por arista). keep(edge) -> False excluye la arista.
    Devuelve (pieza, n_fileteadas, n_total)."""
    label = getattr(part, "label", None)
    try:
        part = part.clean()
    except Exception:
        pass
    edges = [e for e in concave_edges(part, min_len) if keep is None or keep(e)]
    n_tot = len(edges)
    for rr in (radii or (r, r * 0.66, r * 0.5, r * 0.33)):
        try:
            out = part.fillet(rr, edges)
            if out.is_valid and len(out.solids()) == 1:
                out.label = label
                return out, n_tot, n_tot
        except Exception:
            pass
    # uno por uno (re-detectando, porque la topología cambia)
    done = 0
    for _ in range(n_tot):
        cands = [e for e in concave_edges(part, min_len) if keep is None or keep(e)]
        if not cands:
            break
        ok = False
        for e in sorted(cands, key=lambda e: -e.length):
            for rr in (radii or (r, r * 0.66, r * 0.4, r * 0.25)):
                try:
                    out = part.fillet(rr, [e])
                    if out.is_valid and len(out.solids()) == 1:
                        part, ok = out, True
                        done += 1
                        break
                except Exception:
                    continue
            if ok:
                break
        if not ok:
            break
    part.label = label
    return part, done, n_tot


def round_convex(part, r, axis=None, min_len=2.0):
    """Redondea aristas convexas rectas paralelas a `axis` (esquinas exteriores
    de perfiles). Si falla, deja la pieza igual."""
    label = getattr(part, "label", None)
    sel = []
    for e, fs in _edge_face_map(part):
        if e.geom_type.name != "LINE" or e.length < min_len:
            continue
        if axis is not None:
            t = e.tangent_at(0.5)
            if abs(abs(t.dot(Vector(*axis))) - 1) > 1e-6:
                continue
        if len(fs) == 2 and not _is_concave(part, e, fs):
            sel.append(e)
    for rr in (r, r * 0.6, r * 0.4):
        try:
            out = part.fillet(rr, sel)
            if out.is_valid and len(out.solids()) == 1:
                out.label = label
                return out
        except Exception:
            pass
    return part


# ---------------------------------------------------- chequeo de voladizos
def overhang_report(part, print_rot: Location, max_deg=45.0, bed_tol=0.3, min_area=1.0):
    """Área (mm²) de caras que miran hacia abajo más de max_deg respecto de la
    vertical en la orientación de impresión (print_rot aplicado a la pieza),
    excluyendo la cara sobre la cama. Devuelve (area_total, lista (area, z, normal))."""
    p = part.moved(print_rot)
    zmin = p.bounding_box().min.Z
    lim = math.cos(math.radians(90 - max_deg))     # |n_z| mayor que esto = voladizo
    tot, items = 0.0, []
    for f in p.faces():
        a = f.area
        if a < 1e-6:
            continue
        # muestreo de la normal en varias posiciones de la cara
        bad = 0
        pts = [(u, v) for u in (0.2, 0.5, 0.8) for v in (0.2, 0.5, 0.8)]
        for u, v in pts:
            try:
                n = f.normal_at(u, v)
                pos = f.position_at(u, v)
            except Exception:
                continue
            if n.Z < -lim and pos.Z > zmin + bed_tol:
                bad += 1
        if bad:
            fa = a * bad / len(pts)
            if fa >= min_area:
                tot += fa
                items.append((fa, f.center().Z - zmin, tuple(round(c, 2) for c in f.normal_at().to_tuple())))
    items.sort(key=lambda t: -t[0])
    return tot, items


# ------------------------------------------------ envolvente círculo + base
def tangent_point(p, c, r, side):
    """Punto de tangencia desde p al círculo (c, r). side=+1 / -1 elige la
    tangente a la izquierda / derecha mirando de p hacia c."""
    dx, dy = c[0] - p[0], c[1] - p[1]
    d = math.hypot(dx, dy)
    th = math.atan2(p[1] - c[1], p[0] - c[0])
    a = math.acos(r / d)
    ang = th + side * a
    return (c[0] + r * math.cos(ang), c[1] + r * math.sin(ang))


def lug_profile(u0, u1, v_base, v_top_base, c, r):
    """Perfil 2D (u, v) de una oreja: rectángulo [u0,u1]x[v_base,v_top_base]
    que se afina con tangentes rectas hasta un círculo (c, r). Devuelve los
    puntos del polígono; unir con el círculo (cilindro) para cerrar el arco."""
    tl = tangent_point((u0, v_top_base), c, r, -1)
    tr = tangent_point((u1, v_top_base), c, r, +1)
    return [(u0, v_base), (u1, v_base), (u1, v_top_base), tr, tl, (u0, v_top_base)]
