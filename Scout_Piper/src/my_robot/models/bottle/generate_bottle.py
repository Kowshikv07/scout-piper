#!/usr/bin/env python3
"""Generates meshes for a ~500 ml lemon-lime style PET bottle (units: metres, Z up, origin at base)."""
import math, os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")
N = 60  # angular segments

def smooth(t): return t * t * (3 - 2 * t)

def body_profile():
    """(z, r) outer PET profile from bottom centre up to the mouth."""
    p = [(0.0080, 0.000), (0.0065, 0.010), (0.0050, 0.018), (0.0015, 0.025),
         (0.0000, 0.029), (0.0035, 0.0315), (0.0120, 0.0322), (0.0250, 0.0320)]
    # straight body with two grip grooves and label panel
    for i in range(0, 31):
        z = 0.030 + i * (0.090 - 0.030) / 30
        r = 0.0320
        for gz in (0.040, 0.085):                 # shallow ring grooves
            r -= 0.0012 * math.exp(-((z - gz) / 0.0025) ** 2)
        p.append((z, r))
    p.append((0.0985, 0.0320))
    # rounded shoulder 0.100 -> 0.160
    for i in range(1, 25):
        t = i / 24
        p.append((0.100 + 0.060 * t, 0.0320 - (0.0320 - 0.0142) * smooth(t) ** 0.9))
    # neck finish
    p += [(0.1700, 0.0140), (0.1704, 0.0176), (0.1736, 0.0176), (0.1740, 0.0140),
          (0.1900, 0.0140), (0.1900, 0.0120)]
    return p

def petal(theta, z):
    """Radial modulation making 5 feet at the base."""
    if z > 0.030: return 1.0
    k = (0.030 - z) / 0.030
    return 1.0 - 0.10 * k * (0.5 - 0.5 * math.cos(5 * theta))

def revolve(profile, mod=None, uv=None, flip=False, close_bottom=False):
    V, VN, VT, F = [], [], [], []
    rows = len(profile)
    for j, (z, r) in enumerate(profile):
        for i in range(N + 1):
            th = 2 * math.pi * i / N
            rr = r * (mod(th, z) if mod else 1.0)
            V.append((rr * math.cos(th), rr * math.sin(th), z))
            VT.append((i / N, j / (rows - 1)))
    W = N + 1
    for j in range(rows - 1):
        for i in range(N):
            a, b, c, d = j * W + i, j * W + i + 1, (j + 1) * W + i + 1, (j + 1) * W + i
            F.append((a, b, c) if not flip else (a, c, b))
            F.append((a, c, d) if not flip else (a, d, c))
    # smooth normals (area-weighted accumulation)
    acc = [[0.0, 0.0, 0.0] for _ in V]
    for a, b, c in F:
        ax, ay, az = V[a]; bx, by, bz = V[b]; cx, cy, cz = V[c]
        u = (bx - ax, by - ay, bz - az); v = (cx - ax, cy - ay, cz - az)
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        for k in (a, b, c):
            for m in range(3): acc[k][m] += n[m]
    for n in acc:
        l = math.sqrt(sum(x*x for x in n)) or 1.0
        VN.append((n[0]/l, n[1]/l, n[2]/l))
    return V, VN, VT, F

def write_obj(name, parts):
    V, VN, VT, F = [], [], [], []
    for pv, pn, pt, pf in parts:
        o = len(V)
        V += pv; VN += pn; VT += pt
        F += [(a + o, b + o, c + o) for a, b, c in pf]
    with open(os.path.join(OUT, name + ".obj"), "w") as f:
        f.write(f"# {name}\no {name}\n")
        for v in V: f.write("v %.6f %.6f %.6f\n" % v)
        for t in VT: f.write("vt %.5f %.5f\n" % t)
        for n in VN: f.write("vn %.5f %.5f %.5f\n" % n)
        for a, b, c in F:
            f.write(f"f {a+1}/{a+1}/{a+1} {b+1}/{b+1}/{b+1} {c+1}/{c+1}/{c+1}\n")
    print(name, len(V), "verts", len(F), "tris")

# ---------------- bottle body (outer + thin inner wall so it reads as a shell) -------------
outer = body_profile()
write_obj("bottle_body", [revolve(outer, mod=petal)])

# ---------------- liquid: inner profile, filled to the neck base ---------------------------
inner = [(z + (0.0006 if r > 0 else 0.0), max(r - 0.0009, 0.0)) for z, r in outer if z <= 0.1650]
inner = [(z, r) for z, r in inner if not (z < 0.008 and r > 0.0 and r < 0.001)]
inner.append((0.1650, 0.0132))
inner.append((0.1650, 0.0))
write_obj("bottle_liquid", [revolve(inner, mod=petal, flip=False)])

# ---------------- cap: knurled cylinder with domed top and tamper band ---------------------
cap = [(0.1738, 0.0150), (0.1738, 0.0164), (0.1780, 0.0164), (0.1782, 0.0162), (0.1790, 0.0162)]
for i in range(0, 13):
    cap.append((0.1790 + i * (0.1965 - 0.1790) / 12, 0.0163))
cap += [(0.1972, 0.0158), (0.1978, 0.0145), (0.1980, 0.0100), (0.1980, 0.0)]
def knurl(th, z):
    return 1.0 + (0.018 * math.cos(48 * th) if z > 0.1790 and z < 0.1970 else 0.0)
write_obj("bottle_cap", [revolve(cap, mod=knurl)])

# ---------------- label: open double-sided sleeve with UVs -----------------------------------
z0, z1 = 0.0475, 0.0800
lab = [(z0, 0.03225), (z1, 0.03225)]
write_obj("bottle_label", [revolve(lab), revolve(lab, flip=True)])
