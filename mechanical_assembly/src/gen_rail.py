#!/usr/bin/env python3
"""Generate the robot linear-rail base as STL (+ optional STEP).

Exported frame (STL/STEP): X along the rails = direction of travel (X=0 is the
"start" end without end caps), Y across, Z up, Z=0 is the floor, origin on the
centre line between the rails. Robot base X is aligned with the rail X.
Internally the model is built with travel along +Y and rotated by -90 deg about Z
on export. All dimensions in mm.

Telix ALP parts are read directly from the Telix STEP catalog zip; linear guide
rail, carriages and stoppers are modelled parametrically from datasheet values.

Usage (needs `pip install cadquery`), see ../README.md:
    python gen_rail.py                                  # -> ../robot_rail_v3.stl
    python gen_rail.py --step ../robot_rail_v3.step     # + assembly STEP (instanced parts)
BOM: python gen_bom.py ../robot_rail_v3_BOM.xlsx --readme ../README.md   (data in bom_data.py)
"""
import argparse
import glob
import math
import os
import tempfile
import zipfile

import cadquery as cq
from OCP.gp import gp_Trsf

# ----------------------------------------------------------------------------
# Design parameters
# ----------------------------------------------------------------------------
L = 3000.0            # length of the 45x90 profiles and of the linear rails
GAP = 500.0           # clear gap between the 45x90 profiles = spacer length
N_SPACERS = 4         # 45x45 cross spacers, first flush with Y=0, last with Y=L
N_LEGS_PER_SIDE = 4   # 45x45 vertical legs under each 45x90 profile, equally spaced
LEG_LEN = 100.0
# legs sit at the spacer positions; the end legs (castors) get their 2nd bracket on the
# underside of the end spacer instead of on the profile beyond the end
P90_W, P90_H = 45.0, 90.0
P45 = 45.0

# Linear guide size 35: Saier GD35 rail (= HIWIN HGR35R section), SER-GD catalogue
RAIL = dict(W=34.0, H=29.0, P=80.0, E=20.0, D=14.0, h=12.0, d=9.0)
CARR = dict(W=100.0, L=110.0, L1=80.0, H=48.0, H1=10.0, T=14.0, B=82.0, C=62.0,
            M=10.0, body_w=70.0)            # Saier GD35WA (= HIWIN HGW35CC interface)
CARRIAGES_PER_RAIL = 2
PLATE_Y = 1500.0                 # carriage plate centre (picture position)
CARR_DY = 200.0                  # carriage centre offset from the plate centre (spacing 400)
CARRIAGE_Y = (PLATE_Y - CARR_DY, PLATE_Y + CARR_DY)
PLATE = dict(LX=700.0, LY=600.0, T=25.0)   # across x along travel x thickness (Al plate)
M10_CB = dict(d=11.0, D=17.5, h=10.5)      # counterbore for DIN 912 M10 (carriage screws)
# FAIRINO FR20 base (measured on fairino_description base_link.STL): Ø240 flange,
# 6 x Ø10.5 (M10) on PCD 210 at 30 + k*60 deg, Ø8 dowel hole + Ø8 slot (positions
# estimated from the coarse mesh - confirm on the FAIRINO drawing before machining).
ROBOT_PCD = 210.0
ROBOT_HOLE_ANG = (30.0, 90.0, 150.0, 210.0, 270.0, 330.0)
ROBOT_TAP_DRILL = 8.5                                   # M10 thread, through
ROBOT_DOWELS = ((77.7, -56.0), (-87.3, 40.8))          # robot frame (x, y), Ø8 H7 through
STOPPER = dict(W=60.0, L=20.0, H=35.0, top=10.0)  # "HG35 limit block" clamp stopper
RAIL_SCREW_L = 30.0               # DIN 912 M8 length for rail -> T-nut
HANDLE_SCREW_L = 20.0             # DIN 912 M8 for ALP4112 -> T-nut

# Feet: total height from the floor to the bottom of the 45x45 legs
FOOT_H = None  # set from the castor model at runtime

X_IN = GAP / 2                    # inner face of the right profile
X_C = GAP / 2 + P90_W / 2         # centre line of the right profile / rail


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------
def frame_move(shape, xdir, ydir, origin):
    """Rotate so that local +X -> xdir, local +Y -> ydir (global unit vectors),
    then translate local origin to `origin`."""
    zdir = (xdir[1] * ydir[2] - xdir[2] * ydir[1],
            xdir[2] * ydir[0] - xdir[0] * ydir[2],
            xdir[0] * ydir[1] - xdir[1] * ydir[0])
    t = gp_Trsf()
    t.SetValues(xdir[0], ydir[0], zdir[0], origin[0],
                xdir[1], ydir[1], zdir[1], origin[1],
                xdir[2], ydir[2], zdir[2], origin[2])
    return shape.moved(cq.Location(t))


class Catalog:
    """Lazy loader for Telix STEP files straight from the catalog zip."""

    def __init__(self, zip_path):
        self.zip = zipfile.ZipFile(zip_path)
        self.tmp = tempfile.mkdtemp(prefix="telix_")
        self.cache = {}
        self.names = {os.path.splitext(os.path.basename(n))[0].strip().upper(): n
                      for n in self.zip.namelist() if n.lower().endswith((".step", ".stp"))}

    def get(self, code):
        code = code.upper()
        if code not in self.cache:
            member = self.names[code]
            path = self.zip.extract(member, self.tmp)
            self.cache[code] = cq.importers.importStep(path).val()
        return self.cache[code]


def profile_bar(cat, code, length):
    """Extrude the end face of a 1000 mm Telix profile model to `length`.
    Result: cross-section in XY (as in the catalog), extrusion along +Z from 0."""
    src = cat.get(code)
    end = [f for f in src.Faces()
           if abs(f.Center().z) < 1e-6 and abs(abs(f.normalAt().z) - 1) < 1e-6]
    face = max(end, key=lambda f: f.Area())
    return cq.Solid.extrudeLinear(face, cq.Vector(0, 0, length))


# ----------------------------------------------------------------------------
# Parametric linear-guide parts (local frame: rail runs along +Y from Y=0,
# rail bottom at Z=0, rail centred on X=0)
# ----------------------------------------------------------------------------
def rail_model(length):
    r = RAIL
    w, h = r["W"] / 2, r["H"]
    g0, g1, gd = 0.50 * h, 0.78 * h, 2.5          # ball groove band and depth
    pts = [(-w, 0), (w, 0), (w, g0), (w - gd, g0 + gd), (w - gd, g1 - gd), (w, g1),
           (w, h - 1), (w - 1, h), (-w + 1, h), (-w, h - 1), (-w, g1),
           (-w + gd, g1 - gd), (-w + gd, g0 + gd), (-w, g0)]
    body = (cq.Workplane("XZ").polyline(pts).close().extrude(-length))
    n = int((length - 2 * r["E"]) // r["P"]) + 1
    ys = [r["E"] + i * r["P"] for i in range(n)]
    body = (body.faces(">Z").workplane(origin=(0, 0, h))
            .pushPoints([(0, y) for y in ys])
            .cboreHole(r["d"], r["D"], r["h"]))
    return body.val(), ys


def carriage_model():
    c, r = CARR, RAIL
    zb, zt = c["H1"], c["H"]                       # carriage bottom / top
    steel = (cq.Workplane("XY").box(c["body_w"], c["L1"], zt - zb, centered=(True, True, False))
             .translate((0, 0, zb))
             .union(cq.Workplane("XY").box(c["W"], c["L1"], c["T"], centered=(True, True, False))
                    .translate((0, 0, zt - c["T"]))))
    caps = (cq.Workplane("XY").box(c["body_w"] - 2, c["L"], zt - zb - 3, centered=(True, True, False))
            .translate((0, 0, zb)))
    body = steel.union(caps)
    channel = (cq.Workplane("XY").box(r["W"] + 1.0, c["L"] + 2, r["H"] + 0.8 - zb + 0.01,
                                      centered=(True, True, False)).translate((0, 0, zb - 0.01)))
    body = body.cut(channel)
    body = (body.faces(">Z").workplane()
            .rect(c["B"], c["C"], forConstruction=True).vertices()
            .hole(c["M"] * 0.85, c["T"]))
    return body.val()


def stopper_model():
    """Clamp-on rail stopper ("HG35 limit block" type, 60x20x35, 10 mm bridge):
    U-block over the rail end, M6 set screw on top, two rubber bumpers facing the
    carriage (local +Y)."""
    s, r = STOPPER, RAIL
    z0 = r["H"] + s["top"] - s["H"]                        # block bottom above rail bottom
    blk = (cq.Workplane("XY").box(s["W"], s["L"], s["H"], centered=(True, False, False))
           .translate((0, 0, z0)))
    blk = blk.cut(cq.Workplane("XY").box(r["W"] + 0.4, s["L"] + 2, r["H"] - z0 + 0.01,
                                         centered=(True, False, False)).translate((0, -1, z0 - 0.01)))
    # M6 set screw on top; it reaches into the counterbore of the rail hole below
    blk = blk.union(cq.Workplane("XY", origin=(0, s["L"] / 2, r["H"] - 3)).circle(3.0)
                    .extrude(s["top"] + 3))
    leg_x = (r["W"] / 2 + s["W"] / 2) / 2
    for sx in (-1, 1):
        blk = blk.union(cq.Workplane("XZ", origin=(sx * leg_x, s["L"], z0 + 12)).circle(5).extrude(-6))
    return blk.val()


def plate_model():
    """Carriage plate in build frame, centred on (0, PLATE_Y), bottom at Z=0.
    Robot frame on the plate: robot +X -> build +Y, robot +Y -> build -X."""
    p, c = PLATE, CARR
    pl = cq.Workplane("XY").box(p["LX"], p["LY"], p["T"], centered=(True, True, False))
    carr_pts = [(sx * X_C + bx * c["B"] / 2, dy + cy * c["C"] / 2)
                for sx in (-1, 1) for dy in (-CARR_DY, CARR_DY) for bx in (-1, 1) for cy in (-1, 1)]
    pl = (pl.faces(">Z").workplane(origin=(0, 0, p["T"])).pushPoints(carr_pts)
          .cboreHole(M10_CB["d"], M10_CB["D"], M10_CB["h"]))
    r = ROBOT_PCD / 2
    rob = [(-r * math.sin(math.radians(a)), r * math.cos(math.radians(a))) for a in ROBOT_HOLE_ANG]
    pl = pl.faces(">Z").workplane(origin=(0, 0, p["T"])).pushPoints(rob).hole(ROBOT_TAP_DRILL)
    # dowel holes through, so the pins can be knocked out from below
    dow = [(-y, x) for x, y in ROBOT_DOWELS]
    pl = pl.faces(">Z").workplane(origin=(0, 0, p["T"])).pushPoints(dow).hole(8.0)
    # engraved arrow = robot / rail +X direction (build +Y), 1 mm deep
    arrow = (cq.Workplane("XY", origin=(0, 0, p["T"] - 1.0))
             .polyline([(-6, 160), (6, 160), (6, 215), (16, 215), (0, 245), (-16, 215), (-6, 215)])
             .close().extrude(1.01))
    pl = pl.cut(arrow)
    return pl.val().moved(cq.Location(cq.Vector(0, PLATE_Y, 0))), carr_pts, dow


def socket_screw(d, length, head_d, head_h):
    """DIN 912 socket head cap screw; head bearing face at Z=0, shank along -Z."""
    head = (cq.Workplane("XY").circle(head_d / 2).circle(d * 0.4).extrude(head_h))
    shank = cq.Workplane("XY").circle(d / 2).extrude(-length)
    return head.union(shank).val()


# ----------------------------------------------------------------------------
# Telix part placement constants (measured from the catalog STEP files)
# ----------------------------------------------------------------------------
BRK_PLATE_T = 8.0                 # ALP2137 plate thickness
BRK_HOLES = (30.0, 65.75)         # ALP2137 screw positions along each leg
NUT_RIDGE_DEPTH = 1.15            # ALP0643: body (local Y 4.9) seats under the 6 mm slot lip
TAP_DEPTH_FOOT, TAP_DEPTH_CASTOR = 25.0, 30.0   # M12 thread depth in the leg core
CASTOR_FLOOR = -94.9              # ALP5527: wheel contact (local Z)
CASTOR_MOUNT = 17.2               # ALP5527: top of the lock nut (local Z)
# Rubber foot ALP2512 (axis local Y) on stud ALP2613 M12x100 (axis local Z, ball SR7.5)
FOOT_FLOOR = -12.2                # ALP2512: rubber pad bottom (local Y)
FOOT_AXIS = (0.7, 0.4)            # ALP2512: axis (local X, Z)
FOOT_BALL = 1.8                   # ALP2512: ball-socket centre (local Y), estimated
STUD_AXIS = (-4.4, -0.1)          # ALP2613: axis (local X, Y)
STUD_BALL = 56.6                  # ALP2613: ball centre (local Z); thread end at -59.4
STUD_NUT_TOP = -30.1              # ALP2613: lock nut face towards the thread end (local Z)
HANDLE_SEAT = 6.0                 # ALP4112: screw head seat above the foot bottom
HANDLE_HOLES = (-60.0, 60.0)      # ALP4112: hole positions along the handle


class Asm:
    def __init__(self):
        self.parts = []           # (bom_code, name, shape, rgb)

    def add(self, code, name, shape, rgb):
        self.parts.append((code, name, shape, rgb))


def place_local(shape, xdir, ydir, p_local, target):
    """Rotate (local X->xdir, Y->ydir) and move so local point p_local lands on target."""
    zdir = (xdir[1] * ydir[2] - xdir[2] * ydir[1],
            xdir[2] * ydir[0] - xdir[0] * ydir[2],
            xdir[0] * ydir[1] - xdir[1] * ydir[0])
    off = [target[i] - (p_local[0] * xdir[i] + p_local[1] * ydir[i] + p_local[2] * zdir[i])
           for i in range(3)]
    return frame_move(shape, xdir, ydir, off)


def neg(v):
    return tuple(-c for c in v)


def build(cat):
    a = Asm()
    AL, DARK, STEEL, BLACK, RED = (0.78, 0.80, 0.83), (0.25, 0.25, 0.27), (0.55, 0.57, 0.6), (0.1, 0.1, 0.1), (0.75, 0.15, 0.1)
    PLATE_RGB = (0.68, 0.72, 0.78)

    leg_bottom = CASTOR_MOUNT - CASTOR_FLOOR           # castor height incl. lock nut
    zp = leg_bottom + LEG_LEN                          # bottom of the 45x90 profiles
    z_top = zp + P90_H                                 # top face of the 45x90 profiles
    pitch = (L - P45) / (N_SPACERS - 1)
    spacer_y = [i * pitch for i in range(N_SPACERS)]   # spacer start positions
    leg_y = spacer_y                                   # legs under the spacers
    leg_c = [y + P45 / 2 for y in leg_y]
    handle_y = ((leg_c[0] + leg_c[1]) / 2, (leg_c[-2] + leg_c[-1]) / 2)

    p90 = profile_bar(cat, "ALP0047", L)
    p45_spacer = profile_bar(cat, "ALP0045", GAP)
    p45_leg = profile_bar(cat, "ALP0045", LEG_LEN)
    # legs with the M12 thread in the bottom of the core (one shape per variant -> STEP instances)
    leg_variant = {d: p45_leg.cut(cq.Workplane("XY").circle(6.0).extrude(d).val()
                                  .moved(cq.Location(cq.Vector(0, 0, -0.01))))
                   for d in (TAP_DEPTH_CASTOR, TAP_DEPTH_FOOT)}
    brk = cat.get("ALP2137")
    # ALP2138 is modelled mirrored (slope along x = y); 180 deg about x = 44.75 puts it on the
    # ALP2137 slope (x + y = 88) in the bracket's own frame
    brk_cover = cat.get("ALP2138").rotate((44.75, 0, 0), (44.75, 1, 0), 180)
    nut = cat.get("ALP0643")
    scr = cat.get("ALP0175")
    cap = cat.get("ALP4017")
    handle = cat.get("ALP4112")
    castor = cat.get("ALP5527")
    foot = cat.get("ALP2512")
    stud_src = cat.get("ALP2613")
    ball_z = FOOT_BALL - FOOT_FLOOR                    # ball centre above the floor
    # stud is placed ball-down; move its lock nut (solid #1) up against the leg bottom
    nut_top_now = ball_z + (STUD_BALL - STUD_NUT_TOP)
    stud = cq.Compound.makeCompound(
        [sd.translate((0, 0, -(leg_bottom - nut_top_now))) if k == 1 else sd
         for k, sd in enumerate(stud_src.Solids())])
    m8x20 = socket_screw(8, HANDLE_SCREW_L, 13.0, 8.0)

    def t_nut(face_pt, inward, along):
        """ALP0643 in a slot: face_pt on the profile face (slot centre), inward = unit
        normal into the profile, along = slot direction."""
        across = (inward[1] * along[2] - inward[2] * along[1],
                  inward[2] * along[0] - inward[0] * along[2],
                  inward[0] * along[1] - inward[1] * along[0])
        tgt = [face_pt[i] + inward[i] * NUT_RIDGE_DEPTH for i in range(3)]
        return place_local(nut, across, inward, (0, 0, 0), tgt)

    def bracket(corner, n1, n2, slot1, slot2):
        """ALP2137 + cover ALP2138 in the corner between two faces with outward normals
        n1, n2 (slot directions slot1, slot2), with 2 x (ALP0175 + ALP0643) per leg."""
        a.add("ALP2137", "ugaona_spojnica_45_visoka", place_local(brk, n1, n2, (0, 0, 0), corner), STEEL)
        a.add("ALP2138", "poklopac_spojnice_45", place_local(brk_cover, n1, n2, (0, 0, 0), corner), BLACK)
        for leg_n, leg_dir, slot in ((n1, n2, slot1), (n2, n1, slot2)):
            for d in BRK_HOLES:
                p_face = [corner[k] + leg_dir[k] * d for k in range(3)]
                head = [p_face[k] + leg_n[k] * BRK_PLATE_T for k in range(3)]
                sdir = neg(leg_n)                      # ALP0175: shank along local +X
                a.add("ALP0175", "vijak_M8x20", place_local(scr, sdir, leg_dir, (0, 0, 0), head), STEEL)
                a.add("ALP0643", "klizna_navrtka_M8_T10", t_nut(p_face, sdir, slot), STEEL)

    for side in (-1, 1):
        xs = side * X_C
        x_in = side * X_IN
        out = (side, 0, 0)                                 # outward normal of outer face
        n_in = (-side, 0, 0)                               # outward normal of inner face
        # --- 45x90 main profile (local X=90 -> Z, local Z=length -> Y)
        a.add("ALP0047", f"profil_45x90_{'L' if side < 0 else 'R'}",
              frame_move(p90, (0, 0, 1), (1, 0, 0), (xs, 0, zp + 22.5)), AL)
        # --- end cap at the far end only (Y = L)
        a.add("ALP4017", "poklopac_45x90",
              place_local(cap, (0, 0, 1), (-1, 0, 0), (0, 0, 4.3), (xs, L, zp + 45)), DARK)
        # --- handles on the outer face, upper slot, between legs 1-2 and 3-4
        for yh in handle_y:
            face_pt = (xs + side * P90_W / 2, yh, zp + 67.5)
            ydir = (0, 0, side)                             # local Z -> outward
            a.add("ALP4112", "rucka_120", place_local(handle, (0, 1, 0), ydir, (0, 0, 0), face_pt), DARK)
            for hx in HANDLE_HOLES:
                fp = (face_pt[0], yh + hx, face_pt[2])
                a.add("DIN912-M8x20", "vijak_DIN912_M8x20", place_local(
                    m8x20, (0, 1, 0), ydir, (0, 0, 0), (fp[0] + side * HANDLE_SEAT, fp[1], fp[2])), DARK)
                a.add("ALP0643", "klizna_navrtka_M8_T10", t_nut(fp, neg(out), (0, 1, 0)), STEEL)

        # --- legs: 2 x ALP2137 each, foot / castor at the bottom
        for i, y0 in enumerate(leg_y):
            yc = y0 + P45 / 2
            end_leg = i in (0, N_LEGS_PER_SIDE - 1)
            tap = TAP_DEPTH_CASTOR if end_leg else TAP_DEPTH_FOOT
            a.add("ALP0045", "noga_45x45x100",
                  frame_move(leg_variant[tap], (1, 0, 0), (0, 1, 0), (xs, yc, leg_bottom)), AL)
            if end_leg:
                # one bracket towards the middle on the profile underside, one on the inner
                # face to the underside of the end spacer (the leg sits under it)
                ny = (0, 1, 0) if i == 0 else (0, -1, 0)
                bracket((xs, y0 + P45 if i == 0 else y0, zp), (0, 0, -1), ny, (0, 1, 0), (0, 0, 1))
                bracket((x_in, yc, zp), (0, 0, -1), n_in, (1, 0, 0), (0, 0, 1))
            else:
                for yf, ny in ((y0, (0, -1, 0)), (y0 + P45, (0, 1, 0))):
                    bracket((xs, yf, zp), (0, 0, -1), ny, (0, 1, 0), (0, 0, 1))
            if end_leg:
                rot = 0 if i == 0 else 180                 # wheels trail towards the middle
                a.add("ALP5527", "tocak_sa_stopom_CD80", castor.moved(
                    cq.Location(cq.Vector(xs, yc, -CASTOR_FLOOR), cq.Vector(0, 0, 1), rot)), RED)
            else:
                f = place_local(foot, (1, 0, 0), (0, 0, 1), (FOOT_AXIS[0], FOOT_FLOOR, FOOT_AXIS[1]),
                                (xs, yc, 0))
                a.add("ALP2512", "stopa_fi39_Al_guma", f, DARK)
                a.add("ALP2613", "brezon_M12x100", place_local(
                    stud, (1, 0, 0), (0, -1, 0), (STUD_AXIS[0], STUD_AXIS[1], STUD_BALL), (xs, yc, ball_z)), STEEL)

        # --- spacer brackets (spacer = 45x45 between the profiles, bottom 45 mm)
        for y0 in spacer_y:
            faces = []
            if y0 > 1e-6:
                faces.append((y0, (0, -1, 0)))
            if y0 + P45 < L - 1e-6:
                faces.append((y0 + P45, (0, 1, 0)))
            for yf, ny in faces:
                bracket((x_in, yf, zp + 22.5), n_in, ny, (0, 1, 0), (1, 0, 0))

    # --- spacers
    for y0 in spacer_y:
        a.add("ALP0045", "distancer_45x45x500",
              frame_move(p45_spacer, (0, 1, 0), (0, 0, 1), (-GAP / 2, y0 + P45 / 2, zp + 22.5)), AL)

    # --- linear rails, screws, nuts, carriages, stoppers
    rail, holes = rail_model(L)
    carr = carriage_model()
    stop = stopper_model()
    cap_screw = socket_screw(8, RAIL_SCREW_L, 13.0, 8.0)
    for side in (-1, 1):
        xs = side * X_C
        a.add("RAIL", "linearna_sina_35", rail.moved(cq.Location(cq.Vector(xs, 0, z_top))), BLACK)
        for yh in holes:
            a.add("DIN912", "vijak_DIN912_M8x30", cap_screw.moved(
                cq.Location(cq.Vector(xs, yh, z_top + RAIL["H"] - RAIL["h"]))), DARK)
            a.add("ALP0643", "klizna_navrtka_M8_T10",
                  t_nut((xs, yh, z_top), (0, 0, -1), (0, 1, 0)), STEEL)
        for yc in CARRIAGE_Y:
            a.add("CARR", "kolica_35_siroka", carr.moved(cq.Location(cq.Vector(xs, yc, z_top))), STEEL)
        # stoppers centred over the first / last rail hole (set screw in its counterbore)
        a.add("STOP", "stoper_sine", stop.moved(
            cq.Location(cq.Vector(xs, holes[0] - STOPPER["L"] / 2, z_top))), RED)
        a.add("STOP", "stoper_sine", stop.moved(
            cq.Location(cq.Vector(xs, holes[-1] + STOPPER["L"] / 2, z_top), cq.Vector(0, 0, 1), 180)), RED)

    # --- carriage plate + M10 screws + 2 dowel pins Ø8x24 (9 mm stick out into the robot base)
    z_plate = z_top + CARR["H"]
    plate, carr_pts, dowels = plate_model()
    a.add("PLATE", "ploca_kolica", plate.moved(cq.Location(cq.Vector(0, 0, z_plate))), PLATE_RGB)
    m10 = socket_screw(10, 30.0, 16.0, 10.0)
    for x, y in carr_pts:
        a.add("DIN912-M10x30", "vijak_DIN912_M10x30", m10.moved(
            cq.Location(cq.Vector(x, PLATE_Y + y, z_plate + PLATE["T"] - M10_CB["h"]))), DARK)
    pin = cq.Workplane("XY").circle(4.0).extrude(24.0).val()
    for x, y in dowels:
        a.add("PIN8x24", "cilindricna_civija_8x24", pin.moved(
            cq.Location(cq.Vector(x, PLATE_Y + y, z_plate + PLATE["T"] - 15.0))), STEEL)
    return a


# ----------------------------------------------------------------------------
# Export
# ----------------------------------------------------------------------------
def export(asm, stl_path=None, step_path=None, tol=0.3, ang_tol=0.5):
    """STL: one binary mesh of the whole assembly (absolute 0.3 mm chord tolerance keeps
    it ~30 MB). STEP: assembly with one named, coloured node per part; repeated parts
    are written once and instanced."""
    to_rail = cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), -90)   # build +Y -> +X
    parts = [(c, n, s.moved(to_rail), rgb) for c, n, s, rgb in asm.parts]
    if stl_path:
        comp = cq.Compound.makeCompound([p[2] for p in parts])
        comp.exportStl(stl_path, tolerance=tol, angularTolerance=ang_tol, ascii=False, relative=False)
    if step_path:
        # identical parts (same TShape, different placement) share one STEP definition
        top = cq.Assembly(name="robot_rail")
        groups = []                                   # (base shape, sub-assembly)
        counts = {}
        for code, name, shape, rgb in parts:
            sub = next((sa for base, sa in groups if base.wrapped.IsPartner(shape.wrapped)), None)
            if sub is None:
                sub = cq.Assembly(shape.located(cq.Location()), name=f"{code}_{name}_{len(groups):03d}",
                                  color=cq.Color(*rgb))
                groups.append((shape, sub))
            counts[name] = counts.get(name, 0) + 1
            top.add(sub, loc=shape.location(), name=f"{name}_{counts[name]:03d}")
        top.export(step_path)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    zips = sorted(glob.glob(os.path.join(here, "..", "3D modeli - Telix ALP katalog*.zip")) +
                  glob.glob(os.path.join(here, "..", "..", "3D modeli - Telix ALP katalog*.zip")))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zip", default=zips[0] if zips else None, required=not zips,
                    help="Telix ALP STEP catalog zip (default: found next to mechanical_assembly/ or one level up)")
    ap.add_argument("--stl", default=os.path.join(here, "..", "robot_rail_v3.stl"))
    ap.add_argument("--step", default=None, help="also write an assembly STEP (optional)")
    args = ap.parse_args()
    asm = build(Catalog(args.zip))
    export(asm, args.stl, args.step)
    print(f"{len(asm.parts)} parts -> {args.stl}" + (f", {args.step}" if args.step else ""))


if __name__ == "__main__":
    main()
