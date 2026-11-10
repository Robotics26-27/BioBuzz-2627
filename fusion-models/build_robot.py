#!/usr/bin/env python3
"""BioBuzz 2627 (FTC DECODE-era chassis, BIOBUZZ game) robot CAD generator.

CadQuery 2.8. Builds simplified solids at measured goBILDA sizes/positions,
runs geometric asserts A1..A7 (exit 1 on failure), then exports:
  robot_assembly.step, robot_assembly_with_pollen.step, prints/*.step|stl,
  bom.csv, renders/*.png, fetch_gobilda_step.sh, .gitignore
Usage: python3 build_robot.py [out_dir]   (default: directory of this script)

Frame: X = width, Y = length, Z = up, Z=0 = floor. -Y = open end = intake/front.
Ramp frame: s = distance along the ramp surface from its floor line (Y=RAMP_Y0_MM,
Z=0), n = normal distance above the ramp surface. u = (0,cos,sin), nrm = (0,-sin,cos).
Not modelled: screws/nuts, shaft collars/spacers, channel hole patterns, wires.
"""
import csv
import fnmatch
import math
import os
import sys

import cadquery as cq

OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------- parameters
LIMIT_MM = 457.2                 # R102 18 in cube
TARGET_MM = 450.0                # team margin target
FRAME_DZ_MM = 51.97              # Strafer STEP axle z=0 -> floor frame
WALL_MM = 2.5                    # 1120 U-channel wall
CH_MM = 48.0                     # 1120 U-channel section
RAIL_X_MM = (132.0, 180.0)       # rail |x| range (measured)
FRAME_Z_MM = (28.0, 76.0)        # channel z range after shift
WHEEL_R_MM = 51.97               # 104 mecanum, tight-bbox radius so wheels touch Z=0
WHEEL_W_MM = 37.8
WHEEL_X_MM = (188.1, 225.9)
WHEEL_Y_MM = 168.0

BALL_D_MM = 71.0                 # POLLEN (G407: max 4 held)
BALL_R_MM = BALL_D_MM / 2
BALL_GAP_MM = 0.1                # ref balls float this far off the ramp plate
BALL_PITCH_MM = 71.5             # stored balls touch-ish (0.5 mm gap)

RAMP_DEG = 55.0                  # launch angle = ramp angle
RAMP_Y0_MM = -85.0               # where the ball-contact line meets the floor
PLATE_S0_MM = 3.0                # plate lower edge s (keeps underside above floor)
PLATE_T_MM = 1.5
PLATE_W_MM = 88.0                # 1117-0088-0352
PLATE_L_MM = 352.0
LANE_GAP_MM = 78.0               # inner gap between lane walls
LANE_WALL_H_MM = 40.0            # 1117-0040-0352 on edge
LANE_WALL_S0_MM = 55.0           # walls start up the plate so off-centre balls meet the funnel, not the wall end

ROLLER_D_MM = 50.0               # team's existing rubber-band roller stand-in
ROLLER_COMPRESS_MM = 8.0
INTAKE_ROLLER_LEN_MM = 120.0
XFER_ROLLER_LEN_MM = 76.0
INTAKE_Y_MM = -176.0             # roller axis Y; further forward than ~-150 so the intake ball
                                 # (71 mm) cannot overlap the ball resting in the ramp corner
RHINO_D_MM = 96.0                # 3614-0014-0096
RHINO_W_MM = 32.0
FLYWHEEL_COMPRESS_MM = 6.0       # pinch: gap wheel->ramp = 71 - 6 = 65
HUB_MM = (24.0, 20.0)            # hyper hub square, length

PULLEY_SKU = "3422-4008-0020"   # GT2 2 mm, 20T, 8 mm REX bore, all belts 1:1
PULLEY_PD_MM = 20 * 2.0 / math.pi
PULLEY_OD_MM = PULLEY_PD_MM - 0.51
PULLEY_W_MM = 14.0               # stand-in length (hub + flanges, not measured)
BELT_W_MM = 6.0
BELT_PITCH_LEN_MM = 20 * 2.0     # L = 2*C + 40 for 20T:20T
PLANE_R_X_MM = (97.0, 111.0)     # right belt plane (pulley x range)
PLANE_L_X_MM = (-111.0, -97.0)   # left belt plane

MOTOR_D_MM = 37.5
MOTOR6000_L_MM = 107.67          # 5203-2402-0001 face->rear (small_parts.md)
MOTOR435_L_MM = 116.55           # 5203-2402-0014 face->rear
MOTOR_SHAFT_L_MM = 23.5
MOUNT_T_MM = 12.0                # 1201-0043-0002 is 43 x 12 x 43
MOUNT_W_MM = 43.0
MOUNT_X_MM = (84.0, 96.0)        # right-side motor mounts; motors point +X

SHAFT_R_MM = 4.0
BEARING_R_MM = 7.5               # 1611 flanged, 15 flange dia x 5
BEARING_T_MM = 5.0

GT2_BELTS_MM = {184: "3423-0006-0184", 264: "3423-0006-0264", 328: "3423-0006-0328", 514: "3423-0006-0514"}

# 3D print params
PRINT_WALL_MM = 4.0
M4_CLEAR_MM = 4.3

SIN, COS = math.sin(math.radians(RAMP_DEG)), math.cos(math.radians(RAMP_DEG))
TAN = SIN / COS

# Parts confirmed on gobilda.com by U2 (not in catalog.json): SKU -> URL (page shows the SKU)
CONFIRMED_URLS = {
    "3423-0006-0184": "https://www.gobilda.com/2mm-pitch-gt2-timing-belt-6mm-width-184mm-pitch-length-92-tooth/",
    "3423-0006-0264": "https://www.gobilda.com/2mm-pitch-gt2-timing-belt-6mm-width-264mm-pitch-length-132-tooth/",
    "3423-0006-0328": "https://www.gobilda.com/2mm-pitch-gt2-timing-belt-6mm-width-328mm-pitch-length-164-tooth/",
    "3423-0006-0514": "https://www.gobilda.com/2mm-pitch-gt2-timing-belt-6mm-width-514mm-pitch-length-257-tooth/",
}
# Copied from U1 catalog.json (product_url) so A7 runs without the scratch catalog; if
# GOBILDA_CATALOG / ../parts/catalog.json exists, A7 also cross-checks against it.
CATALOG_URLS = {
    "3209-0001-0007": "https://www.gobilda.com/strafer-chassis-kit-104mm-gripforce-mecanum-wheels/",
    "1120-0010-0264": "https://www.gobilda.com/1120-series-u-channel-10-hole-264mm-length/",
    "1120-0005-0144": "https://www.gobilda.com/1120-series-u-channel-5-hole-144mm-length/",
    "1120-0013-0336": "https://www.gobilda.com/1120-series-u-channel-13-hole-336mm-length/",
    "1201-0043-0002": "https://www.gobilda.com/1201-series-quad-block-pattern-mount-43-2/",
    "1117-0088-0352": "https://www.gobilda.com/clear-polycarbonate-grid-plate-1-5mm-thickness-11-x-44-hole-88-x-352mm/",
    "1117-0040-0352": "https://www.gobilda.com/clear-polycarbonate-grid-plate-1-5mm-thickness-5-x-44-hole-40-x-352mm/",
    "1611-0514-4008": "https://www.gobilda.com/1611-series-flanged-ball-bearing-8mm-rex-id-x-14mm-od-5mm-thickness-2-pack/",
    "2106-4008-3840": "https://www.gobilda.com/8mm-rex-shaft-with-e-clip-stainless-steel-384mm-length/",
    "2106-4008-2880": "https://www.gobilda.com/8mm-rex-shaft-with-e-clip-stainless-steel-288mm-length/",
    "3614-0014-0096": "https://www.gobilda.com/3614-series-rhino-wheel-14mm-bore-96mm-diameter-30a-durometer/",
    "1310-0016-4008": "https://www.gobilda.com/1310-series-hyper-hub-8mm-rex-bore/",
    "5203-2402-0001": "https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/",
    "5203-2402-0014": "https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/",
    "3422-4008-0020": "https://www.gobilda.com/2mm-pitch-gt2-pinion-timing-pulley-8mm-rex-bore-20-tooth/",
}
CATALOG = os.environ.get("GOBILDA_CATALOG", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "parts", "catalog.json"))


# ---------------------------------------------------------------- helpers
def V(x, y, z):
    return cq.Vector(x, y, z)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_x(x0, x1, y, z, r):
    return cq.Solid.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def cyl_y(y0, y1, x, z, r):
    return cq.Solid.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def sphere(c, r):
    return cq.Solid.makeSphere(r, V(*c), V(0, 0, 1), -90, 90, 360)


def uchannel(x, y, z, long_axis, web):
    """U-channel solid in bbox x,y,z (pairs). web: '+x','-x','+y','-y','+z','-z' (web side);
    open side is opposite the web; flanges are the two remaining faces."""
    b = [list(x), list(y), list(z)]
    outer = box(*x, *y, *z)
    ax = "xyz".index(long_axis)
    wa = "xyz".index(web[1])
    cut = [list(v) for v in b]
    cut[ax] = [b[ax][0] - 1, b[ax][1] + 1]
    if web[0] == "+":
        cut[wa] = [b[wa][0] - 1, b[wa][1] - WALL_MM]
    else:
        cut[wa] = [b[wa][0] + WALL_MM, b[wa][1] + 1]
    fa = 3 - ax - wa
    cut[fa] = [b[fa][0] + WALL_MM, b[fa][1] - WALL_MM]
    return outer.cut(box(*cut[0], *cut[1], *cut[2]))


def ramp_pt(s, n, x=0.0):
    return (x, RAMP_Y0_MM + s * COS - n * SIN, s * SIN + n * COS)


def on_ramp(shape):
    """Local (x, s, n) as (X, Y, Z) -> world."""
    return shape.rotate(V(0, 0, 0), V(1, 0, 0), RAMP_DEG).translate(V(0, RAMP_Y0_MM, 0))


def ramp_under_z(y):
    """World Z of the ramp plate underside at world Y."""
    return (y - RAMP_Y0_MM) * TAN - PLATE_T_MM / COS


def slot_belt(p1, p2, x0, x1, r_in, r_out):
    """Belt loop (stadium ring) between pulley centres p1,p2 = (y,z), in plane x0..x1."""
    (y1, z1), (y2, z2) = p1, p2
    c = math.hypot(y2 - y1, z2 - z1)
    ang = math.degrees(math.atan2(z2 - z1, y2 - y1))
    mid = ((y1 + y2) / 2, (z1 + z2) / 2)

    def stadium(r):
        return (cq.Workplane("YZ", origin=(x0, 0, 0)).center(*mid)
                .slot2D(c + 2 * r, 2 * r, ang).extrude(x1 - x0).val())
    return stadium(r_out).cut(stadium(r_in)), c


def poly_extrude(pts, z0, z1, plane="XY"):
    return cq.Workplane(plane).workplane(offset=z0).polyline(pts).close().extrude(z1 - z0).val()


# ---------------------------------------------------------------- part registry
PARTS = []  # (group, name, shape, rgb)
COL = {"alu": (0.75, 0.75, 0.78), "rail": (0.25, 0.4, 0.85), "wheel": (0.3, 0.3, 0.3),
       "motor": (0.95, 0.75, 0.1), "poly": (0.7, 0.85, 1.0), "print": (0.1, 0.75, 0.3),
       "team": (0.85, 0.2, 0.2), "rhino": (0.15, 0.15, 0.15), "steel": (0.55, 0.55, 0.6),
       "belt": (0.1, 0.1, 0.1), "env": (0.6, 0.3, 0.8), "ball": (1.0, 0.85, 0.2)}
COUNT = {}


def add(group, sku, short, shape, col):
    key = (sku, short)
    COUNT[key] = COUNT.get(key, 0) + 1
    name = f"{sku}_{short}_{COUNT[key]}"
    PARTS.append((group, name, shape, COL[col]))
    return name


def add_motor(group, sku, short, length, x_face, y, z):
    """Motor axis along X, output shaft pointing +X from face at x_face. Bolts to a 1201 mount face."""
    body = cyl_x(x_face - length, x_face, y, z, MOTOR_D_MM / 2)
    body = body.fuse(cyl_x(x_face, x_face + MOTOR_SHAFT_L_MM, y, z, SHAFT_R_MM))
    return add(group, sku, short, body, "motor")


def add_pulley(group, short, x0, y, z):
    return add(group, PULLEY_SKU, short, cyl_x(x0, x0 + PULLEY_W_MM, y, z, PULLEY_OD_MM / 2), "alu")


def add_belt(group, short, p1, p2, plane, belt_len_mm):
    xm = (plane[0] + plane[1]) / 2
    shape, c = slot_belt(p1, p2, xm - BELT_W_MM / 2, xm + BELT_W_MM / 2,
                         PULLEY_PD_MM / 2 - 1.0, PULLEY_PD_MM / 2 + 0.6)
    c_belt = (belt_len_mm - BELT_PITCH_LEN_MM) / 2
    BELTS.append((short, GT2_BELTS_MM[belt_len_mm], belt_len_mm, c, c_belt))
    return add(group, GT2_BELTS_MM[belt_len_mm], short, shape, "belt")


BELTS = []


def add_bearings(group, short, x_faces, y, z):
    """1611-0514-4008 flanged bearing in each channel/mount face (x_faces = face mid-planes)."""
    for xf in x_faces:
        add(group, "1611-0514-4008", short, cyl_x(xf - BEARING_T_MM / 2, xf + BEARING_T_MM / 2, y, z, BEARING_R_MM), "steel")


def mount_in_end_z(x, y, z0, z1):
    """1201-0043-0002 sitting inside a vertical channel end (43x43 in plan, 12 tall)."""
    return box(*x, *y, z0, z1)


# ================================================================ BUILD
ZB, ZT = FRAME_Z_MM

# ---------------- drivetrain (Strafer 3209-0001-0007, simplified, measured positions)
for sx in (-1, 1):
    xr = sorted((sx * RAIL_X_MM[0], sx * RAIL_X_MM[1]))
    # rail: web on top, open down; bolts to cross members via kit 1201 blocks
    add("drivetrain", "1120-0017-0432", "rail", uchannel(xr, (-216, 216), (ZB, ZT), "y", "+z"), "rail")
for yc in ((-24, 24), (144, 192)):
    # cross members A (centre) and B (rear), web up, bolted between rails (kit)
    add("drivetrain", "1120-0010-0264", "crossmember", uchannel((-132, 132), yc, (ZB, ZT), "x", "+z"), "rail")
for sx in (-1, 1):
    for sy in (-1, 1):
        sku = "3625-0001-0104" if sx * sy > 0 else "3625-0100-0104"
        xw = sorted((sx * WHEEL_X_MM[0], sx * WHEEL_X_MM[1]))
        # mecanum wheel on kit axle
        add("drivetrain", sku, "mecanum", cyl_x(*xw, sy * WHEEL_Y_MM, FRAME_DZ_MM, WHEEL_R_MM), "wheel")
        # axle stub through rail flange bearings into wheel (kit part)
        xa = sorted((sx * 131.0, sx * 203.0))
        add("drivetrain", "3209-0001-0007", "axle", cyl_x(*xa, sy * WHEEL_Y_MM, FRAME_DZ_MM, SHAFT_R_MM), "steel")
        # drive motor inside rail cavity, axis Y (kit 5203-2402-0019)
        yr = sorted((sy * 21.4, sy * 163.5))
        add("drivetrain", "5203-2402-0019", "drive-motor", cyl_y(*yr, sx * 156.0, FRAME_DZ_MM, 18.8), "motor")

# ---------------- pollen lane (ramp plate + side walls)
s1 = PLATE_S0_MM
plate_local = box(-PLATE_W_MM / 2, PLATE_W_MM / 2, s1, s1 + PLATE_L_MM, -PLATE_T_MM, 0)
add("pollen_lane", "1117-0088-0352", "ramp-plate", on_ramp(plate_local), "poly")  # screws to both ramp_brackets
gx = LANE_GAP_MM / 2
for sx in (-1, 1):
    xw = sorted((sx * gx, sx * (gx + PLATE_T_MM)))
    w = box(*xw, LANE_WALL_S0_MM, LANE_WALL_S0_MM + PLATE_L_MM, 0, LANE_WALL_H_MM)
    add("pollen_lane", "1117-0040-0352", "lane-wall", on_ramp(w), "poly")  # held to plate by wall_clips

# ---------------- reference ball positions (ramp coords)
# lowest ball resting on floor + ramp: centre z = r  ->  s*SIN + n*COS = r with n = r + gap
N_BALL = BALL_R_MM + BALL_GAP_MM
S_CORNER = (BALL_R_MM + 0.1 - N_BALL * COS) / SIN
S_BALLS = [S_CORNER + i * BALL_PITCH_MM for i in range(4)]
S_PINCH = S_CORNER + 4 * BALL_PITCH_MM
N_ROLLER = BALL_D_MM - ROLLER_COMPRESS_MM + ROLLER_D_MM / 2
N_FLY = BALL_D_MM - FLYWHEEL_COMPRESS_MM + RHINO_D_MM / 2

# ---------------- launcher
_, Y_FLY, Z_FLY = ramp_pt(S_PINCH, N_FLY)
S_XB = S_BALLS[3]                          # roller B grips ball 4
S_XA = S_XB - (328 - BELT_PITCH_LEN_MM) / 2  # roller A from 328 belt C = 144 -> grips ball 2
_, Y_XB, Z_XB = ramp_pt(S_XB, N_ROLLER)
_, Y_XA, Z_XA = ramp_pt(S_XA, N_ROLLER)
UP_FLY_Y = (-35.0, 13.0)                   # upright spans flywheel shaft and roller-B shaft
assert UP_FLY_Y[0] + 9 < Y_XB and Y_FLY < UP_FLY_Y[1] - 9, (Y_XB, Y_FLY)
UP_FLY_Z = (ZT, ZT + 336.0)
for sx in (-1, 1):
    xr = sorted((sx * RAIL_X_MM[0], sx * RAIL_X_MM[1]))
    # flywheel upright: stands on rail top, 1201 in its lower end bolts to rail web; flanges carry shaft bearings
    add("launcher", "1120-0013-0336", "upright-fly", uchannel(xr, UP_FLY_Y, UP_FLY_Z, "z", "-y"), "alu")
    add("launcher", "1201-0043-0002", "mount-upright-fly",
        mount_in_end_z((xr[0] + WALL_MM, xr[1] - WALL_MM), (UP_FLY_Y[0] + WALL_MM, UP_FLY_Y[0] + WALL_MM + MOUNT_W_MM), ZT, ZT + MOUNT_T_MM), "alu")
    add_bearings("launcher", "bearing-fly", [sx * (RAIL_X_MM[0] + WALL_MM / 2), sx * (RAIL_X_MM[1] - WALL_MM / 2)], Y_FLY, Z_FLY)
# top bar ties the uprights; 1201 blocks in its ends bolt to upright inner flanges; web up, open down
BAR_FLY_Z = (UP_FLY_Z[1] - CH_MM, UP_FLY_Z[1])
add("launcher", "1120-0010-0264", "bar-fly", uchannel((-132, 132), UP_FLY_Y, BAR_FLY_Z, "x", "+z"), "alu")
for sx in (-1, 1):
    add("launcher", "1201-0043-0002", "mount-bar-fly",
        box(*sorted((sx * 120.0, sx * 132.0)), UP_FLY_Y[0] + WALL_MM, UP_FLY_Y[1] - WALL_MM, BAR_FLY_Z[1] - WALL_MM - MOUNT_W_MM, BAR_FLY_Z[1] - WALL_MM), "alu")
add("launcher", "2106-4008-3840", "shaft-fly", cyl_x(-192, 192, Y_FLY, Z_FLY, SHAFT_R_MM), "steel")
for sx in (-1, 1):
    x0, x1 = sorted((sx * 0.5, sx * (0.5 + RHINO_W_MM)))
    wheel = cyl_x(x0, x1, Y_FLY, Z_FLY, RHINO_D_MM / 2).cut(cyl_x(x0 - 1, x1 + 1, Y_FLY, Z_FLY, 7.0))
    add("launcher", "3614-0014-0096", "rhino-fly", wheel, "rhino")  # screwed to hyper hub
    h0, h1 = sorted((sx * (0.5 + RHINO_W_MM), sx * (0.5 + RHINO_W_MM + HUB_MM[1])))
    add("launcher", "1310-0016-4008", "hub-fly", box(h0, h1, Y_FLY - 12, Y_FLY + 12, Z_FLY - 12, Z_FLY + 12), "alu")  # on shaft
add_pulley("launcher", "pulley-fly", PLANE_R_X_MM[0], Y_FLY, Z_FLY)
# flywheel motor: 1201 bolted to bar's front flange, motor above-front of flywheel, 184 belt
C_FLY = (184 - BELT_PITCH_LEN_MM) / 2
Y_MFLY = UP_FLY_Y[0] - MOUNT_W_MM / 2
Z_MFLY = Z_FLY + math.sqrt(C_FLY ** 2 - (Y_MFLY - Y_FLY) ** 2)
add("launcher", "1201-0043-0002", "mount-motor-fly",
    box(*MOUNT_X_MM, UP_FLY_Y[0] - MOUNT_W_MM, UP_FLY_Y[0], Z_MFLY - MOUNT_W_MM / 2, Z_MFLY + MOUNT_W_MM / 2), "alu")
add_motor("launcher", "5203-2402-0001", "motor-fly", MOTOR6000_L_MM, MOUNT_X_MM[0], Y_MFLY, Z_MFLY)
add_pulley("launcher", "pulley-motor-fly", PLANE_R_X_MM[0], Y_MFLY, Z_MFLY)
add_belt("launcher", "belt-fly", (Y_FLY, Z_FLY), (Y_MFLY, Z_MFLY), PLANE_R_X_MM, 184)

# ---------------- transfer
UP_XA_Y = (-142.0, -94.0)   # forward of shaft A so bar-xa clears the A-B belt
assert UP_XA_Y[0] + 9 < Y_XA < UP_XA_Y[1] - 9, Y_XA
UP_XA_Z = (ZT, ZT + 144.0)
for sx in (-1, 1):
    xr = sorted((sx * RAIL_X_MM[0], sx * RAIL_X_MM[1]))
    # short upright on rail for roller A shaft; 1201 in lower end bolts to rail web
    add("transfer", "1120-0005-0144", "upright-xa", uchannel(xr, UP_XA_Y, UP_XA_Z, "z", "-y"), "alu")
    add("transfer", "1201-0043-0002", "mount-upright-xa",
        mount_in_end_z((xr[0] + WALL_MM, xr[1] - WALL_MM), (UP_XA_Y[0] + WALL_MM, UP_XA_Y[0] + WALL_MM + MOUNT_W_MM), ZT, ZT + MOUNT_T_MM), "alu")
    add_bearings("transfer", "bearing-xa", [sx * (RAIL_X_MM[0] + WALL_MM / 2), sx * (RAIL_X_MM[1] - WALL_MM / 2)], Y_XA, Z_XA)
    # roller B shaft runs in the flywheel uprights
    add_bearings("transfer", "bearing-xb", [sx * (RAIL_X_MM[0] + WALL_MM / 2), sx * (RAIL_X_MM[1] - WALL_MM / 2)], Y_XB, Z_XB)
BAR_XA_Z = (UP_XA_Z[1] - CH_MM, UP_XA_Z[1])
# bar across A-uprights (carries intake motor); 1201s in ends bolt to upright inner flanges
add("transfer", "1120-0010-0264", "bar-xa", uchannel((-132, 132), UP_XA_Y, BAR_XA_Z, "x", "+z"), "alu")
for sx in (-1, 1):
    add("transfer", "1201-0043-0002", "mount-bar-xa",
        box(*sorted((sx * 120.0, sx * 132.0)), UP_XA_Y[0] + WALL_MM, UP_XA_Y[1] - WALL_MM, BAR_XA_Z[1] - WALL_MM - MOUNT_W_MM, BAR_XA_Z[1] - WALL_MM), "alu")
for tag, y, z in (("xa", Y_XA, Z_XA), ("xb", Y_XB, Z_XB)):
    add("transfer", "2106-4008-3840", f"shaft-{tag}", cyl_x(-192, 192, y, z, SHAFT_R_MM), "steel")
    add("transfer", "TEAM", f"existing_band_roller-{tag}", cyl_x(-XFER_ROLLER_LEN_MM / 2, XFER_ROLLER_LEN_MM / 2, y, z, ROLLER_D_MM / 2), "team")
    add_pulley("transfer", f"pulley-{tag}", PLANE_L_X_MM[0], y, z)      # A-B link, left plane
add_belt("transfer", "belt-xab", (Y_XA, Z_XA), (Y_XB, Z_XB), PLANE_L_X_MM, 328)
add_pulley("transfer", "pulley-xb", PLANE_R_X_MM[0], Y_XB, Z_XB)        # motor belt, right plane
# transfer motor: 1201 bottom edge bolted to rear cross member B web, 514 belt to shaft B
C_XM = (514 - BELT_PITCH_LEN_MM) / 2
Z_MX = ZT + MOUNT_W_MM / 2
Y_MX = Y_XB + math.sqrt(C_XM ** 2 - (Z_XB - Z_MX) ** 2)
assert 144 + MOUNT_W_MM / 2 - 1 <= Y_MX <= 192 - MOUNT_W_MM / 2, Y_MX  # <=1 mm overhang off the web allowed
add("transfer", "1201-0043-0002", "mount-motor-xfer", box(*MOUNT_X_MM, Y_MX - MOUNT_W_MM / 2, Y_MX + MOUNT_W_MM / 2, ZT, ZT + MOUNT_W_MM), "alu")
add_motor("transfer", "5203-2402-0014", "motor-xfer", MOTOR435_L_MM, MOUNT_X_MM[0], Y_MX, Z_MX)
add_pulley("transfer", "pulley-motor-xfer", PLANE_R_X_MM[0], Y_MX, Z_MX)
add_belt("transfer", "belt-xfer", (Y_XB, Z_XB), (Y_MX, Z_MX), PLANE_R_X_MM, 514)

# ---------------- intake
Z_INT = BALL_D_MM - ROLLER_COMPRESS_MM + ROLLER_D_MM / 2
for sx in (-1, 1):
    # 1201 bolted flat to rail inner flange; its centre bore holds the shaft bearing
    xm = sorted((sx * (RAIL_X_MM[0] - MOUNT_T_MM), sx * RAIL_X_MM[0]))
    add("intake", "1201-0043-0002", "mount-intake", box(*xm, INTAKE_Y_MM - MOUNT_W_MM / 2, INTAKE_Y_MM + MOUNT_W_MM / 2, Z_INT - MOUNT_W_MM / 2, Z_INT + MOUNT_W_MM / 2), "alu")
    add_bearings("intake", "bearing-intake", [sx * (RAIL_X_MM[0] - MOUNT_T_MM / 2)], INTAKE_Y_MM, Z_INT)
add("intake", "2106-4008-2880", "shaft-intake", cyl_x(-144, 144, INTAKE_Y_MM, Z_INT, SHAFT_R_MM), "steel")
add("intake", "TEAM", "existing_band_roller-intake", cyl_x(-INTAKE_ROLLER_LEN_MM / 2, INTAKE_ROLLER_LEN_MM / 2, INTAKE_Y_MM, Z_INT, ROLLER_D_MM / 2), "team")
add_pulley("intake", "pulley-intake", PLANE_R_X_MM[0], INTAKE_Y_MM, Z_INT)
C_INT = (264 - BELT_PITCH_LEN_MM) / 2
Y_MI = UP_XA_Y[0] - MOUNT_W_MM / 2
Z_MI = Z_INT + math.sqrt(C_INT ** 2 - (Y_MI - INTAKE_Y_MM) ** 2)
# intake motor: 1201 bolted to front flange of bar-xa
add("intake", "1201-0043-0002", "mount-motor-intake", box(*MOUNT_X_MM, UP_XA_Y[0] - MOUNT_W_MM, UP_XA_Y[0], Z_MI - MOUNT_W_MM / 2, Z_MI + MOUNT_W_MM / 2), "alu")
add_motor("intake", "5203-2402-0014", "motor-intake", MOTOR435_L_MM, MOUNT_X_MM[0], Y_MI, Z_MI)
add_pulley("intake", "pulley-motor-intake", PLANE_R_X_MM[0], Y_MI, Z_MI)
add_belt("intake", "belt-intake", (INTAKE_Y_MM, Z_INT), (Y_MI, Z_MI), PLANE_R_X_MM, 264)

# ---------------- ramp support structure (cross member C, post, electronics deck)
BRACKET_GAP = ramp_under_z(-24.0) - ZT     # wedge front height at cross member A
DECK_Z = (ZT, ZT + PLATE_T_MM)
POST_Z = (DECK_Z[1], DECK_Z[1] + 144.0)
Y_POST0 = (POST_Z[1] + BRACKET_GAP + PLATE_T_MM / COS) / TAN + RAMP_Y0_MM
POST_Y = (Y_POST0, Y_POST0 + CH_MM)
# cross member C between rails like the kit ones (1201s in its ends bolt to rail inner flanges)
add("pollen_lane", "1120-0010-0264", "crossmember-c", uchannel((-132, 132), POST_Y, (ZB, ZT), "x", "+z"), "alu")
for sx in (-1, 1):
    add("pollen_lane", "1201-0043-0002", "mount-cross-c", box(*sorted((sx * 120.0, sx * 132.0)), POST_Y[0] + WALL_MM, POST_Y[1] - WALL_MM, ZT - WALL_MM - MOUNT_W_MM, ZT - WALL_MM), "alu")
# electronics deck: polycarbonate grid plate across rail tops, screwed to rails and cross member C
DECK_Y = (30.0, 118.0)
add("electronics", "1117-0088-0352", "deck", box(-176, 176, *DECK_Y, *DECK_Z), "poly")
# post: vertical channel on deck, 1201 in lower end bolts through deck into cross member C web
add("pollen_lane", "1120-0005-0144", "post", uchannel((-24, 24), POST_Y, POST_Z, "z", "+y"), "alu")
for z0 in (POST_Z[0], POST_Z[1] - MOUNT_T_MM):
    add("pollen_lane", "1201-0043-0002", "mount-post", box(-21.5, 21.5, POST_Y[0] + 0.5, POST_Y[0] + 0.5 + MOUNT_W_MM, z0, z0 + MOUNT_T_MM), "alu")


# ---------------- 3D prints (built flat on the bed, then placed)
def ramp_bracket_print():
    """Wedge: profile in (Y,Z) on the bed, extruded 48 (bed = profile face). Base on a channel
    web/1201, top face = ramp underside. 2 vertical M4 holes (counterbored from top, installed
    before the plate) + 2 slots normal to the ramp face exiting the back face (nut on back)."""
    g, L, W = BRACKET_GAP, CH_MM, CH_MM
    pts = [(0, 0), (L, 0), (L, g + L * TAN), (0, g)]
    wedge = poly_extrude(pts, 0, W)            # local X=y, Y=z, Z=x (print frame)
    for zx in (W / 2 - 16, W / 2 + 16):         # holes at x = +-16 (8 mm grid)
        hole = cq.Solid.makeCylinder(M4_CLEAR_MM / 2, 200, V(12, -1, zx), V(0, 1, 0))
        cb = cq.Solid.makeCylinder(4.2, 200, V(12, 12, zx), V(0, 1, 0))
        wedge = wedge.cut(hole).cut(cb)
        # slot normal to ramp face, centre at yl=32, 12 long along the face
        yl = 32.0
        p = V(yl, g + yl * TAN, zx)
        d = V(SIN, -COS, 0)
        along = V(COS, SIN, 0)
        for k in range(-3, 4):
            c = p + along * (2.0 * k) - d * 2
            wedge = wedge.cut(cq.Solid.makeCylinder(M4_CLEAR_MM / 2, 80, c, d))
    return wedge


def place_bracket(shape, y0, z0):
    # print frame (X=y, Y=z, Z=x) -> world: X_w = Z_p - 24, Y_w = X_p + y0, Z_w = Y_p + z0
    m = cq.Matrix([[0, 0, 1, -CH_MM / 2], [1, 0, 0, y0], [0, 1, 0, z0]])
    return shape.transformGeometry(m)


def funnel_print(side):
    """Plan polygon extruded in Z (bed = bottom face). Guide wall from (100,-216) to (42,-98),
    front web to the rail, flange screwed to rail inner flange (2x M4 along X)."""
    p0, p1 = (100.0, -216.0), (42.0, -98.0)
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    ln = math.hypot(dx, dy)
    nx, ny = dy / ln, -dx / ln            # outward (+x) normal
    t = PRINT_WALL_MM
    wall = [p0, p1, (p1[0] + nx * t, p1[1] + ny * t), (p0[0] + nx * t, max(-216.0, p0[1] + ny * t))]
    z0, z1 = 6.0, 48.0
    s = poly_extrude(wall, z0, z1)
    s = s.fuse(box(p0[0], RAIL_X_MM[0], -216, -210, z0, z1))
    s = s.fuse(box(RAIL_X_MM[0] - t, RAIL_X_MM[0], -216, -184, z0, z1)).clean()
    for y in (-208.0, -192.0):
        s = s.cut(cyl_x(RAIL_X_MM[0] - 20, RAIL_X_MM[0] + 1, y, 36.0, M4_CLEAR_MM / 2))
    if side < 0:
        s = s.mirror("YZ")
    return s


def wall_clip_print():
    """Profile in (x, n) extruded 24 along s. Leg against wall outer face, wraps the plate edge,
    foot under the plate. M4 through leg+wall (n=20) and foot+plate (x=32)."""
    pts = [(gx + PLATE_T_MM, 30), (48, 30), (48, -5.5), (26, -5.5), (26, -PLATE_T_MM),
           (PLATE_W_MM / 2, -PLATE_T_MM), (PLATE_W_MM / 2, 0), (gx + PLATE_T_MM, 0)]
    s = poly_extrude(pts, 0, 24)              # local X=x, Y=n, Z=s
    s = s.cut(cq.Solid.makeCylinder(M4_CLEAR_MM / 2, 40, V(30, 20, 12), V(1, 0, 0)))
    s = s.cut(cq.Solid.makeCylinder(M4_CLEAR_MM / 2, 40, V(32, -20, 12), V(0, 1, 0)))
    return s


def place_clip(shape, side, s0):
    m = cq.Matrix([[1, 0, 0, 0], [0, 0, 1, s0], [0, 1, 0, 0]])   # (x, n, s) -> (x, s, n)
    loc = shape.transformGeometry(m)
    if side < 0:
        loc = loc.mirror("YZ")
    return on_ramp(loc)


PRINTS = {"ramp_bracket": ramp_bracket_print(), "funnel_guide_left": funnel_print(-1),
          "funnel_guide_right": funnel_print(1), "wall_clip": wall_clip_print()}
PRINT_QTY = {"ramp_bracket": 2, "funnel_guide_left": 1, "funnel_guide_right": 1, "wall_clip": 4}
add("prints", "PRINT", "ramp_bracket", place_bracket(PRINTS["ramp_bracket"], -24.0, ZT), "print")       # on cross member A web
add("prints", "PRINT", "ramp_bracket", place_bracket(PRINTS["ramp_bracket"], POST_Y[0], POST_Z[1]), "print")  # on post-top 1201
add("prints", "PRINT", "funnel_guide_left", PRINTS["funnel_guide_left"], "print")    # screwed to left rail inner flange
add("prints", "PRINT", "funnel_guide_right", PRINTS["funnel_guide_right"], "print")  # screwed to right rail inner flange
for s0 in (60.0, 250.0):
    for side in (-1, 1):
        add("prints", "PRINT", "wall_clip", place_clip(PRINTS["wall_clip"], side, s0), "print")  # wall + plate

# ---------------- electronics envelopes (approximate boxes)
# battery standing on the deck in front of the post, strapped to deck (strap not modelled)
add("electronics", "REV", "battery_envelope", box(-60, 75, 32, 57, DECK_Z[1], DECK_Z[1] + 75), "env")
# hubs on edge on the deck, left side (bracket/velcro to deck not modelled); USB/ports face up
add("electronics", "REV", "control_hub_envelope", box(-127.5, -98, 28, 171, DECK_Z[1], DECK_Z[1] + 103), "env")
add("electronics", "REV", "expansion_hub_envelope", box(-95, -65.5, 28, 171, DECK_Z[1], DECK_Z[1] + 103), "env")

# ---------------- reference pollen
BALLS = []
BALLS.append(("ref_intake", sphere((0, INTAKE_Y_MM, BALL_R_MM + 0.1), BALL_R_MM)))
for i, s in enumerate(S_BALLS):
    BALLS.append((f"ref_stored_{i + 1}", sphere(ramp_pt(s, N_BALL), BALL_R_MM)))
BALLS.append(("ref_pinch", sphere(ramp_pt(S_PINCH, N_BALL), BALL_R_MM)))

# ================================================================ ASSERTS
FAIL = []


def check(tag, ok, msg):
    print(("PASS " if ok else "FAIL ") + tag + ": " + msg)
    if not ok:
        FAIL.append(tag)


def ivol(a, b):
    ba, bb = a.BoundingBox(), b.BoundingBox()
    if ba.xmin > bb.xmax or bb.xmin > ba.xmax or ba.ymin > bb.ymax or bb.ymin > ba.ymax or ba.zmin > bb.zmax or bb.zmin > ba.zmax:
        return 0.0
    return a.intersect(b).Volume()


robot = cq.Compound.makeCompound([p[2] for p in PARTS])
bb = robot.BoundingBox()
ext = (bb.xlen, bb.ylen, bb.zlen)
wheel_zmin = min(p[2].BoundingBox().zmin for p in PARTS if "mecanum" in p[1])
ok = bb.zmin >= -1e-6 and abs(wheel_zmin) <= 0.1 and all(e <= LIMIT_MM for e in ext)
check("A1", ok, "extents X %.1f Y %.1f Z %.1f mm; margin to 457.2: %.1f / %.1f / %.1f; to 450 target: %.1f / %.1f / %.1f; Zmin %.3f, wheel Zmin %.3f"
      % (*ext, *(LIMIT_MM - e for e in ext), *(TARGET_MM - e for e in ext), bb.zmin, wheel_zmin))

# A2 interference whitelist: (pattern_a, pattern_b, reason)
WHITELIST = [
    ("*axle*", "*rail*", "kit axle passes through rail flange bearing bore"),
    ("*axle*", "*mecanum*", "wheel mounted on axle"),
    ("*shaft-fly*", "*bearing-fly*", "shaft runs in flanged bearing"),
    ("*shaft-fly*", "*upright-fly*", "shaft passes through 14 mm channel bore"),
    ("*bearing-fly*", "*upright-fly*", "bearing pressed in channel bore"),
    ("*shaft-fly*", "*hub-fly*", "hyper hub on REX shaft"),
    ("*shaft-fly*", "*pulley-fly*", "pulley on shaft"),
    ("*shaft-xb*", "*bearing-xb*", "shaft runs in flanged bearing"),
    ("*shaft-xb*", "*upright-fly*", "roller-B shaft passes through flywheel-upright bore"),
    ("*bearing-xb*", "*upright-fly*", "bearing pressed in channel bore"),
    ("*shaft-xb*", "*roller-xb*", "roller on shaft"),
    ("*shaft-xb*", "*pulley-xb*", "pulley on shaft"),
    ("*shaft-xa*", "*bearing-xa*", "shaft runs in flanged bearing"),
    ("*shaft-xa*", "*upright-xa*", "shaft passes through channel bore"),
    ("*bearing-xa*", "*upright-xa*", "bearing pressed in channel bore"),
    ("*shaft-xa*", "*roller-xa*", "roller on shaft"),
    ("*shaft-xa*", "*pulley-xa*", "pulley on shaft"),
    ("*shaft-intake*", "*bearing-intake*", "shaft runs in flanged bearing"),
    ("*shaft-intake*", "*mount-intake*", "shaft through 1201 centre bore"),
    ("*bearing-intake*", "*mount-intake*", "bearing pressed in 1201 centre bore"),
    ("*shaft-intake*", "*roller-intake*", "roller on shaft"),
    ("*shaft-intake*", "*pulley-intake*", "pulley on shaft"),
    ("*motor-fly*", "*mount-motor-fly*", "motor output shaft through 1201 bore"),
    ("*motor-fly*", "*pulley-motor-fly*", "pulley on motor shaft"),
    ("*motor-xfer*", "*mount-motor-xfer*", "motor output shaft through 1201 bore"),
    ("*motor-xfer*", "*pulley-motor-xfer*", "pulley on motor shaft"),
    ("*motor-intake*", "*mount-motor-intake*", "motor output shaft through 1201 bore"),
    ("*motor-intake*", "*pulley-motor-intake*", "pulley on motor shaft"),
    ("*belt-*", "*pulley-*", "belt teeth in pulley grooves"),
]
names = [p[1] for p in PARTS]
hits, used = [], set()
for i in range(len(PARTS)):
    for j in range(i + 1, len(PARTS)):
        v = ivol(PARTS[i][2], PARTS[j][2])
        if v < 1.0:
            continue
        a, b = names[i], names[j]
        w = next((k for k, (pa, pb, _) in enumerate(WHITELIST)
                  if (fnmatch.fnmatch(a, pa) and fnmatch.fnmatch(b, pb)) or (fnmatch.fnmatch(b, pa) and fnmatch.fnmatch(a, pb))), None)
        if w is None:
            hits.append(f"{a} x {b}: {v:.1f} mm3")
        else:
            used.add(w)
for h in hits:
    print("   interference:", h)
check("A2", not hits, f"{len(PARTS)} solids, {len(hits)} non-whitelisted overlaps >= 1 mm3; {len(used)}/{len(WHITELIST)} whitelist rules used")

# A3 ball path
COMPLIANT = ("existing_band_roller", "rhino")
a3 = []
for bn, bs in BALLS:
    for pn, ps in zip(names, (p[2] for p in PARTS)):
        v = ivol(bs, ps)
        if v <= 0.01:
            continue
        if any(c in pn for c in COMPLIANT):
            # depth = r_ball + r_part - centre-to-axis distance (axes are along X)
            c = bs.Center()
            pb = ps.BoundingBox()
            r = (pb.zmax - pb.zmin) / 2
            d = math.hypot(c.y - (pb.ymin + pb.ymax) / 2, c.z - (pb.zmin + pb.zmax) / 2)
            depth = BALL_R_MM + r - d
            ok = 3.0 <= depth <= 10.0
            a3.append((ok, f"{bn} squeezed by {pn} depth {depth:.2f} mm"))
        else:
            a3.append((False, f"{bn} hits {pn} {v:.2f} mm3"))
for i in range(len(BALLS)):
    for j in range(i + 1, len(BALLS)):
        v = ivol(BALLS[i][1], BALLS[j][1])
        if v > 0.01:
            a3.append((False, f"{BALLS[i][0]} overlaps {BALLS[j][0]} {v:.2f} mm3"))
for ok, m in a3:
    print("   ", "ok " if ok else "BAD", m)
gripped = {m.split()[0] for ok, m in a3 if ok}
occupied = S_BALLS + [S_PINCH]
fifth = [s / 2 for s in range(int(S_CORNER * 2), int(S_PINCH * 2))
         if all(abs(s / 2 - o) >= BALL_D_MM for o in occupied)]
check("A3", all(ok for ok, _ in a3) and not fifth,
      f"balls on ramp at s = {', '.join('%.1f' % s for s in occupied)} (pinch last); compliant contacts {len(gripped)}; "
      f"free slot for a 5th ball between corner s={S_CORNER:.1f} and pinch: {'NONE' if not fifth else fifth[:3]}")

# A4 launch exit
pinch = V(*ramp_pt(S_PINCH, N_BALL))
launch = cq.Solid.makeCylinder(BALL_R_MM, 600, pinch, V(0, COS, SIN))
a4 = [(n, ivol(launch, s)) for n, s in zip(names, (p[2] for p in PARTS)) if "rhino" not in n]
a4 = [(n, v) for n, v in a4 if v > 0.01]
for n, v in a4:
    print("   launch path hits", n, "%.2f mm3" % v)
check("A4", not a4, "71 mm cylinder, 600 mm along 55 deg from pinch, clear of every solid except the pinching Rhino wheels")

# A5 intake entry (capsule along Y on the floor)
Y_FOOT = RAMP_Y0_MM - (BALL_R_MM + 0.5 - BALL_R_MM * COS) / SIN  # ball 0.5 mm short of touching the ramp plane
a5ok, a5msg = True, []
for x in (0.0, 50.0, -50.0):
    zc = BALL_R_MM + 0.1
    cap = cyl_y(-260, Y_FOOT, x, zc, BALL_R_MM).fuse(sphere((x, -260, zc), BALL_R_MM)).fuse(sphere((x, Y_FOOT, zc), BALL_R_MM))
    hit = {n: v for n, v in ((n, ivol(cap, s)) for n, s in zip(names, (p[2] for p in PARTS))) if v > 0.01}
    allowed = ("roller-intake",) if x == 0 else ("roller-intake", "funnel_guide")
    bad = [n for n in hit if not any(a in n for a in allowed)]
    need = [a for a in allowed if not any(a in n for n in hit)]
    a5ok &= not bad and not need
    a5msg.append(f"x={x:+.0f}: hits {sorted(hit)}" + (f" BAD {bad}" if bad else "") + (f" MISSING {need}" if need else ""))
for m in a5msg:
    print("   ", m)
check("A5", a5ok, f"ball swept Y -260 -> {Y_FOOT:.1f} (ramp foot) on X=0,+-50")

# A6 ramp angle from built plate geometry
plate = next(p[2] for p in PARTS if "ramp-plate" in p[1])
big = max(plate.Faces(), key=lambda f: f.Area())
nz = abs(big.normalAt().z)
ang = math.degrees(math.acos(nz))
pc = pinch
check("A6", abs(ang - RAMP_DEG) <= 0.2, f"measured ramp angle {ang:.3f} deg; launch-exit (ball centre at pinch) height {pc.z:.1f} mm = {pc.z / 25.4:.2f} in, Y {pc.y:.1f}")

# A7 BOM sku sources
import json  # noqa: E402
known = dict(CATALOG_URLS, **CONFIRMED_URLS)
if os.path.exists(CATALOG):
    cat = json.load(open(CATALOG))
    cat_urls = {k: v["product_url"] for sec in ("parts", "listings_no_step") for k, v in cat[sec].items()}
    assert all(cat_urls.get(k) == u for k, u in CATALOG_URLS.items()), "embedded catalog URLs drifted"
KIT = "3209-0001-0007"
KIT_SKUS = {"1120-0017-0432_rail", "1120-0010-0264_crossmember", "3625-0001-0104_mecanum", "3625-0100-0104_mecanum",
            "3209-0001-0007_axle", "5203-2402-0019_drive-motor"}
bom = {}
for g, n, s, c in PARTS:
    sku, short = n.split("_")[0], "_".join(n.split("_")[1:-1])
    if f"{sku}_{short}" in KIT_SKUS:
        continue
    key = (sku, short.split("-")[0] if sku not in ("PRINT", "TEAM", "REV") else short)
    bom[key] = bom.get(key, 0) + 1
rows = [(KIT, "Strafer chassis kit (rails, cross A/B, 4 mecanum, 4 drive motors)", 1, known.get(KIT, ""), "")]
for (sku, short), q in sorted(bom.items()):
    if sku == "PRINT":
        rows.append(("3D-PRINT", short, PRINT_QTY[short], "", f"prints/{short}.step|stl"))
    elif sku == "TEAM":
        rows.append(("TEAM", short, q, "", "team-supplied, 50 mm stand-in"))
    elif sku == "REV":
        rows.append(("REV", short, q, "", "approximate envelope"))
    else:
        note = "sold as 2-pack" if sku.startswith("1611") else ""
        rows.append((sku, short, q, known.get(sku, ""), note))
missing = [r[0] for r in rows if r[0] not in ("3D-PRINT", "TEAM", "REV") and r[0] not in known]
check("A7", not missing, f"{len(rows)} BOM rows; skus without catalog/confirmed source: {missing or 'none'}")

print("Belts (GT2 2 mm, 6 mm wide, 20T:20T 1:1):")
for short, sku, L, c, cb in BELTS:
    print(f"   {short}: {sku} {L} mm, modelled centre distance {c:.2f} mm, belt nominal {cb:.2f} mm")
    if abs(c - cb) > 0.6:
        FAIL.append("belt " + short)
        print("FAIL belt centre distance mismatch", short)

if FAIL:
    print("FAILED:", FAIL)
    sys.exit(1)

# ================================================================ EXPORT
os.makedirs(os.path.join(OUT, "prints"), exist_ok=True)
os.makedirs(os.path.join(OUT, "renders"), exist_ok=True)


def assembly(with_balls):
    top = cq.Assembly(name="biobuzz_robot")
    for g in ("drivetrain", "pollen_lane", "intake", "transfer", "launcher", "electronics", "prints"):
        sub = cq.Assembly(name=g)
        for gg, n, s, c in PARTS:
            if gg == g:
                sub.add(s, name=n, color=cq.Color(*c))
        top.add(sub, name=g)
    if with_balls:
        sub = cq.Assembly(name="ref_pollen")
        for n, s in BALLS:
            sub.add(s, name=n, color=cq.Color(*COL["ball"]))
        top.add(sub, name="ref_pollen")
    return top


assembly(False).export(os.path.join(OUT, "robot_assembly.step"))
assembly(True).export(os.path.join(OUT, "robot_assembly_with_pollen.step"))
for n, s in PRINTS.items():
    b = s.BoundingBox()
    s = s.translate(V(-b.xmin, -b.ymin, -b.zmin))   # bed at Z=0, print orientation
    cq.exporters.export(s, os.path.join(OUT, "prints", n + ".step"))
    cq.exporters.export(s, os.path.join(OUT, "prints", n + ".stl"), tolerance=0.05, angularTolerance=0.2)
    print(f"print {n}: {b.xlen:.1f} x {b.ylen:.1f} x {b.zlen:.1f} mm (print orientation), qty {PRINT_QTY[n]}, vol {s.Volume() / 1000:.1f} cm3")

with open(os.path.join(OUT, "bom.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["sku", "name", "qty", "product_url", "note"])
    w.writerows(rows)
skus = sorted({r[0] for r in rows if r[0] not in ("3D-PRINT", "TEAM", "REV")})
with open(os.path.join(OUT, "fetch_gobilda_step.sh"), "w") as f:
    f.write("#!/bin/sh\n# Download real goBILDA STEP zips for every BOM SKU into ./gobilda_step/ (git-ignored).\n"
            "# Note: the Strafer kit zip is large (~157 MB STEP).\nset -e\ncd \"$(dirname \"$0\")\"\nmkdir -p gobilda_step\n"
            f"for sku in {' '.join(skus)}; do\n"
            "  curl -fL --retry 2 -o \"gobilda_step/$sku.zip\" \"https://www.gobilda.com/content/step_files/$sku.zip\" || echo \"failed: $sku\"\n"
            "done\n")
os.chmod(os.path.join(OUT, "fetch_gobilda_step.sh"), 0o755)
with open(os.path.join(OUT, ".gitignore"), "w") as f:
    f.write("gobilda_step/\n")

# ---------------- renders (matplotlib, tessellated, painter's sort)
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
import numpy as np  # noqa: E402

TRIS = []
for g, n, s, c in PARTS + [("ref", n, s, COL["ball"]) for n, s in BALLS]:
    vs, ts = s.tessellate(1.0, 0.5)
    P = np.array([[v.x, v.y, v.z] for v in vs])
    for t in ts:
        TRIS.append((P[list(t)], c))
VIEWS = {"iso": (25, -55), "side": (0, 0), "top": (90, -90), "front": (0, -90)}
L = np.array([0.4, -0.5, 0.75]); L /= np.linalg.norm(L)
for vname, (elev, azim) in VIEWS.items():
    fig = plt.figure(figsize=(9, 9), dpi=110)
    ax = fig.add_subplot(111, projection="3d", proj_type="ortho")
    polys, cols = [], []
    for tri, c in TRIS:
        nrm = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        nn = np.linalg.norm(nrm)
        k = 0.45 + 0.55 * abs(nrm @ L) / nn if nn > 0 else 0.6
        polys.append(tri)
        cols.append((c[0] * k, c[1] * k, c[2] * k))
    ax.add_collection3d(Poly3DCollection(polys, facecolors=cols, edgecolors="none"))
    ax.set_xlim(-230, 230); ax.set_ylim(-230, 230); ax.set_zlim(0, 460)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title(f"BioBuzz robot - {vname} (mm, Z=0 floor, -Y = intake)")
    fig.savefig(os.path.join(OUT, "renders", vname + ".png"), bbox_inches="tight")
    plt.close(fig)
# drill list: shaft bores are not on the 1120 hole grid; drill 14 mm for the 1611 bearings
print("Drill list (14 mm bore for 1611 bearing, both flanges of each upright pair):")
for nm, (y0, z0), y, z in (("upright-fly (1120-0013-0336)", (UP_FLY_Y[0], UP_FLY_Z[0]), Y_FLY, Z_FLY),
                           ("upright-fly (roller B)", (UP_FLY_Y[0], UP_FLY_Z[0]), Y_XB, Z_XB),
                           ("upright-xa (1120-0005-0144)", (UP_XA_Y[0], UP_XA_Z[0]), Y_XA, Z_XA)):
    print(f"   {nm}: {z - z0:.1f} mm above channel bottom end, {y - y0:.1f} mm from its -Y (web) face")
print(f"   intake: 1201 centre bore at Z {Z_INT:.1f} = {Z_INT - sum(FRAME_Z_MM) / 2:.1f} mm above rail mid-height, Y {INTAKE_Y_MM:.1f}")
# handoff gaps (unpowered travel), informational
reach = math.sqrt((BALL_R_MM + ROLLER_D_MM / 2) ** 2 - (Z_INT - BALL_R_MM) ** 2)
corner_y = ramp_pt(S_CORNER, N_BALL)[1]
reach_a = math.sqrt((BALL_R_MM + ROLLER_D_MM / 2) ** 2 - (N_ROLLER - N_BALL) ** 2)
print(f"Handoff: intake roller contact ends at ball Y {INTAKE_Y_MM + reach:.1f}; ramp corner ball at Y {corner_y:.1f} "
      f"({corner_y - INTAKE_Y_MM - reach:.1f} mm unpowered); roller A contact starts at s {S_XA - reach_a:.1f} vs corner s {S_CORNER:.1f} "
      f"({S_XA - reach_a - S_CORNER:.1f} mm unpowered up the ramp)")
print("exported to", OUT)
