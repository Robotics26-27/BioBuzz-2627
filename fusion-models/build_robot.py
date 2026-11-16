#!/usr/bin/env python3
"""BioBuzz 2627 (FTC DECODE-era chassis, BIOBUZZ game) robot CAD generator.

CadQuery 2.8. Builds simplified solids at measured goBILDA sizes/positions,
runs geometric asserts A1..A9 (exit 1 on failure), then exports:
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
import json
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
PLATE_S0_MM = 3.0                # plate lower edge s (keeps underside above floor)
PLATE_T_MM = 1.5
PLATE_W_MM = 88.0                # 1117-0088-0352
PLATE_L_MM = 352.0
LANE_GAP_MM = 78.0               # inner gap between lane walls
LANE_WALL_H_MM = 40.0            # 1117-0040-0352 on edge
LANE_WALL_S0_MM = 55.0           # walls start up the plate so off-centre balls meet the funnel, not the wall end

# Shaft positions are ON real goBILDA holes (A8). Hole rows measured from the 1120 STEP files:
# 14 mm flange holes every 24 mm from the channel end, centred on the 48 mm section; rail-web
# and 1201 4 mm holes on an 8 mm grid -> uprights/1201s sit at Y = multiple of 8.
INTAKE_COMPRESS_MM = 8.0         # squeeze of a floor ball directly under the intake roller
INTAKE_Y_MM, INTAKE_Z_MM = -168.0, 84.0   # 1201 centre on rail inner flange: Y on 8 grid, Z = rail mid + 32
INTAKE_ROLLER_LEN_MM = 38.0      # x +-19 (team roller length)
RHINO_D_MM = 96.0                # 3614-0014-0096
RHINO_W_MM = 32.0
RHINO_X_MM = (27.5, 59.5)        # ball touches the wheels' inner edges (pinch squeeze ~6.9 mm)
HUB_MM = (24.0, 20.0)            # hyper hub square, length
UP_FLY_Y, UP_FLY_Z, FLY_HOLE = (-24.0, 24.0), (76.0, 412.0), 11  # 1120-0013-0336 on rail, centre Y 0
N_FLY_MM = 91.5                  # flywheel axis height above ramp plane: sets ramp position (brackets) -> ~6.3 mm pinch
UP_XA_Y, UP_XA_Z = (-176.0, -128.0), (76.0, 220.0)      # 1120-0005-0144 on rail: intake motor bar + lower ladder spacer
# transfer = tilted "ladder": two 1120-0009-0240 channels parallel to the ramp at x +-(60..108);
# 5 shafts on their own 14 mm holes, 48 mm apart (every 2nd hole), GripForce Gecko 48 mm wheels (compliant)
LADDER_SKU, LADDER_L_MM, LADDER_X_MM = "1120-0009-0240", 240.0, (60.0, 108.0)
LADDER_N_MM, LADDER_S1_MM = 84.6, 99.5       # hole-row height above the ramp, first shaft along the ramp
GECKO_SKU, GECKO_R_MM = "3613-4008-0048", 24.0   # STEP: OD 48 (tread mostly r23.5), 16 wide, 8 mm REX bore
GECKO_TYPES = {"C": [(-8.0, 8.0)], "P": [(-25.0, -9.0), (9.0, 25.0)]}   # centre wheel / wheel pair, alternating

PULLEY_SKU = "3422-4008-0020"   # GT2 2 mm, 20T, 8 mm REX bore, all belts 1:1
PULLEY_PD_MM = 20 * 2.0 / math.pi
PULLEY_OD_MM = PULLEY_PD_MM - 0.51
PULLEY_W_MM = 14.0               # stand-in length (hub + flanges, not measured)
BELT_W_MM = 6.0
BELT_PITCH_LEN_MM = 20 * 2.0     # L = 2*C + 40 for 20T:20T
PLANE_FLY_X_MM = (97.0, 111.0)   # flywheel belt plane
PLANE_CHAIN_X_MM = (64.0, 78.0)   # 20T chain belts inside the ladder channel cavity (left/right alternate)
PLANE_XF_X_MM = (90.0, 104.0)    # transfer motor -> shaft 4 (20T), inside the right ladder cavity
PLANE_IN_X_MM = (-43.0, -29.0)   # intake motor -> intake shaft

MOTOR_D_MM = 37.5
MOTOR6000_L_MM = 107.67          # 5203-2402-0001 face->rear (small_parts.md)
MOTOR435_L_MM = 116.55           # 5203-2402-0014 face->rear
MOTOR_SHAFT_L_MM = 23.5
MOUNT_T_MM = 12.0                # 1201-0043-0002 is 43 x 12 x 43
MOUNT_W_MM = 43.0
# 5203 motor face: 6x M4 threaded holes (4 on a 16 mm square + 2 at +-12 mm) -> M4 screws through the 1201 16 mm pattern

SHAFT_R_MM = 4.0
BEARING_R_MM = 7.5               # 1611 flanged, 15 flange dia x 5
BEARING_T_MM = 5.0

GT2_BELTS_MM = {136: "3423-0006-0136", 184: "3423-0006-0184", 376: "3423-0006-0376"}

# 3D print params
PRINT_WALL_MM = 4.0
M4_CLEAR_MM = 4.3

SIN, COS = math.sin(math.radians(RAMP_DEG)), math.cos(math.radians(RAMP_DEG))
TAN = SIN / COS
Y_FLY, Z_FLY = sum(UP_FLY_Y) / 2, UP_FLY_Z[0] + 24 * FLY_HOLE
RAMP_Y0_MM = Y_FLY + (N_FLY_MM - Z_FLY * COS) / SIN   # ramp floor line, from the flywheel hole + N_FLY

# Parts confirmed on gobilda.com by U2 (not in catalog.json): SKU -> URL (page shows the SKU)
CONFIRMED_URLS = {
    "3423-0006-0136": "https://www.gobilda.com/2mm-pitch-gt2-timing-belt-6mm-width-136mm-pitch-length-68-tooth/",
    "3423-0006-0184": "https://www.gobilda.com/2mm-pitch-gt2-timing-belt-6mm-width-184mm-pitch-length-92-tooth/",
    "3423-0006-0376": "https://www.gobilda.com/2mm-pitch-gt2-timing-belt-6mm-width-376mm-pitch-length-188-tooth/",
    "3613-4008-0048": "https://www.gobilda.com/gripforce-gecko-wheel-8mm-rex-bore-48mm-diameter-30a-durometer/",
}
# Copied from U1 catalog.json (product_url) so A7 runs without the scratch catalog; if
# GOBILDA_CATALOG / ../parts/catalog.json exists, A7 also cross-checks against it.
CATALOG_URLS = {
    "3209-0001-0007": "https://www.gobilda.com/strafer-chassis-kit-104mm-gripforce-mecanum-wheels/",
    "1120-0010-0264": "https://www.gobilda.com/1120-series-u-channel-10-hole-264mm-length/",
    "1120-0005-0144": "https://www.gobilda.com/1120-series-u-channel-5-hole-144mm-length/",
    "1120-0003-0096": "https://www.gobilda.com/1120-series-u-channel-3-hole-96mm-length/",
    "1120-0009-0240": "https://www.gobilda.com/1120-series-u-channel-9-hole-240mm-length/",
    "2102-0008-0220": "https://www.gobilda.com/2102-series-stainless-steel-rex-shaft-8mm-diameter-220mm-length/",
    "3422-4008-0040": "https://www.gobilda.com/2mm-pitch-gt2-pinion-timing-pulley-8mm-rex-bore-40-tooth/",
    "1120-0013-0336": "https://www.gobilda.com/1120-series-u-channel-13-hole-336mm-length/",
    "1201-0043-0002": "https://www.gobilda.com/1201-series-quad-block-pattern-mount-43-2/",
    "1117-0088-0352": "https://www.gobilda.com/clear-polycarbonate-grid-plate-1-5mm-thickness-11-x-44-hole-88-x-352mm/",
    "1117-0040-0352": "https://www.gobilda.com/clear-polycarbonate-grid-plate-1-5mm-thickness-5-x-44-hole-40-x-352mm/",
    "1611-0514-4008": "https://www.gobilda.com/1611-series-flanged-ball-bearing-8mm-rex-id-x-14mm-od-5mm-thickness-2-pack/",
    "2106-4008-3840": "https://www.gobilda.com/8mm-rex-shaft-with-e-clip-stainless-steel-384mm-length/",
    "2102-0008-0260": "https://www.gobilda.com/2102-series-stainless-steel-rex-shaft-8mm-diameter-260mm-length/",
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
    s = outer.cut(box(*cut[0], *cut[1], *cut[2]))
    CH_PENDING[id(s)] = (b, ax, wa, fa, web[0])
    return s


CH_PENDING, CH_META = {}, {}   # channel placement, used by A8 with the measured hole tables


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
    if id(shape) in CH_PENDING:
        CH_META[name] = (sku, CH_PENDING.pop(id(shape)))
    PARTS.append((group, name, shape, COL[col]))
    return name


def add_motor(group, sku, short, length, x_face, y, z, d=1):
    """Motor axis along X, output shaft pointing d*X from face at x_face (face screws: 6x M4 into a 1201)."""
    body = cyl_x(*sorted((x_face - d * length, x_face)), y, z, MOTOR_D_MM / 2)
    body = body.fuse(cyl_x(*sorted((x_face, x_face + d * MOTOR_SHAFT_L_MM)), y, z, SHAFT_R_MM))
    return add(group, sku, short, body, "motor")


def add_pulley(group, short, x0, y, z, teeth=20):
    pd = teeth * 2.0 / math.pi
    sku = {20: PULLEY_SKU, 40: "3422-4008-0040"}[teeth]
    return add(group, sku, short, cyl_x(x0, x0 + PULLEY_W_MM, y, z, (pd - 0.51) / 2), "alu")


def add_belt(group, short, p1, p2, plane, belt_len_mm, teeth=20):
    xm = (plane[0] + plane[1]) / 2
    pd = teeth * 2.0 / math.pi
    shape, c = slot_belt(p1, p2, xm - BELT_W_MM / 2, xm + BELT_W_MM / 2, pd / 2 - 1.0, pd / 2 + 0.6)
    c_belt = (belt_len_mm - 2.0 * teeth) / 2     # equal pulleys: L = 2C + teeth * 2 mm
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

# kit screws/nuts that reach inside the rails (bboxes from the real Strafer STEP, U1 strafer_solids.json,
# kit frame -> +FRAME_DZ_MM); simplified as boxes so A2 sees them
KIT_HW = [
    (-203.0, -173.0, -5.0, -131.0, -163.0, 5.0),
    (-203.0, 163.0, -5.0, -131.0, 173.0, 5.0),
    (-142.0, -139.5, -19.5, -128.0, -132.5, -12.5),
    (-142.0, -139.5, 12.5, -128.0, -132.5, 19.5),
    (-142.0, 132.5, -19.5, -128.0, 139.5, -12.5),
    (-142.0, 132.5, 12.5, -128.0, 139.5, 19.5),
    (-140.0, -171.5, -3.5, -126.0, -164.5, 3.5),
    (-140.0, 164.5, -3.5, -126.0, 171.5, 3.5),
    (-138.5, -19.5, -19.5, -126.5, -12.5, -12.5),
    (-138.5, -19.5, 12.5, -126.5, -12.5, 19.5),
    (-138.5, 12.5, -19.5, -126.5, 19.5, -12.5),
    (-138.5, 12.5, 12.5, -126.5, 19.5, 19.5),
    (-138.5, 148.5, -19.5, -126.5, 155.5, -12.5),
    (-138.5, 148.5, 12.5, -126.5, 155.5, 19.5),
    (-138.5, 180.5, -19.5, -126.5, 187.5, -12.5),
    (-138.5, 180.5, 12.5, -126.5, 187.5, 19.5),
    (-136.0, -175.5, -7.5, -131.0, -160.5, 7.5),
    (-136.0, -172.4, -4.4, -131.0, -163.6, 4.4),
    (-136.0, 160.5, -7.5, -131.0, 175.5, 7.5),
    (-136.0, 163.6, -4.4, -131.0, 172.4, 4.4),
    (-131.6, -174.3, -6.3, -131.4, -161.7, 6.3),
    (-131.6, 161.7, -6.3, -131.4, 174.3, 6.3),
    (-131.0, -172.0, -4.0, -130.0, -164.0, 4.0),
    (-131.0, 164.0, -4.0, -130.0, 172.0, 4.0),
    (-130.8, -15.2, -17.6, -127.0, -13.9, -14.4),
    (-130.8, -15.2, 14.4, -127.0, -13.9, 17.6),
    (-130.8, 16.8, -17.6, -127.0, 18.1, -14.4),
    (-130.8, 16.8, 14.4, -127.0, 18.1, 17.6),
    (-130.8, 152.8, -17.6, -127.0, 154.1, -14.4),
    (-130.8, 152.8, 14.4, -127.0, 154.1, 17.6),
    (-130.8, 184.8, -17.6, -127.0, 186.1, -14.4),
    (-130.8, 184.8, 14.4, -127.0, 186.1, 17.6),
    (-127.5, -28.0, -19.5, -120.5, -14.0, -12.5),
    (-127.5, -28.0, 12.5, -120.5, -14.0, 19.5),
    (-127.5, 14.0, -19.5, -120.5, 28.0, -12.5),
    (-127.5, 14.0, 12.5, -120.5, 28.0, 19.5),
    (-127.5, 140.0, -19.5, -120.5, 154.0, -12.5),
    (-127.5, 140.0, 12.5, -120.5, 154.0, 19.5),
    (-127.5, 182.0, -19.5, -120.5, 196.0, -12.5),
    (-127.5, 182.0, 12.5, -120.5, 196.0, 19.5),
    (-125.6, -19.0, -18.1, -122.4, -15.0, -16.8),
    (-125.6, -19.0, 13.9, -122.4, -15.0, 15.2),
    (-125.6, 15.0, -15.2, -122.4, 19.0, -13.9),
    (-125.6, 15.0, 16.8, -122.4, 19.0, 18.1),
    (-125.6, 149.0, -18.1, -122.4, 153.0, -16.8),
    (-125.6, 149.0, 13.9, -122.4, 153.0, 15.2),
    (-125.6, 183.0, -15.2, -122.4, 187.0, -13.9),
    (-125.6, 183.0, 16.8, -122.4, 187.0, 18.1),
    (120.5, -28.0, -19.5, 127.5, -14.0, -12.5),
    (120.5, -28.0, 12.5, 127.5, -14.0, 19.5),
    (120.5, 14.0, -19.5, 127.5, 28.0, -12.5),
    (120.5, 14.0, 12.5, 127.5, 28.0, 19.5),
    (120.5, 140.0, -19.5, 127.5, 154.0, -12.5),
    (120.5, 140.0, 12.5, 127.5, 154.0, 19.5),
    (120.5, 182.0, -19.5, 127.5, 196.0, -12.5),
    (120.5, 182.0, 12.5, 127.5, 196.0, 19.5),
    (122.4, -19.0, -18.1, 125.6, -15.0, -16.8),
    (122.4, -19.0, 13.9, 125.6, -15.0, 15.2),
    (122.4, 15.0, -15.2, 125.6, 19.0, -13.9),
    (122.4, 15.0, 16.8, 125.6, 19.0, 18.1),
    (122.4, 149.0, -18.1, 125.6, 153.0, -16.8),
    (122.4, 149.0, 13.9, 125.6, 153.0, 15.2),
    (122.4, 183.0, -15.2, 125.6, 187.0, -13.9),
    (122.4, 183.0, 16.8, 125.6, 187.0, 18.1),
    (126.0, -171.5, -3.5, 140.0, -164.5, 3.5),
    (126.0, 164.5, -3.5, 140.0, 171.5, 3.5),
    (126.5, -19.5, -19.5, 138.5, -12.5, -12.5),
    (126.5, -19.5, 12.5, 138.5, -12.5, 19.5),
    (126.5, 12.5, -19.5, 138.5, 19.5, -12.5),
    (126.5, 12.5, 12.5, 138.5, 19.5, 19.5),
    (126.5, 148.5, -19.5, 138.5, 155.5, -12.5),
    (126.5, 148.5, 12.5, 138.5, 155.5, 19.5),
    (126.5, 180.5, -19.5, 138.5, 187.5, -12.5),
    (126.5, 180.5, 12.5, 138.5, 187.5, 19.5),
    (127.0, -18.1, -17.6, 130.8, -16.8, -14.4),
    (127.0, -18.1, 14.4, 130.8, -16.8, 17.6),
    (127.0, 13.9, -17.6, 130.8, 15.2, -14.4),
    (127.0, 13.9, 14.4, 130.8, 15.2, 17.6),
    (127.0, 149.9, -17.6, 130.8, 151.2, -14.4),
    (127.0, 149.9, 14.4, 130.8, 151.2, 17.6),
    (127.0, 181.9, -17.6, 130.8, 183.2, -14.4),
    (127.0, 181.9, 14.4, 130.8, 183.2, 17.6),
    (128.0, -139.5, -19.5, 142.0, -132.5, -12.5),
    (128.0, -139.5, 12.5, 142.0, -132.5, 19.5),
    (128.0, 132.5, -19.5, 142.0, 139.5, -12.5),
    (128.0, 132.5, 12.5, 142.0, 139.5, 19.5),
    (130.0, -172.0, -4.0, 131.0, -164.0, 4.0),
    (130.0, 164.0, -4.0, 131.0, 172.0, 4.0),
    (131.0, -175.5, -7.5, 136.0, -160.5, 7.5),
    (131.0, -173.0, -5.0, 203.0, -163.0, 5.0),
    (131.0, -172.4, -4.4, 136.0, -163.6, 4.4),
    (131.0, 160.5, -7.5, 136.0, 175.5, 7.5),
    (131.0, 163.0, -5.0, 203.0, 173.0, 5.0),
    (131.0, 163.6, -4.4, 136.0, 172.4, 4.4),
    (131.4, -174.3, -6.3, 131.6, -161.7, 6.3),
    (131.4, 161.7, -6.3, 131.6, 174.3, 6.3),
]
for b in KIT_HW:
    add("drivetrain", "3209-0001-0007", "kit-hw", box(b[0], b[3], b[1], b[4], b[2] + FRAME_DZ_MM, b[5] + FRAME_DZ_MM), "steel")

# ---------------- pollen lane (ramp plate + side walls)
s1 = PLATE_S0_MM
plate_local = box(-PLATE_W_MM / 2, PLATE_W_MM / 2, s1, s1 + PLATE_L_MM, -PLATE_T_MM, 0)
add("pollen_lane", "1117-0088-0352", "ramp-plate", on_ramp(plate_local), "poly")  # screws to both ramp_brackets
gx = LANE_GAP_MM / 2
for sx in (-1, 1):
    xw = sorted((sx * gx, sx * (gx + PLATE_T_MM)))
    w = box(*xw, LANE_WALL_S0_MM, LANE_WALL_S0_MM + PLATE_L_MM, 0, LANE_WALL_H_MM)
    add("pollen_lane", "1117-0040-0352", "lane-wall", on_ramp(w), "poly")  # held to plate by wall_clips

# ---------------- reference geometry (ramp coords)
N_BALL = BALL_R_MM + BALL_GAP_MM
S_CORNER = (BALL_R_MM + 0.1 - N_BALL * COS) / SIN      # ball touching floor and ramp
S_PINCH = (Y_FLY - RAMP_Y0_MM) * COS + Z_FLY * SIN      # flywheel axis foot on the ramp


def to_sn(y, z):
    return (y - RAMP_Y0_MM) * COS + z * SIN, -(y - RAMP_Y0_MM) * SIN + z * COS


def on_ramp_pt(s, n):
    return ramp_pt(s, n)[1:]


# ---------------- launcher: flywheel shaft in hole #11 of the 336 uprights
for sx in (-1, 1):
    xr = sorted((sx * RAIL_X_MM[0], sx * RAIL_X_MM[1]))
    # flywheel upright: stands on rail top, 1201 in its lower end bolts to the rail web; flanges carry bearings
    add("launcher", "1120-0013-0336", "upright-fly", uchannel(xr, UP_FLY_Y, UP_FLY_Z, "z", "-y"), "alu")
    add("launcher", "1201-0043-0002", "mount-upright-fly",
        box(xr[0] + WALL_MM, xr[1] - WALL_MM, UP_FLY_Y[0] + WALL_MM, UP_FLY_Y[0] + WALL_MM + MOUNT_W_MM, ZT, ZT + MOUNT_T_MM), "alu")
    add_bearings("launcher", "bearing-fly", [sx * (RAIL_X_MM[0] + WALL_MM / 2), sx * (RAIL_X_MM[1] - WALL_MM / 2)], Y_FLY, Z_FLY)
add("launcher", "2106-4008-3840", "shaft-fly", cyl_x(-192, 192, Y_FLY, Z_FLY, SHAFT_R_MM), "steel")
for sx in (-1, 1):
    x0, x1 = sorted((sx * RHINO_X_MM[0], sx * RHINO_X_MM[1]))
    wheel = cyl_x(x0, x1, Y_FLY, Z_FLY, RHINO_D_MM / 2).cut(cyl_x(x0 - 1, x1 + 1, Y_FLY, Z_FLY, 7.0))
    add("launcher", "3614-0014-0096", "rhino-fly", wheel, "rhino")  # screwed to hyper hub
    h0, h1 = sorted((sx * RHINO_X_MM[1], sx * (RHINO_X_MM[1] + HUB_MM[1])))
    add("launcher", "1310-0016-4008", "hub-fly", box(h0, h1, Y_FLY - 12, Y_FLY + 12, Z_FLY - 12, Z_FLY + 12), "alu")  # on shaft
add_pulley("launcher", "pulley-fly", PLANE_FLY_X_MM[0], Y_FLY, Z_FLY)
# F4 cross brace: 1120-0013-0336 across both upright front webs, rear flange bolted to each web (A8 checks
# the 4 mm holes line up), above the flywheel and in front of the launch exit
BRACE = ((-164.0, 172.0), (UP_FLY_Y[0] - CH_MM, UP_FLY_Y[0]), (388.0, 436.0))
add("launcher", "1120-0013-0336", "brace-fly", uchannel(*BRACE, "x", "+z"), "alu")

# ---------------- transfer: tilted ladder (two 1120 channels parallel to the ramp, web toward the ramp, open
# side up so belts can leave the cavity). Each ladder bolts at both ends through a printed spacer to the
# inner flange of an upright (A-upright at the low end, flywheel upright at the high end).
for sx in (-1, 1):
    xr = sorted((sx * RAIL_X_MM[0], sx * RAIL_X_MM[1]))
    add("transfer", "1120-0005-0144", "upright-xa", uchannel(xr, UP_XA_Y, UP_XA_Z, "z", "-y"), "alu")  # on rail, 1201 in end
    add("transfer", "1201-0043-0002", "mount-upright-xa",
        box(xr[0] + WALL_MM, xr[1] - WALL_MM, UP_XA_Y[0] + WALL_MM, UP_XA_Y[0] + WALL_MM + MOUNT_W_MM, ZT, ZT + MOUNT_T_MM), "alu")
LAD_S0 = LADDER_S1_MM - 24.0                  # shaft 1 sits in hole #1
S_SHAFTS = [LADDER_S1_MM + 48.0 * k for k in range(5)]
for sx in (-1, 1):
    xl = sorted((sx * LADDER_X_MM[0], sx * LADDER_X_MM[1]))
    lad = uchannel(xl, (LAD_S0, LAD_S0 + LADDER_L_MM), (LADDER_N_MM - 24, LADDER_N_MM + 24), "y", "-z")
    meta = CH_PENDING.pop(id(lad))
    lad_w = on_ramp(lad)
    CH_PENDING[id(lad_w)] = meta + ("ramp",)
    add("transfer", LADDER_SKU, "ladder", lad_w, "alu")
SHAFT_YZ = [on_ramp_pt(s, LADDER_N_MM) for s in S_SHAFTS]
for k, (y, z) in enumerate(SHAFT_YZ, 1):
    add("transfer", "2102-0008-0220", f"shaft-x{k}", cyl_x(-110, 110, y, z, SHAFT_R_MM), "steel")   # collars not modelled
    for sx in (-1, 1):
        add_bearings("transfer", f"bearing-x{k}", [sx * (LADDER_X_MM[0] + WALL_MM / 2), sx * (LADDER_X_MM[1] - WALL_MM / 2)], y, z)
    for x0, x1 in GECKO_TYPES["C" if k % 2 else "P"]:
        add("transfer", GECKO_SKU, f"gecko-x{k}", cyl_x(x0, x1, y, z, GECKO_R_MM), "team")   # REX bore on the shaft
# 20T chain inside the ladder cavities: 1-2 and 3-4 left, 2-3 and 4-5 right, C = 48 exactly (136 belts)
for k in range(1, 5):
    sx = -1 if k % 2 else 1
    x0 = PLANE_CHAIN_X_MM[0] if sx > 0 else -PLANE_CHAIN_X_MM[1]
    for j in (k, k + 1):
        add_pulley("transfer", f"pulley-x{j}", x0, *SHAFT_YZ[j - 1])
    add_belt("transfer", f"belt-x{k}{k + 1}", SHAFT_YZ[k - 1], SHAFT_YZ[k], (x0, x0 + PULLEY_W_MM), 136)
# transfer motor: 1201 bolted by its rear edge to the brace's front flange, motor points +X, GT2 376 to shaft 4
Y_MX, Z_MX = BRACE[1][0] - MOUNT_W_MM / 2, sum(BRACE[2]) / 2
X_MX = PLANE_XF_X_MM[0] - 14.0               # motor face; 1201 between face and pulley
add("transfer", "1201-0043-0002", "mount-motor-xfer", box(X_MX, X_MX + MOUNT_T_MM, BRACE[1][0] - MOUNT_W_MM, BRACE[1][0], Z_MX - MOUNT_W_MM / 2, Z_MX + MOUNT_W_MM / 2), "alu")
add_motor("transfer", "5203-2402-0014", "motor-xfer", MOTOR435_L_MM, X_MX, Y_MX, Z_MX)
add_pulley("transfer", "pulley-motor-xfer", PLANE_XF_X_MM[0], Y_MX, Z_MX)
add_pulley("transfer", "pulley-x4", PLANE_XF_X_MM[0], *SHAFT_YZ[3])
add_belt("transfer", "belt-xfer", SHAFT_YZ[3], (Y_MX, Z_MX), PLANE_XF_X_MM, 376)

# flywheel motor: 1201 bolted by its front edge to the brace's rear flange (free between the uprights),
# motor points +X, GT2 184 straight down to the flywheel shaft
Y_MFLY, Z_MFLY = BRACE[1][1] + MOUNT_W_MM / 2, sum(BRACE[2]) / 2
add("launcher", "1201-0043-0002", "mount-motor-fly", box(84, 96, BRACE[1][1], BRACE[1][1] + MOUNT_W_MM, Z_MFLY - MOUNT_W_MM / 2, Z_MFLY + MOUNT_W_MM / 2), "alu")
add_motor("launcher", "5203-2402-0001", "motor-fly", MOTOR6000_L_MM, 84.0, Y_MFLY, Z_MFLY)
add_pulley("launcher", "pulley-motor-fly", PLANE_FLY_X_MM[0], Y_MFLY, Z_MFLY)
add_belt("launcher", "belt-fly", (Y_FLY, Z_FLY), (Y_MFLY, Z_MFLY), PLANE_FLY_X_MM, 184)

# ---------------- intake: shaft in 1201 bores bolted flat to the rail inner flanges
Z_INT, ZC_FLOOR = INTAKE_Z_MM, BALL_R_MM + 0.1
INTAKE_R_MM = INTAKE_COMPRESS_MM + (Z_INT - ZC_FLOOR) - BALL_R_MM     # solved: 8 mm squeeze at the on-grid axis
assert 15.0 <= INTAKE_R_MM <= 35.0, INTAKE_R_MM
for sx in (-1, 1):
    xm = sorted((sx * (RAIL_X_MM[0] - MOUNT_T_MM), sx * RAIL_X_MM[0]))
    add("intake", "1201-0043-0002", "mount-intake", box(*xm, INTAKE_Y_MM - MOUNT_W_MM / 2, INTAKE_Y_MM + MOUNT_W_MM / 2, Z_INT - MOUNT_W_MM / 2, Z_INT + MOUNT_W_MM / 2), "alu")
    add_bearings("intake", "bearing-intake", [sx * (RAIL_X_MM[0] - MOUNT_T_MM / 2)], INTAKE_Y_MM, Z_INT)
add("intake", "2102-0008-0260", "shaft-intake", cyl_x(-130, 130, INTAKE_Y_MM, Z_INT, SHAFT_R_MM), "steel")
add("intake", "TEAM", "existing_band_roller-intake", cyl_x(-INTAKE_ROLLER_LEN_MM / 2, INTAKE_ROLLER_LEN_MM / 2, INTAKE_Y_MM, Z_INT, INTAKE_R_MM), "team")
add_pulley("intake", "pulley-intake", PLANE_IN_X_MM[0], INTAKE_Y_MM, Z_INT)
# intake motor: left cantilever 144 channel on the front web of the left A-upright, 1201 on top, motor points +X
BAR_IN = ((-188.0, -44.0), (UP_XA_Y[0] - CH_MM, UP_XA_Y[0]), (180.0, 228.0))
add("intake", "1120-0005-0144", "bar-intake", uchannel(*BAR_IN, "x", "+z"), "alu")
Y_MI, Z_MI = sum(BAR_IN[1]) / 2, BAR_IN[2][1] + MOUNT_W_MM / 2
add("intake", "1201-0043-0002", "mount-motor-intake", box(-56, -44, Y_MI - MOUNT_W_MM / 2, Y_MI + MOUNT_W_MM / 2, BAR_IN[2][1], BAR_IN[2][1] + MOUNT_W_MM), "alu")
add_motor("intake", "5203-2402-0014", "motor-intake", MOTOR435_L_MM, -56.0, Y_MI, Z_MI)
add_pulley("intake", "pulley-motor-intake", PLANE_IN_X_MM[0], Y_MI, Z_MI)
add_belt("intake", "belt-intake", (INTAKE_Y_MM, Z_INT), (Y_MI, Z_MI), PLANE_IN_X_MM, 376)

# ---------------- ramp support structure (cross member C, post, electronics deck)
DECK_Z = (ZT, ZT + PLATE_T_MM)
POST_Z = (DECK_Z[1], DECK_Z[1] + 96.0)
POST_Y = (48.0, 96.0)                        # cross member C + post on the 8 mm rail grid
BRACKET_GAP = ramp_under_z(POST_Y[0]) - POST_Z[1]   # same wedge on both supports
Y_BRK_LO = RAMP_Y0_MM + (ZT + BRACKET_GAP + PLATE_T_MM / COS) / TAN   # lower wedge front edge on cross member A
assert -24.0 <= Y_BRK_LO <= -24.0 + 6.0, Y_BRK_LO   # <= 6 mm overhang off the rear of cross member A
add("pollen_lane", "1120-0010-0264", "crossmember-c", uchannel((-132, 132), POST_Y, (ZB, ZT), "x", "+z"), "alu")
for sx in (-1, 1):
    add("pollen_lane", "1201-0043-0002", "mount-cross-c", box(*sorted((sx * 120.0, sx * 132.0)), POST_Y[0] + WALL_MM, POST_Y[1] - WALL_MM, ZT - WALL_MM - MOUNT_W_MM, ZT - WALL_MM), "alu")
DECK_Y = (40.0, 128.0)
add("electronics", "1117-0088-0352", "deck", box(-176, 176, *DECK_Y, *DECK_Z), "poly")   # screwed to rails + cross member C
add("pollen_lane", "1120-0003-0096", "post", uchannel((-24, 24), POST_Y, POST_Z, "z", "+y"), "alu")  # 1201 bolts through deck into C
for z0 in (POST_Z[0], POST_Z[1] - MOUNT_T_MM):
    add("pollen_lane", "1201-0043-0002", "mount-post", box(-21.5, 21.5, POST_Y[0] + 0.5, POST_Y[0] + 0.5 + MOUNT_W_MM, z0, z0 + MOUNT_T_MM), "alu")


# measured hole tables (U2 measure_holes.py on the real goBILDA STEP files)
HOLES = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "hole_tables.json")))


def world_holes(name):
    """[(world axis index or -1 for a tilted web, dia, (x, y, z) on the wall mid-plane)] for a channel instance"""
    sku, meta = CH_META[name]
    b, la, wa, fa, sign = meta[:5]
    tilted = len(meta) > 5
    out = []
    for ax, dia, lx, ly, lz in HOLES[sku]["holes"]:
        w = [0.0, 0.0, 0.0]
        w[la] = b[la][0] + ly
        pts = []
        if ax == "x":          # flange hole: through both flanges, (ly, lz) in the flange plane
            w[wa] = (b[wa][1] - 2.5 + lz) if sign == "+" else (b[wa][0] + 2.5 - lz)
            for plane in (b[fa][0] + WALL_MM / 2, b[fa][1] - WALL_MM / 2):
                w[fa] = plane
                pts.append((fa, tuple(w)))
        elif ax == "z":        # web hole
            w[fa] = (b[fa][0] + b[fa][1]) / 2 + lx
            w[wa] = (b[wa][1] - WALL_MM / 2) if sign == "+" else (b[wa][0] + WALL_MM / 2)
            pts.append((wa, tuple(w)))
        for axis, q in pts:
            if tilted:         # local (x, s, n) -> world; only X-normal walls stay axis-aligned
                q = ramp_pt(q[1], q[2], q[0])
                axis = 0 if axis == 0 else -1
            out.append((axis, dia, q))
    return out


def ladder_spacer(lo):
    """Printed 24 mm spacer between the right ladder's outer flange (x=108) and an upright's inner flange
    (x=132); blind M4 holes at real holes of both flanges, clearance for the ladder shaft ends."""
    lad = f"{LADDER_SKU}_ladder_2"
    up = "1120-0005-0144_upright-xa_2" if lo else "1120-0013-0336_upright-fly_2"
    x0, x1 = LADDER_X_MM[1], RAIL_X_MM[0]
    s_a, s_b = (LAD_S0 + 2, LAD_S0 + 60) if lo else (LAD_S0 + LADDER_L_MM - 60, LAD_S0 + LADDER_L_MM - 2)
    strip = on_ramp(box(x0, x1, s_a, s_b, LADDER_N_MM - 21, LADDER_N_MM + 21))
    ub = next(q[2] for q in PARTS if q[1] == up).BoundingBox()
    body = strip.intersect(box(x0, x1, ub.ymin + 3, ub.ymax - 3, ub.zmin + 3, ub.zmax - 3))
    inside = lambda y, z: all(body.isInside(V((x0 + x1) / 2, y + dy, z + dz)) for dy, dz in ((4, 4), (-4, 4), (4, -4), (-4, -4)))  # noqa: E731
    lh = [w for ax, dia, w in world_holes(lad) if ax == 0 and dia == 4.0 and abs(w[0] - (x0 - WALL_MM / 2)) < 0.01 and inside(w[1], w[2])]
    uh = [w for ax, dia, w in world_holes(up) if ax == 0 and dia == 4.0 and abs(w[0] - (x1 + WALL_MM / 2)) < 0.01 and inside(w[1], w[2])]
    for y, z in SHAFT_YZ:
        if body.BoundingBox().ymin - 9 < y < body.BoundingBox().ymax + 9 and body.BoundingBox().zmin - 9 < z < body.BoundingBox().zmax + 9:
            body = body.cut(cyl_x(x0 - 1, x1 + 1, y, z, 9.0))
    lh = [w for w in lh if all(math.hypot(w[1] - y, w[2] - z) > 13 for y, z in SHAFT_YZ)]
    uh = [w for w in uh if all(math.hypot(w[1] - y, w[2] - z) > 13 for y, z in SHAFT_YZ)]
    for w in lh:
        body = body.cut(cyl_x(x0 - 1, x0 + 12, w[1], w[2], M4_CLEAR_MM / 2))
    for w in uh:
        body = body.cut(cyl_x(x1 - 12, x1 + 1, w[1], w[2], M4_CLEAR_MM / 2))
    SPACER_HOLES["lower" if lo else "upper"] = (len(lh), len(uh))
    return body


SPACER_HOLES = {}


# ---------------- 3D prints (built flat on the bed, then placed)
LO_HOLES = (0.0 - 16 - Y_BRK_LO, 0.0 + 8 - Y_BRK_LO)   # cross member A web holes at y = -16, +8 (post-top 1201: local 8 / 32)


def ramp_bracket_print():
    """Open-frame wedge (F6: < 50 cm3): 6 mm base plate on a channel web / 1201, 5 mm top plate under
    the ramp, one 4 mm side web with a lightening window. Print frame: X = y (0..48), Y = z, Z = x
    (0..48); the side web lies on the bed, base/top plates stand vertical -> no overhangs.
    Base: M4 slots at x = +-16 covering the cross-member-A holes and the post-top 1201 holes (screw
    heads on the open top of the base, fitted before the plate). Top: M4 slots normal to the ramp at
    plate holes x = +-8."""
    g, L, W = BRACKET_GAP, CH_MM, CH_MM
    top = g + L * TAN
    side = poly_extrude([(0, 0), (L, 0), (L, top), (0, g)], 0, 4)
    side = side.cut(poly_extrude([(8, 12), (L - 6, 12), (L - 6, top - 14), (8, g + 8 * TAN - 14)], -1, 5))
    base = box(0, L, 0, 6, 0, W)
    t_n = 5 / COS
    plate = poly_extrude([(0, g), (L, top), (L, top - t_n), (0, g - t_n)], 0, W)
    br = side.fuse(base).fuse(plate).clean()
    for zx in (W / 2 - 16, W / 2 + 16):
        for ya, yb in zip(LO_HOLES, (8.0, 32.0)):
            for yl in (ya, yb, (ya + yb) / 2):
                br = br.cut(cq.Solid.makeCylinder(M4_CLEAR_MM / 2, 20, V(yl, -1, zx), V(0, 1, 0)))
    for zx in (W / 2 - 8, W / 2 + 8):
        p = V(32.0, g + 32.0 * TAN, zx)
        d = V(SIN, -COS, 0)
        for k in range(-3, 4):
            br = br.cut(cq.Solid.makeCylinder(M4_CLEAR_MM / 2, 12, p + V(COS, SIN, 0) * (2.0 * k) - d * 2, d))
    return br


def place_bracket(shape, y0, z0):
    # print frame (X=y, Y=z, Z=x) -> world: X_w = Z_p - 24, Y_w = X_p + y0, Z_w = Y_p + z0
    m = cq.Matrix([[0, 0, 1, -CH_MM / 2], [1, 0, 0, y0], [0, 1, 0, z0]])
    return shape.transformGeometry(m)


def funnel_print(side):
    """Plan polygon extruded in Z (bed = bottom face). Guide wall from (100,-216) to (42, ramp foot),
    front web to the rail, flange screwed to rail inner flange (2x M4 along X)."""
    p0, p1 = (100.0, -216.0), (42.0, RAMP_Y0_MM - (BALL_R_MM + 1.0 - BALL_R_MM * COS) / SIN)  # ends at the ramp foot
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    ln = math.hypot(dx, dy)
    nx, ny = dy / ln, -dx / ln            # outward (+x) normal
    t = PRINT_WALL_MM
    wall = [p0, p1, (p1[0] + nx * t, p1[1] + ny * t), (p0[0] + nx * t, max(-216.0, p0[1] + ny * t))]
    z0, z1 = 12.0, 60.0                 # symmetric about the M4 row z=36 -> one print serves both sides (flipped)
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


SP_LO, SP_HI = ladder_spacer(True), ladder_spacer(False)
_flat = lambda s: s.rotate(V(0, 0, 0), V(0, 1, 0), 90)  # noqa: E731   spacer: x-thickness -> print Z
PRINTS = {"ramp_bracket": ramp_bracket_print(), "funnel_guide": funnel_print(1), "wall_clip": wall_clip_print(),
          "ladder_spacer_lower": _flat(SP_LO), "ladder_spacer_upper": _flat(SP_HI)}
PRINT_QTY = {"ramp_bracket": 2, "funnel_guide": 2, "wall_clip": 4, "ladder_spacer_lower": 2, "ladder_spacer_upper": 2}
add("prints", "PRINT", "ramp_bracket", place_bracket(PRINTS["ramp_bracket"], Y_BRK_LO, ZT), "print")   # on cross member A web
add("prints", "PRINT", "ramp_bracket", place_bracket(PRINTS["ramp_bracket"], POST_Y[0], POST_Z[1]), "print")  # on post-top 1201
add("prints", "PRINT", "funnel_guide", funnel_print(-1), "print")    # screwed to left rail inner flange (same print, flipped)
add("prints", "PRINT", "funnel_guide", funnel_print(1), "print")     # screwed to right rail inner flange
for sp in (SP_LO, SP_HI):   # ladder outer flange <-> upright inner flange, both sides (left = same print flipped)
    nm = "ladder_spacer_lower" if sp is SP_LO else "ladder_spacer_upper"
    add("prints", "PRINT", nm, sp, "print")
    add("prints", "PRINT", nm, sp.mirror("YZ"), "print")
for s0 in (60.0, 250.0):
    for side in (-1, 1):
        add("prints", "PRINT", "wall_clip", place_clip(PRINTS["wall_clip"], side, s0), "print")  # wall + plate

# ---------------- electronics envelopes (approximate boxes)
# battery standing on the deck in front of the post, strapped to deck (strap not modelled)
add("electronics", "REV", "battery_envelope", box(-60, 75, 100, 125, DECK_Z[1], DECK_Z[1] + 75), "env")
# hubs on edge on the deck, left side (bracket/velcro to deck not modelled); USB/ports face up
add("electronics", "REV", "control_hub_envelope", box(-127.5, -98, 40, 183, DECK_Z[1], DECK_Z[1] + 103), "env")
add("electronics", "REV", "expansion_hub_envelope", box(-95, -65.5, 40, 183, DECK_Z[1], DECK_Z[1] + 103), "env")

# ---------------- squeeze model: squeeze = ball radius - distance(ball centre, part solid)
from OCP.BRepExtrema import BRepExtrema_DistShapeShape  # noqa: E402
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex  # noqa: E402
from OCP.gp import gp_Pnt  # noqa: E402
names = [p[1] for p in PARTS]
# Gecko tread is mostly r23.5 (measured): squeeze uses that radius, interference uses the 48 mm OD
POWERED = [(p[1], p[2]) for p in PARTS if any(k in p[1] for k in ("roller-intake", "rhino-fly"))] + \
          [(p[1], cyl_x(p[2].BoundingBox().xmin, p[2].BoundingBox().xmax, (p[2].BoundingBox().ymin + p[2].BoundingBox().ymax) / 2,
                        (p[2].BoundingBox().zmin + p[2].BoundingBox().zmax) / 2, 23.5)) for p in PARTS if "gecko" in p[1]]


def squeezes(pt, parts=POWERED):
    v = BRepBuilderAPI_MakeVertex(gp_Pnt(*pt)).Vertex()
    return [(BALL_R_MM - BRepExtrema_DistShapeShape(v, s.wrapped).Value(), n) for n, s in parts]


def squeeze(pt):
    return max(squeezes(pt))


def contacts_ok(pt):
    """every compliant contact is either absent (<= 0) or a real squeeze of 3..10 mm"""
    return all(q <= 0 or 3.0 <= q <= 10.0 for q, _ in squeezes(pt))


# ---------------- reference pollen: stored balls at a common pitch below the pinch, chosen so every
# compliant contact is 3..10 mm (A3) and there is no room for a 5th ball; intake ball on the floor
def clean(s):
    """placement uses only the powered/compliant contacts; rigid contacts are reported by A3, not filtered"""
    pt = ramp_pt(s, N_BALL)
    return contacts_ok(pt) and squeeze(pt)[0] >= 2.0


GRID = [S_CORNER + 0.5 * i for i in range(int((S_PINCH - S_CORNER) / 0.5) + 1)]
CLEAN = {round(s, 2): clean(s) for s in GRID}   # evaluated once; search below is pure lookup


def place(k, upper):
    """highest clean centre for stored ball k (1-based) below `upper`, recursing downwards"""
    for s in sorted((g for g in GRID if upper - 2 * BALL_D_MM + 0.05 < g <= upper - BALL_D_MM - 0.05), reverse=True):
        if not CLEAN[round(s, 2)]:
            continue
        if k == 1:
            if s < S_CORNER + BALL_D_MM:
                return [s]
            continue
        rest = place(k - 1, s)
        if rest:
            return rest + [s]
    return None


sb = place(4, S_PINCH)
PLACE_OK = bool(sb)   # reported by A3; fall back to a plain pitch so the other asserts still run
sb = sb or [S_PINCH - k * (BALL_D_MM + 0.5) for k in (4, 3, 2, 1)]
pick = (min(b - a for a, b in zip(sb, sb[1:] + [S_PINCH])), sb)
BALL_PITCH_MM, S_BALLS = pick
b1 = V(*ramp_pt(S_BALLS[0], N_BALL))
# with 4 stored (G407 max) the next ball waits at the intake: nearest floor spot >= 71 mm from ball 1
# with clean contacts (it is then just outside the roller's reach, i.e. not gripped)
Y_REF_IN = next(y for y in [INTAKE_Y_MM - 0.5 * k for k in range(120)]
                if contacts_ok((0, y, ZC_FLOOR)) and (b1 - V(0, y, ZC_FLOOR)).Length >= BALL_D_MM + 0.05)
BALLS = [("ref_intake", sphere((0, Y_REF_IN, ZC_FLOOR), BALL_R_MM))]
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
    ("*kit-hw*", "3209-0001-0007_*", "kit screws/nuts/axle hardware within the Strafer kit itself"),
    ("*kit-hw*", "*_rail_*", "kit screws through the rail flanges"),
    ("*kit-hw*", "*_crossmember_*", "kit screws through the kit cross members"),
    ("*kit-hw*", "*mecanum*", "kit wheel hardware"),
    ("*kit-hw*", "*drive-motor*", "kit motor mount screws"),
    ("*shaft-fly*", "*bearing-fly*", "shaft runs in flanged bearing"),
    ("*shaft-fly*", "*upright-fly*", "shaft passes through 14 mm channel hole #11"),
    ("*bearing-fly*", "*upright-fly*", "bearing seated in 14 mm channel hole"),
    ("*shaft-fly*", "*hub-fly*", "hyper hub on REX shaft"),
    ("*shaft-fly*", "*pulley-fly*", "pulley on shaft"),
    ("*hub-fly*", "*rhino-fly*", "hub spigot in Rhino 14 mm bore"),
    ("*shaft-x?_*", "*bearing-x*", "shaft runs in flanged bearing"),
    ("*shaft-x?_*", "*_ladder_*", "shaft passes through the ladder's 14 mm hole"),
    ("*bearing-x*", "*_ladder_*", "bearing seated in the ladder's 14 mm hole"),
    ("*shaft-x?_*", "*gecko-x*", "Gecko wheel on REX shaft"),
    ("*shaft-x?_*", "*pulley-x*", "pulley on shaft"),
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

# A3 ball path: squeeze = r - distance(centre, solid); compliant parts may squeeze 3..10 mm, rigid parts 0
COMPLIANT = [(p[1], p[2]) for p in PARTS if any(k in p[1] for k in ("existing_band_roller", "rhino", "gecko"))]
RIGID = [(p[1], p[2]) for p in PARTS if not any(k in p[1] for k in ("existing_band_roller", "rhino", "gecko"))]
a3 = []
for bn, bs in BALLS:
    c = bs.Center()
    for q, pn in squeezes((c.x, c.y, c.z), COMPLIANT):
        if q > 0:
            a3.append((3.0 <= q <= 10.0, f"{bn} squeezed by {pn} depth {q:.2f} mm"))
    for pn, ps in RIGID:
        v = ivol(bs, ps)
        if v > 0.01:
            a3.append((False, f"{bn} hits {pn} {v:.2f} mm3"))
for i in range(len(BALLS)):
    for j in range(i + 1, len(BALLS)):
        v = ivol(BALLS[i][1], BALLS[j][1])
        if v > 0.01:
            a3.append((False, f"{BALLS[i][0]} overlaps {BALLS[j][0]} {v:.2f} mm3"))
for ok, m in a3:
    print("   ", "ok " if ok else "BAD", m)
occupied = S_BALLS + [S_PINCH]
fifth = [s / 2 for s in range(int(S_CORNER * 2), int(S_PINCH * 2))
         if all(abs(s / 2 - o) >= BALL_D_MM for o in occupied)]
check("A3", PLACE_OK and all(ok for ok, _ in a3) and not fifth,
      f"balls on ramp at s = {', '.join('%.1f' % s for s in occupied)} (min pitch {BALL_PITCH_MM:.2f}, pinch last); "
      f"{len(a3)} contacts; clean 4-ball placement {'found' if PLACE_OK else 'NOT FOUND'}; free slot for a 5th ball between corner s={S_CORNER:.1f} and pinch: {'NONE' if not fifth else fifth[:3]}")

# A8 holes, independent of the layout constants: hole_tables.json was measured from the real goBILDA
# 1120 STEP files (cylindrical faces, U2 measure_holes.py). Each channel instance maps its measured local
# holes to world; every shaft must sit on a 14 mm hole of every channel wall it passes, and every
# channel-to-channel bolted joint must have >= 2 coincident 4 mm holes.
a8, a8msg = [], []
for p_ in PARTS:
    if "_shaft-" not in p_[1]:
        continue
    sb_ = p_[2].BoundingBox()
    sy, sz = (sb_.ymin + sb_.ymax) / 2, (sb_.zmin + sb_.zmax) / 2
    for cn in CH_META:
        cb = next(q[2] for q in PARTS if q[1] == cn).BoundingBox()
        walls = {w[0] for ax, dia, w in world_holes(cn) if ax == 0}          # flange planes normal to X
        crossed = [x for x in walls if sb_.xmin <= x <= sb_.xmax] if (cb.ymin - 4 <= sy <= cb.ymax + 4 and cb.zmin - 4 <= sz <= cb.zmax + 4) else []
        for x in crossed:
            d = min((math.hypot(w[1] - sy, w[2] - sz), i) for i, (ax, dia, w) in enumerate(world_holes(cn)) if ax == 0 and dia == 14.0 and abs(w[0] - x) < 0.01)
            a8.append(d[0] <= 0.2)
            a8msg.append(f"{p_[1].split('_')[1]} through {cn} wall x={x:.1f}: nearest 14 mm hole {d[0]:.3f} mm")
JOINTS = [("1120-0005-0144_bar-intake_1", "1120-0005-0144_upright-xa_1"),
          ("1120-0013-0336_brace-fly_1", "1120-0013-0336_upright-fly_1"), ("1120-0013-0336_brace-fly_1", "1120-0013-0336_upright-fly_2")]
for na, nb in JOINTS:
    ba, bb_ = (next(q[2] for q in PARTS if q[1] == n).BoundingBox() for n in (na, nb))
    lo_a, hi_a = (ba.xmin, ba.ymin, ba.zmin), (ba.xmax, ba.ymax, ba.zmax)
    lo_b, hi_b = (bb_.xmin, bb_.ymin, bb_.zmin), (bb_.xmax, bb_.ymax, bb_.zmax)
    ax_c = next(i for i in range(3) if abs(hi_a[i] - lo_b[i]) < 0.01 or abs(hi_b[i] - lo_a[i]) < 0.01)
    plane = hi_a[ax_c] if abs(hi_a[ax_c] - lo_b[ax_c]) < 0.01 else lo_a[ax_c]
    ha = [w for ax, dia, w in world_holes(na) if ax == ax_c and dia == 4.0 and abs(w[ax_c] - plane) <= WALL_MM]
    hb = [w for ax, dia, w in world_holes(nb) if ax == ax_c and dia == 4.0 and abs(w[ax_c] - plane) <= WALL_MM]
    oth = [i for i in range(3) if i != ax_c]
    n_match = sum(1 for p1 in ha if any(abs(p1[oth[0]] - p2[oth[0]]) < 0.2 and abs(p1[oth[1]] - p2[oth[1]]) < 0.2 for p2 in hb))
    a8.append(n_match >= 2)
    a8msg.append(f"joint {na.split('_')[1]} / {nb.split('_')[1]}_{nb.split('_')[2]} on {'xyz'[ax_c]}={plane:.1f}: {n_match} coincident 4 mm holes")
on8 = lambda v: abs(v / 8 - round(v / 8)) < 0.01  # noqa: E731
grid_ok = on8(INTAKE_Y_MM) and on8(INTAKE_Z_MM - sum(FRAME_Z_MM) / 2) and on8(Y_MI)
# motor 1201s on the brace: measured 1201 edge holes (thickness 8, +-16) must hit the brace flange 4 mm holes
e1201 = [(ly, lz) for ax, dia, lx, ly, lz in HOLES["1201-0043-0002"]["holes"] if ax == "x"]
bh = world_holes("1120-0013-0336_brace-fly_1")
for mname, yface in (("mount-motor-xfer", BRACE[1][0]), ("mount-motor-fly", BRACE[1][1])):
    mb = next(q[2] for q in PARTS if mname in q[1]).BoundingBox()
    zc = (mb.zmin + mb.zmax) / 2
    best_n = 0
    for xh in (mb.xmin + 8, mb.xmax - 8):
        n_ = sum(1 for ly, lz in e1201 if any(ax == 1 and dia == 4.0 and abs(w[1] - yface) <= WALL_MM and abs(w[0] - xh) < 0.2 and abs(w[2] - (zc + lz)) < 0.2 for ax, dia, w in bh))
        best_n = max(best_n, n_)
    a8.append(best_n >= 2)
    a8msg.append(f"{mname} on brace flange y={yface:.0f}: {best_n} measured 1201 edge holes coincide with brace holes")
for k, (nl, nu) in SPACER_HOLES.items():
    a8.append(nl >= 2 and nu >= 2)
    a8msg.append(f"ladder_spacer_{k}: {nl} ladder-flange holes + {nu} upright-flange holes used (real holes only)")
for m in a8msg:
    print("   ", m)
print(f"    intake/motor 1201 centres on the 8 mm grid: {'ON' if grid_ok else 'OFF'} (1201s are not in the hole table)")
check("A8", all(a8) and grid_ok and len(a8) >= 10, f"{len(a8)} hole checks against measured STEP hole tables, {a8.count(False)} failed")

# A9 powered path, 5 mm steps: floor (from the first step where the intake roller grips >= 2 mm), corner,
# ramp up to the pinch. Powered = intake roller, Gecko transfer wheels, Rhino wheels. Also flags squeeze > 10.5.
y_corner = ramp_pt(S_CORNER, N_BALL)[1]
path = [(0.0, y, ZC_FLOOR) for y in [y_corner - 5 * i for i in range(60)][::-1]] + \
       [ramp_pt(S_CORNER + 5 * i, N_BALL) for i in range(1, int((S_PINCH - S_CORNER) / 5) + 1)] + [ramp_pt(S_PINCH, N_BALL)]
sq = [squeeze(p) for p in path]
start = next(i for i, (v, n) in enumerate(sq) if v >= 2.0 and "intake" in n)
gap, run, worst, low = 0.0, 0.0, 0.0, 99.0
segs, cur = [], None
for i in range(start, len(path)):
    ok9 = sq[i][0] >= 2.0
    run = 0 if ok9 else run + 5
    gap = max(gap, run)
    worst, low = max(worst, sq[i][0]), min(low, sq[i][0])
    lab = sq[i][1].split("_", 1)[1].rsplit("_", 1)[0] if ok9 else "UNPOWERED"
    if cur is None or cur[0] != lab:
        cur = [lab, path[i], path[i], sq[i][0], sq[i][0]]
        segs.append(cur)
    else:
        cur[2], cur[3], cur[4] = path[i], min(cur[3], sq[i][0]), max(cur[4], sq[i][0])
for lab, a_, b_, lo, hi in segs:
    print(f"    {lab:30s} Y {a_[1]:7.1f}..{b_[1]:7.1f}  Z {a_[2]:6.1f}..{b_[2]:6.1f}  squeeze {lo:.1f}..{hi:.1f}")
stored_ok = []
for i, s in enumerate(S_BALLS):
    v, n = squeeze(ramp_pt(s, N_BALL))
    stored_ok.append(v >= 2.0 and "rhino" not in n)
    print(f"    stored ball {i + 1} at s {s:.1f}: squeeze {v:.1f} mm by {n}")
corner_sq = squeeze(ramp_pt(S_CORNER, N_BALL))
# lane play x = -3.5 / 0 / +3.5: compliant squeeze >= 2 and NO rigid intrusion (distance-based)
RIGID_ALL = [(p[1], p[2]) for p in PARTS if not any(k in p[1] for k in ("existing_band_roller", "rhino", "gecko"))]
RIGID_BB = [(n, s, s.BoundingBox()) for n, s in RIGID_ALL]
rig_worst, lat_low, rig_where = 0.0, 99.0, []
for xo in (-3.5, 0.0, 3.5):
    for pt in path[start:]:
        q = (xo, pt[1], pt[2])
        lat_low = min(lat_low, squeeze(q)[0])
        near = [(n, s) for n, s, b_ in RIGID_BB if b_.xmin - 36 < q[0] < b_.xmax + 36 and b_.ymin - 36 < q[1] < b_.ymax + 36 and b_.zmin - 36 < q[2] < b_.zmax + 36]
        for qq, n in squeezes(q, near):
            if qq > 0.05:
                rig_where.append((round(qq, 1), n.split("_")[1], xo, round(pt[1], 1), round(pt[2], 1)))
                rig_worst = max(rig_worst, qq)
for r_ in sorted(set(rig_where), reverse=True)[:8]:
    print(f"    RIGID intrusion {r_[0]} mm by {r_[1]} at x {r_[2]:+.1f}, Y {r_[3]}, Z {r_[4]}")
check("A9", gap == 0 and all(stored_ok) and worst <= 10.5 and lat_low >= 2.0 and rig_worst <= 0.05,
      f"{len(path) - start} steps from first intake grip (Y {path[start][1]:.1f}) to pinch: longest unpowered run {gap:.0f} mm, "
      f"squeeze min {low:.1f} / max {worst:.1f} mm, corner {corner_sq[0]:.1f} mm, stored balls gripped {sum(stored_ok)}/4; "
      f"x=+-3.5: min compliant squeeze {lat_low:.1f} mm, max rigid intrusion {rig_worst:.1f} mm")

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
    need = [a for a in (("roller-intake",) if x == 0 else ("funnel_guide",)) if not any(a in n for n in hit)]  # centre ball is squeezed, off-centre ones deflected
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
            "3209-0001-0007_axle", "5203-2402-0019_drive-motor", "3209-0001-0007_kit-hw"}
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
        rows.append(("TEAM", short, q, "", f"team-supplied band roller: required OD {2 * INTAKE_R_MM:.1f} mm, length {INTAKE_ROLLER_LEN_MM:.0f} mm (8 mm squeeze at on-grid axis)"))
    elif sku == "REV":
        rows.append(("REV", short, q, "", "approximate envelope"))
    else:
        note = "sold as 2-pack" if sku.startswith("1611") else ("motor face: 6x M4 threaded (16 mm square + 2 at +-12 mm)" if sku.startswith("1201") else "")
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
print(f"Required TEAM roller: intake OD {2 * INTAKE_R_MM:.1f} mm x {INTAKE_ROLLER_LEN_MM:.0f} mm long")
print("exported to", OUT)
