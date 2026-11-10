#!/usr/bin/env python3
"""Launch-speed window for POLLEN into the CELL goal opening.

Model: 2D projectile, NO DRAG (ideal ballistics, air resistance ignored).
Ball centre leaves (0, H0) at fixed THETA. Scores if the centre crosses the
opening segment from the robot side, at a point >= BALL_R from both ends.
Stdlib only.
"""
import math

IN_M = 0.0254
G_M_S2 = 9.81
H0_IN = 15.0                 # launcher exit height above floor
H0_M = H0_IN * IN_M
BALL_R_IN = 1.4              # POLLEN radius (2.8 in dia)
LIP_H_IN = 53.5              # CELL lower lip height
TOP_H_IN = 65.6              # CELL upper edge height
TILT_DX_IN = 7.0             # upper edge is 7.0 in farther from robot (14*sin30)
DT_S = 0.002                 # trajectory sample step
V_MIN_MS, V_MAX_MS, V_STEP_MS = 3.0, 12.0, 0.005
THETAS_DEG = (50, 55, 60, 65)
DS_IN = (24, 36, 48, 60, 72)
EFF = 0.8                    # ASSUMPTION: ball_speed = 0.5 * wheel_surface_speed * EFF
WHEEL_D_M = 0.096


def pos(v, theta, t, h0_m=H0_M):
    """Ball centre (x, y) in m at time t; speed v in m/s, angle theta in rad."""
    return (v * math.cos(theta) * t,
            h0_m + v * math.sin(theta) * t - 0.5 * G_M_S2 * t * t)


def scores(v, theta, d_in):
    """True if a ball launched at v (m/s) enters the opening at distance d_in (in)."""
    ax, ay = d_in * IN_M, LIP_H_IN * IN_M                        # lower lip
    dx, dy = TILT_DX_IN * IN_M, (TOP_H_IN - LIP_H_IN) * IN_M     # lip -> upper edge
    seg_len = math.hypot(dx, dy)
    r = BALL_R_IN * IN_M
    side = lambda p: dx * (p[1] - ay) - dy * (p[0] - ax)         # >0 = robot side
    p, t = pos(v, theta, 0.0), 0.0
    while True:
        t += DT_S
        q = pos(v, theta, t)
        s0, s1 = side(p), side(q)
        if s0 > 0 and s1 <= 0:                                   # robot side -> goal side
            f = s0 / (s0 - s1)
            cx, cy = p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1])
            along = ((cx - ax) * dx + (cy - ay) * dy) / seg_len   # distance from lower lip
            if r <= along <= seg_len - r:
                return True
        p = q
        if q[0] > ax + dx or q[1] < 0:                           # past opening or floor
            return False


def window(theta_deg, d_in):
    """(v_min, v_max) over scoring speeds on the grid, or None if none score."""
    theta = math.radians(theta_deg)
    n = round((V_MAX_MS - V_MIN_MS) / V_STEP_MS)
    speeds = [V_MIN_MS + i * V_STEP_MS for i in range(n + 1)]
    hits = [v for v in speeds if scores(v, theta, d_in)]
    return (hits[0], hits[-1]) if hits else None


def landing_range(v, theta):
    """Horizontal distance (m) where ball centre returns to h=0, via pos() bisection."""
    lo, hi = 1e-9, 10.0      # y > 0 at lo, y < 0 at hi
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if pos(v, theta, mid, 0.0)[1] > 0:
            lo = mid
        else:
            hi = mid
    return pos(v, theta, lo, 0.0)[0]


def main():
    print(f"{'theta':>5} {'d_in':>4} {'v_min':>7} {'v_max':>7} {'v_mid':>7} {'win%':>7} {'rpm':>6}")
    rows, best = [], {}
    for th in THETAS_DEG:
        for d in DS_IN:
            w = window(th, d)
            if w is None:
                print(f"{th:>5} {d:>4}  none")
                continue
            vmin, vmax = w
            vmid = 0.5 * (vmin + vmax)
            wpct = (vmax - vmin) / vmid * 100
            surf = vmid / (0.5 * EFF)                             # wheel surface speed m/s
            rpm = surf / (math.pi * WHEEL_D_M) * 60
            rows.append((th, d, vmin, vmax, vmid))
            print(f"{th:>5} {d:>4} {vmin:7.3f} {vmax:7.3f} {vmid:7.3f} {wpct:7.2f} {rpm:6.0f}")
            if th not in best or wpct > best[th][1]:
                best[th] = (d, wpct)
    for th in THETAS_DEG:
        b = best.get(th)
        print(f"best d for theta {th}: " + (f"{b[0]} in (win {b[1]:.2f}%)" if b else "none"))

    # self-checks
    v = 5.0
    assert abs(landing_range(v, math.radians(45)) - v * v / G_M_S2) < 1e-3
    for th, d, vmin, vmax, vmid in rows:
        assert scores(vmid, math.radians(th), d), (th, d, vmid)
        assert vmin < vmax, (th, d)
    print("asserts passed")


if __name__ == "__main__":
    main()
