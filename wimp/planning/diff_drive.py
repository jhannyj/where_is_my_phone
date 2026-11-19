"""Steering for a skid-steer / differential-drive base (D-015).

Such a base can turn in place and drive straight forwards or backwards, but
cannot move sideways. Between two poses a = (x, y, yaw) and b we use the
rotate-translate-rotate (RTR) motion:

    1. turn in place from yaw_a to the driving heading h,
    2. drive straight from (x_a, y_a) to (x_b, y_b) with heading h,
    3. turn in place from h to yaw_b,

where h is the segment direction (forwards) or that +pi (reversing),
whichever needs less total turning. Any sequence of RTR motions is feasible
for a diff-drive base, so a planner that only uses RTR edges produces
executable paths.

Cost (the planner's distance metric) = drive length + YAW_WEIGHT * total
turning. It is symmetric: going b -> a uses the same heading line. No Drake
dependency, so it is unit tested in tests/test_diff_drive.py.
"""

from typing import List, Tuple

import numpy as np

# Metres of cost per radian of turning ~ how far the chassis corners sweep
# per radian (chassis half-diagonal is 0.5 m). D-004.
YAW_WEIGHT = 0.5
_EPS = 1e-9


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rtr(a, b) -> Tuple[float, float, float]:
    """(drive length L, first turn r1, second turn r2), turns in radians."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = float(np.hypot(dx, dy))
    if L < _EPS:
        return 0.0, float(wrap_angle(b[2] - a[2])), 0.0
    theta = np.arctan2(dy, dx)
    best = None
    for h in (theta, theta + np.pi):  # forwards, backwards
        r1 = float(wrap_angle(h - a[2]))
        r2 = float(wrap_angle(b[2] - (a[2] + r1)))
        turn = abs(r1) + abs(r2)
        if best is None or turn < best[0] - 1e-12:
            best = (turn, r1, r2)
    return L, best[1], best[2]


def distance(a, b) -> float:
    L, r1, r2 = rtr(a, b)
    return L + YAW_WEIGHT * (abs(r1) + abs(r2))


def interpolate(a, b, t: float) -> np.ndarray:
    """Pose a fraction t (of the cost) along the RTR motion from a to b.
    Yaw is returned unwrapped relative to a's yaw."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    L, r1, r2 = rtr(a, b)
    c1, c2, c3 = YAW_WEIGHT * abs(r1), L, YAW_WEIGHT * abs(r2)
    total = c1 + c2 + c3
    if total < _EPS:
        return a.copy()
    s = np.clip(t, 0.0, 1.0) * total
    out = a.copy()
    if s <= c1 and c1 > 0:
        out[2] = a[2] + r1 * s / c1
    elif s <= c1 + c2 and c2 > 0:
        f = (s - c1) / c2
        out[:2] = a[:2] + f * (b[:2] - a[:2])
        out[2] = a[2] + r1
    else:
        f = (s - c1 - c2) / c3 if c3 > 0 else 1.0
        out[:2] = b[:2]
        out[2] = a[2] + r1 + r2 * f
    return out


def expand_to_primitives(path) -> List[np.ndarray]:
    """Turn a list of poses into the waypoints of the actual RTR motion:
    each consecutive pair becomes up to three waypoints (after turn 1,
    after the drive, after turn 2). Between consecutive output waypoints
    only yaw changes, or only (x, y) changes, so straight-line joint-space
    interpolation between them reproduces the RTR motion exactly. Yaw is
    continuous (unwrapped)."""
    path = [np.asarray(q, float) for q in path]
    cur = path[0].copy()
    out = [cur.copy()]
    for b in path[1:]:
        L, r1, r2 = rtr(cur, b)
        if abs(r1) > 1e-6:
            cur = cur.copy()
            cur[2] += r1
            out.append(cur)
        if L > 1e-6:
            cur = cur.copy()
            cur[:2] = b[:2]
            out.append(cur)
        if abs(r2) > 1e-6:
            cur = cur.copy()
            cur[2] += r2
            out.append(cur)
    return out


def path_cost(path) -> float:
    return float(sum(distance(a, b) for a, b in zip(path[:-1], path[1:])))
