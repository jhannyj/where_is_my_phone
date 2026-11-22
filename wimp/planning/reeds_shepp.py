"""Reeds-Shepp car steering (forward/reverse arcs + straight lines).

No spinning in place: every motion obeys |d(yaw)/ds| <= 1 / TURNING_RADIUS.
"""

from typing import List, Tuple

import numpy as np
from pydrake.all import PiecewisePolynomial

# Minimum turning radius (m). Smaller = tighter turns; larger = wider car-like arcs.
TURNING_RADIUS = 0.6
_EPS = 1e-9


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _mod2pi(a: float) -> float:
    return float(a % (2 * np.pi))


def _polar(x: float, y: float) -> Tuple[float, float]:
    return float(np.hypot(x, y)), _mod2pi(np.arctan2(y, x))


# Each segment is (kind, signed_arc_length_in_unit_radius) where kind in {'L', 'R', 'S'}.
Segment = Tuple[str, float]


def _lsl(x, y, phi) -> List[Segment] | None:
    u, t = _polar(x - np.sin(phi), y - 1.0 + np.cos(phi))
    v = _mod2pi(phi - t)
    return [("L", t), ("S", u), ("L", v)]


def _lsr(x, y, phi) -> List[Segment] | None:
    u1, t1 = _polar(x + np.sin(phi), y - 1.0 - np.cos(phi))
    if u1 < 2.0:
        return None
    u = float(np.sqrt(u1 * u1 - 4.0))
    theta = float(np.arctan2(2.0, u))
    t = _mod2pi(t1 + theta)
    v = _mod2pi(t - phi)
    return [("L", t), ("S", u), ("R", v)]


def _lrl(x, y, phi) -> List[Segment] | None:
    u1, t1 = _polar(x - np.sin(phi), y - 1.0 + np.cos(phi))
    if u1 > 4.0:
        return None
    u = float(np.arccos(np.clip(1.0 - (u1 * u1) / 8.0, -1.0, 1.0)))
    t = _mod2pi(t1 + np.pi - 0.5 * u)
    v = _mod2pi(phi - t - u)
    return [("L", t), ("R", -u), ("L", v)]


def _timeflip(segs: List[Segment]) -> List[Segment]:
    return [(k, -s) for k, s in segs]


def _reflect(segs: List[Segment]) -> List[Segment]:
    swap = {"L": "R", "R": "L", "S": "S"}
    return [(swap[k], s) for k, s in segs]


def shortest_path(a, b, r: float = TURNING_RADIUS) -> List[Segment]:
    """Shortest Reeds-Shepp path from pose a to pose b, with lengths in meters."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = np.cos(a[2]), np.sin(a[2])
    x = (c * dx + s * dy) / r
    y = (-s * dx + c * dy) / r
    phi = float(wrap_angle(b[2] - a[2]))

    candidates: List[List[Segment]] = []
    for fn in (_lsl, _lsr, _lrl):
        for (xx, yy, pp), tf, rf in [
            ((x, y, phi), False, False),
            ((-x, y, -phi), True, False),
            ((x, -y, -phi), False, True),
            ((-x, -y, phi), True, True),
        ]:
            segs = fn(xx, yy, pp)
            if segs is None:
                continue
            if tf:
                segs = _timeflip(segs)
            if rf:
                segs = _reflect(segs)
            candidates.append(segs)

    best = min(candidates, key=lambda segs: sum(abs(l) for _, l in segs))
    return [(k, l * r) for k, l in best if abs(l * r) > _EPS]


def distance(a, b, r: float = TURNING_RADIUS) -> float:
    return float(sum(abs(l) for _, l in shortest_path(a, b, r)))


def _advance(pose: np.ndarray, kind: str, length: float, r: float) -> np.ndarray:
    x, y, yaw = pose
    if kind == "S":
        return np.array([x + length * np.cos(yaw), y + length * np.sin(yaw), yaw])
    dtheta = (length / r) if kind == "L" else -(length / r)
    if kind == "L":
        nx = x + r * (np.sin(yaw + dtheta) - np.sin(yaw))
        ny = y + r * (-np.cos(yaw + dtheta) + np.cos(yaw))
    else:
        nx = x + r * (-np.sin(yaw + dtheta) + np.sin(yaw))
        ny = y + r * (np.cos(yaw + dtheta) - np.cos(yaw))
    return np.array([nx, ny, yaw + dtheta])


def interpolate(a, b, t: float, r: float = TURNING_RADIUS) -> np.ndarray:
    """Pose at fraction t in [0, 1] along the Reeds-Shepp path from a to b."""
    a = np.asarray(a, float)
    segs = shortest_path(a, b, r)
    total = sum(abs(l) for _, l in segs)
    if total < _EPS:
        return a.copy()
    rem = float(np.clip(t, 0.0, 1.0)) * total
    cur = a.copy()
    for kind, length in segs:
        alen = abs(length)
        if rem <= alen + _EPS:
            return _advance(cur, kind, np.sign(length) * rem, r)
        cur = _advance(cur, kind, length, r)
        rem -= alen
    return cur


def sample_path(path, step: float = 0.05, r: float = TURNING_RADIUS) -> List[np.ndarray]:
    """Densify a sequence of poses along exact Reeds-Shepp curves (unwrapped yaw)."""
    path = [np.asarray(q, float) for q in path]
    out = [path[0].copy()]
    for b in path[1:]:
        a = out[-1]
        segs = shortest_path(a, b, r)
        cur = a.copy()
        for kind, length in segs:
            n = max(1, int(np.ceil(abs(length) / step)))
            for i in range(1, n + 1):
                out.append(_advance(cur, kind, length * (i / n), r))
            cur = out[-1].copy()
    return out


def time_parameterize(
    path, v_max: float = 0.5, w_max: float = 0.8, step: float = 0.05, r: float = TURNING_RADIUS
) -> PiecewisePolynomial:
    """Smooth CubicHermite trajectory along Reeds-Shepp arcs.

    Drives continuously through curves and only comes to a stop at the start,
    end, and forward/reverse cusps.
    """
    path = [np.asarray(q, float) for q in path]
    # Flatten into (kind, signed_length) segments across the whole path.
    all_segs: List[Segment] = []
    cur = path[0].copy()
    for b in path[1:]:
        segs = shortest_path(cur, b, r)
        for kind, length in segs:
            all_segs.append((kind, length))
            cur = _advance(cur, kind, length, r)

    if not all_segs:
        q0 = path[0].reshape(3, 1)
        return PiecewisePolynomial.CubicHermite([0.0, 0.1], np.hstack([q0, q0]), np.zeros((3, 2)))

    # Group consecutive segments that share the same driving direction (+1 or -1).
    chunks: List[List[Segment]] = [[all_segs[0]]]
    for seg in all_segs[1:]:
        if np.sign(seg[1]) == np.sign(chunks[-1][-1][1]):
            chunks[-1].append(seg)
        else:
            chunks.append([seg])

    v_cruise = min(v_max, w_max * r)
    times = [0.0]
    qs = [path[0].copy()]
    qdots = [np.zeros(3)]
    cur = path[0].copy()

    for chunk in chunks:
        direction = float(np.sign(chunk[0][1]))
        # Sample points and arc-lengths within this same-direction chunk.
        c_pts = [cur.copy()]
        c_s = [0.0]
        c_kappa = [0.0]
        for kind, length in chunk:
            kappa = (1.0 / r) if kind == "L" else ((-1.0 / r) if kind == "R" else 0.0)
            n = max(1, int(np.ceil(abs(length) / step)))
            for i in range(1, n + 1):
                c_pts.append(_advance(cur, kind, length * (i / n), r))
                c_s.append(c_s[-1] + abs(length) / n)
                c_kappa.append(kappa)
            cur = c_pts[-1].copy()

        total_s = c_s[-1]
        # Smooth cubic s(tau) = total_s * (3 tau^2 - 2 tau^3) so v=0 at chunk ends.
        T = max(1.5 * total_s / v_cruise, 0.2)
        t0 = times[-1]
        for i in range(1, len(c_pts)):
            # Invert s(tau) approximately via uniform tau = c_s[i] / total_s is fine
            # for interior points; compute exact speed from cubic profile.
            tau = c_s[i] / total_s
            v_mag = (total_s / T) * 6.0 * tau * (1.0 - tau)
            v_signed = direction * v_mag
            yaw = c_pts[i][2]
            times.append(t0 + tau * T)
            qs.append(c_pts[i])
            qdots.append(np.array([v_signed * np.cos(yaw), v_signed * np.sin(yaw), v_signed * c_kappa[i]]))

    Q = np.array(qs).T
    Qd = np.array(qdots).T
    return PiecewisePolynomial.CubicHermite(np.array(times), Q, Qd)
