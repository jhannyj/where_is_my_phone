"""Turn-drive-turn steering for the diff-drive base (no Drake needed)."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from wimp.planning import diff_drive as dd  # noqa: E402


def heading_of_motion(p, q):
    return np.arctan2(q[1] - p[1], q[0] - p[0])


def test_never_moves_sideways():
    rng = np.random.default_rng(0)
    for _ in range(200):
        a = np.array([*rng.uniform(-3, 3, 2), rng.uniform(-np.pi, np.pi)])
        b = np.array([*rng.uniform(-3, 3, 2), rng.uniform(-np.pi, np.pi)])
        wps = dd.expand_to_primitives([a, b])
        for p, q in zip(wps[:-1], wps[1:]):
            moved = np.hypot(*(q[:2] - p[:2]))
            turned = abs(q[2] - p[2])
            assert moved < 1e-9 or turned < 1e-9, "turn and drive must not mix"
            if moved > 1e-9:  # driving: heading along the motion, fwd or back
                err = dd.wrap_angle(heading_of_motion(p, q) - p[2])
                assert min(abs(err), abs(abs(err) - np.pi)) < 1e-9
        end = wps[-1]
        assert np.allclose(end[:2], b[:2])
        assert abs(dd.wrap_angle(end[2] - b[2])) < 1e-9


def test_reversing_used_when_cheaper():
    # Facing +x, goal 1 m behind with the same heading: back up, don't turn.
    L, r1, r2 = dd.rtr(np.array([0.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0]))
    assert L == 1.0 and abs(r1) < 1e-12 and abs(r2) < 1e-12


def test_distance_symmetric_and_interpolate_endpoints():
    rng = np.random.default_rng(1)
    for _ in range(100):
        a = np.array([*rng.uniform(-3, 3, 2), rng.uniform(-np.pi, np.pi)])
        b = np.array([*rng.uniform(-3, 3, 2), rng.uniform(-np.pi, np.pi)])
        assert abs(dd.distance(a, b) - dd.distance(b, a)) < 1e-9
        assert np.allclose(dd.interpolate(a, b, 0.0), a)
        e = dd.interpolate(a, b, 1.0)
        assert np.allclose(e[:2], b[:2]) and abs(dd.wrap_angle(e[2] - b[2])) < 1e-9


if __name__ == "__main__":
    test_never_moves_sideways()
    test_reversing_used_when_cheaper()
    test_distance_symmetric_and_interpolate_endpoints()
    print("ok")
