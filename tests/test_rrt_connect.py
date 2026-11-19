"""RRT-Connect sanity checks on a 2D toy problem (no Drake needed).

    python -m pytest tests            # or: python tests/test_rrt_connect.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from wimp.planning.rrt_connect import ConfigSpace, path_length, rrt_connect, shortcut  # noqa: E402


def wall_with_gap_cspace():
    """Unit square with a vertical wall at x = 0.5 and a gap at y in [0.8, 0.9]."""

    def is_free(q):
        x, y = q
        if not (0 <= x <= 1 and 0 <= y <= 1):
            return False
        return not (0.45 <= x <= 0.55 and not 0.8 <= y <= 0.9)

    return ConfigSpace(
        sample=lambda rng: rng.uniform(0, 1, size=2),
        distance=lambda a, b: float(np.linalg.norm(b - a)),
        interpolate=lambda a, b, t: a + t * (b - a),
        is_free=is_free,
        edge_resolution=0.005,
    )


def test_finds_path_through_gap_and_shortcut_helps():
    cs = wall_with_gap_cspace()
    start, goal = np.array([0.1, 0.1]), np.array([0.9, 0.1])
    for seed in range(5):
        rng = np.random.default_rng(seed)
        res = rrt_connect(start, goal, cs, step=0.05, max_iterations=20000, rng=rng)
        assert res.path is not None, f"seed {seed}: no path"
        assert np.allclose(res.path[0], start) and np.allclose(res.path[-1], goal)
        for a, b in zip(res.path[:-1], res.path[1:]):
            assert cs.check_edge(a, b)
        short = shortcut(res.path, cs, iterations=300, rng=rng)
        for a, b in zip(short[:-1], short[1:]):
            assert cs.check_edge(a, b)
        assert path_length(short, cs.distance) <= path_length(res.path, cs.distance) + 1e-9
        # Must go up through the gap: any valid path is at least this long.
        assert path_length(short, cs.distance) > 2 * np.hypot(0.4, 0.7) - 0.05


def test_straight_line_when_free():
    cs = wall_with_gap_cspace()
    res = rrt_connect(np.array([0.1, 0.1]), np.array([0.3, 0.9]), cs, step=0.05,
                      rng=np.random.default_rng(0))
    short = shortcut(res.path, cs, iterations=300, rng=np.random.default_rng(0))
    assert len(short) == 2


if __name__ == "__main__":
    test_finds_path_through_gap_and_shortcut_helps()
    test_straight_line_when_free()
    print("ok")
