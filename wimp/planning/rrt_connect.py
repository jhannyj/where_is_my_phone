"""RRT-Connect (Kuffner & LaValle, 2000) and random shortcutting.

Generic over the configuration space: the caller supplies sampling, distance,
interpolation and collision checking. No Drake dependency, so it can be unit
tested on toy problems (tests/test_rrt_connect.py).

Algorithm (see docs/DECISIONS.md D-002):
  Grow two trees, one from the start and one from the goal. Each iteration:
    1. sample q_rand; EXTEND tree A one step (size `step`) toward it.
    2. if that produced a new node q_new, CONNECT tree B toward q_new: keep
       stepping toward it until reaching it (-> path found) or hitting an
       obstacle.
    3. swap the roles of A and B.
  The greedy CONNECT is what makes it much faster than plain RRT in
  relatively open spaces like a room.
"""

from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

Config = np.ndarray


@dataclass
class ConfigSpace:
    sample: Callable[[np.random.Generator], Config]
    distance: Callable[[Config, Config], float]
    # interpolate(a, b, t) for t in [0, 1]; must return a at 0 and b at 1.
    interpolate: Callable[[Config, Config, float], Config]
    is_free: Callable[[Config], bool]
    # Edge check; default discretizes with interpolate + is_free.
    edge_is_free: Optional[Callable[[Config, Config], bool]] = None
    edge_resolution: float = 0.05

    def check_edge(self, a: Config, b: Config) -> bool:
        if self.edge_is_free is not None:
            return self.edge_is_free(a, b)
        n = max(1, int(np.ceil(self.distance(a, b) / self.edge_resolution)))
        return all(self.is_free(self.interpolate(a, b, i / n)) for i in range(1, n + 1))


@dataclass
class _Tree:
    nodes: List[Config] = field(default_factory=list)
    parents: List[int] = field(default_factory=list)

    def add(self, q: Config, parent: int) -> int:
        self.nodes.append(q)
        self.parents.append(parent)
        return len(self.nodes) - 1

    def nearest(self, q: Config, distance) -> int:
        # Brute force is fine for the few thousand nodes a room needs.
        return int(np.argmin([distance(n, q) for n in self.nodes]))

    def path_to_root(self, i: int) -> List[Config]:
        out = []
        while i != -1:
            out.append(self.nodes[i])
            i = self.parents[i]
        return out


@dataclass
class RRTConnectResult:
    path: Optional[List[Config]]
    iterations: int
    start_tree: _Tree
    goal_tree: _Tree


_TRAPPED, _ADVANCED, _REACHED = 0, 1, 2


def _extend(tree: _Tree, q_target: Config, cs: ConfigSpace, step: float):
    i_near = tree.nearest(q_target, cs.distance)
    q_near = tree.nodes[i_near]
    d = cs.distance(q_near, q_target)
    if d <= step:
        q_new, status = q_target, _REACHED
    else:
        q_new, status = cs.interpolate(q_near, q_target, step / d), _ADVANCED
    if not cs.check_edge(q_near, q_new):
        return _TRAPPED, -1
    return status, tree.add(q_new, i_near)


def _connect(tree: _Tree, q_target: Config, cs: ConfigSpace, step: float):
    while True:
        status, i = _extend(tree, q_target, cs, step)
        if status != _ADVANCED:
            return status, i


def rrt_connect(
    q_start: Config,
    q_goal: Config,
    cs: ConfigSpace,
    step: float = 0.3,
    max_iterations: int = 5000,
    rng: Optional[np.random.Generator] = None,
) -> RRTConnectResult:
    rng = rng if rng is not None else np.random.default_rng()
    if not cs.is_free(q_start):
        raise ValueError("start configuration is in collision")
    if not cs.is_free(q_goal):
        raise ValueError("goal configuration is in collision")

    start_tree, goal_tree = _Tree(), _Tree()
    start_tree.add(np.asarray(q_start, float), -1)
    goal_tree.add(np.asarray(q_goal, float), -1)
    tree_a, tree_b = start_tree, goal_tree

    for it in range(1, max_iterations + 1):
        q_rand = cs.sample(rng)
        status, i_new = _extend(tree_a, q_rand, cs, step)
        if status != _TRAPPED:
            q_new = tree_a.nodes[i_new]
            status_b, i_b = _connect(tree_b, q_new, cs, step)
            if status_b == _REACHED:
                half_a = tree_a.path_to_root(i_new)[::-1]  # root_a ... q_new
                half_b = tree_b.path_to_root(i_b)[1:]  # (skip q_new) ... root_b
                path = half_a + half_b
                if tree_a is goal_tree:
                    path = path[::-1]
                return RRTConnectResult(path, it, start_tree, goal_tree)
        tree_a, tree_b = tree_b, tree_a

    return RRTConnectResult(None, max_iterations, start_tree, goal_tree)


def shortcut(
    path: List[Config],
    cs: ConfigSpace,
    iterations: int = 200,
    rng: Optional[np.random.Generator] = None,
) -> List[Config]:
    """Random shortcutting: pick two points on the path, and if the straight
    segment between them is collision-free, replace what lies between.
    Shortens the jagged RRT path; never makes it longer."""
    rng = rng if rng is not None else np.random.default_rng()
    path = [np.asarray(q, float) for q in path]
    for _ in range(iterations):
        if len(path) < 3:
            break
        i, j = sorted(rng.choice(len(path), size=2, replace=False))
        if j - i < 2:
            continue
        if cs.check_edge(path[i], path[j]):
            path = path[: i + 1] + path[j:]
    return path


def path_length(path: List[Config], distance) -> float:
    return float(sum(distance(a, b) for a, b in zip(path[:-1], path[1:])))
