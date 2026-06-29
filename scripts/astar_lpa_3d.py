#!/usr/bin/env python3
"""LPAStar3D - Lifelong Planning A* TCP voxel guard planner.

Drop-in alternative to AStarImproved3D: same public API (plan_with_info /
replan_with_info / plan / replan / set_penalty_cells, same PlanResult and reason
codes), selectable in Module F via ~guard_planner_type. LPA* keeps the previous
search and repairs only the cells affected by a local obstacle change instead of
replanning from scratch.

Scope: TCP voxel path existence only. Whole-arm collision validation stays in
MoveIt (check_state_validity, group.plan, trajectory_is_safe). Never execute on
LPA* output alone.

v1 simplifications (ponytail): REPAIR only when the start voxel is unchanged; any
start movement -> RESET (always correct; start-move re-root is a later version).
No path cache; a no-change replan just re-runs compute_shortest_path, which
returns immediately when everything is already consistent.

Offline test: python3 scripts/test_astar_lpa_3d.py
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

from astar_improved_3d import (
    Voxel,
    PlanResult,
    INF,
    OK,
    INVALID_START,
    INVALID_GOAL,
    START_OUT_OF_BOUNDS,
    GOAL_OUT_OF_BOUNDS,
    START_BLOCKED,
    GOAL_BLOCKED,
    NO_PATH,
    MAX_STEPS_REACHED,
    PATH_EXTRACTION_FAILED,
    REPLAN_NOT_INITIALIZED,
    EMPTY_GRID,
    TIMEOUT,
)

Key = Tuple[float, float]


class LPAStar3D:
    def __init__(
        self,
        size_x: int,
        size_y: int,
        size_z: int,
        diagonal: bool = True,
        max_time_ms: float = 50.0,
        max_steps: int = 50000,
        smooth: bool = True,
        epsilon: float = 1.0,
        start_reuse_radius_voxels: int = 1,
        max_changed_obstacles_for_repair: int = 500,
    ) -> None:
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.diagonal = diagonal
        self.max_time_ms = max(0.0, float(max_time_ms))
        self.max_steps = max(1, int(max_steps))
        self.smooth = smooth
        self.epsilon = max(1.0, float(epsilon))
        # Reserved for a later start-move re-root; v1 repairs only on identical start.
        self.start_reuse_radius_voxels = max(0, int(start_reuse_radius_voxels))
        self.max_changed_obstacles_for_repair = max(0, int(max_changed_obstacles_for_repair))

        self._neighbor_offsets = self._build_neighbor_offsets(diagonal)

        self._g: Dict[Voxel, float] = {}
        self._rhs: Dict[Voxel, float] = {}
        self._parent: Dict[Voxel, Optional[Voxel]] = {}
        self._open: List[Tuple[Key, int, Voxel]] = []
        self._open_best: Dict[Voxel, Key] = {}
        self._counter = 0

        self._start: Optional[Voxel] = None
        self._goal: Optional[Voxel] = None
        self._obstacles: Set[Voxel] = set()
        self._previous_obstacles: Set[Voxel] = set()
        self._last_path: List[Voxel] = []

        self._penalty_cells: Set[Voxel] = set()
        self._penalty_weight = 0.0

        self.last_result: Optional[PlanResult] = None

    # ---- grid helpers (own copies; astar_improved_3d untouched) ----------

    @staticmethod
    def _build_neighbor_offsets(diagonal: bool) -> List[Tuple[int, int, int]]:
        if not diagonal:
            return [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        offsets: List[Tuple[int, int, int]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == dy == dz == 0:
                        continue
                    offsets.append((dx, dy, dz))
        return offsets

    @staticmethod
    def is_valid_voxel(s: Any) -> bool:
        if not isinstance(s, (tuple, list)) or len(s) != 3:
            return False
        return all(isinstance(v, int) and not isinstance(v, bool) for v in s)

    def normalize_voxel(self, s: Any) -> Optional[Voxel]:
        if not self.is_valid_voxel(s):
            return None
        x, y, z = s
        return (int(x), int(y), int(z))

    def grid_is_valid(self) -> bool:
        return self.size_x > 0 and self.size_y > 0 and self.size_z > 0

    def in_bounds(self, s: Voxel) -> bool:
        x, y, z = s
        return 0 <= x < self.size_x and 0 <= y < self.size_y and 0 <= z < self.size_z

    def passable(self, s: Voxel) -> bool:
        return s not in self._obstacles

    def neighbors(self, s: Voxel) -> List[Voxel]:
        x, y, z = s
        out: List[Voxel] = []
        for dx, dy, dz in self._neighbor_offsets:
            n = (x + dx, y + dy, z + dz)
            if self.in_bounds(n) and self.passable(n):
                out.append(n)
        return out

    def raw_neighbors(self, s: Voxel) -> List[Voxel]:
        x, y, z = s
        out: List[Voxel] = []
        for dx, dy, dz in self._neighbor_offsets:
            n = (x + dx, y + dy, z + dz)
            if self.in_bounds(n):
                out.append(n)
        return out

    def set_penalty_cells(self, cells: Set[Voxel], weight: float) -> None:
        self._penalty_cells = set(cells) if weight > 0.0 else set()
        self._penalty_weight = max(0.0, float(weight))

    def cost(self, a: Voxel, b: Voxel) -> float:
        if (not self.in_bounds(a)) or (not self.in_bounds(b)):
            return INF
        if (not self.passable(a)) or (not self.passable(b)):
            return INF
        dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
        base = math.sqrt(dx * dx + dy * dy + dz * dz)
        if self._penalty_weight > 0.0 and b in self._penalty_cells:
            base += self._penalty_weight
        return base

    def heuristic(self, a: Voxel, b: Voxel) -> float:
        dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def line_of_sight(self, a: Voxel, b: Voxel) -> bool:
        x0, y0, z0 = a
        x1, y1, z1 = b
        dx, dy, dz = abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        sz = 1 if z1 > z0 else -1
        x, y, z = x0, y0, z0
        if not (self.in_bounds((x, y, z)) and self.passable((x, y, z))):
            return False
        if dx >= dy and dx >= dz:
            ey, ez = 2 * dy - dx, 2 * dz - dx
            for _ in range(dx):
                if ey >= 0:
                    y += sy
                    ey -= 2 * dx
                if ez >= 0:
                    z += sz
                    ez -= 2 * dx
                x += sx
                ey += 2 * dy
                ez += 2 * dz
                if not (self.in_bounds((x, y, z)) and self.passable((x, y, z))):
                    return False
        elif dy >= dx and dy >= dz:
            ex, ez = 2 * dx - dy, 2 * dz - dy
            for _ in range(dy):
                if ex >= 0:
                    x += sx
                    ex -= 2 * dy
                if ez >= 0:
                    z += sz
                    ez -= 2 * dy
                y += sy
                ex += 2 * dx
                ez += 2 * dz
                if not (self.in_bounds((x, y, z)) and self.passable((x, y, z))):
                    return False
        else:
            ex, ey = 2 * dx - dz, 2 * dy - dz
            for _ in range(dz):
                if ex >= 0:
                    x += sx
                    ex -= 2 * dz
                if ey >= 0:
                    y += sy
                    ey -= 2 * dz
                z += sz
                ex += 2 * dx
                ey += 2 * dy
                if not (self.in_bounds((x, y, z)) and self.passable((x, y, z))):
                    return False
        return True

    def smooth_path(self, path: List[Voxel]) -> List[Voxel]:
        if len(path) <= 2:
            return list(path)
        smoothed = [path[0]]
        anchor = 0
        i = 2
        while i < len(path):
            if not self.line_of_sight(path[anchor], path[i]):
                smoothed.append(path[i - 1])
                anchor = i - 1
            i += 1
        smoothed.append(path[-1])
        return smoothed

    def filter_obstacles(self, obstacles: Iterable[Any]) -> Tuple[Set[Voxel], int, int]:
        filtered: Set[Voxel] = set()
        ignored = 0
        raw = 0
        if obstacles is None:
            return filtered, ignored, raw
        try:
            it = iter(obstacles)
        except TypeError:
            return filtered, 1, raw
        for obstacle in it:
            raw += 1
            v = self.normalize_voxel(obstacle)
            if v is None or not self.in_bounds(v):
                ignored += 1
                continue
            filtered.add(v)
        return filtered, ignored, raw

    # ---- LPA* core -------------------------------------------------------

    def _g_of(self, s: Voxel) -> float:
        return self._g.get(s, INF)

    def _rhs_of(self, s: Voxel) -> float:
        return self._rhs.get(s, INF)

    def calculate_key(self, s: Voxel) -> Key:
        if self._goal is None:
            return (INF, INF)
        base = min(self._g_of(s), self._rhs_of(s))
        return (base + self.epsilon * self.heuristic(s, self._goal), base)

    def _open_insert(self, s: Voxel) -> None:
        key = self.calculate_key(s)
        self._open_best[s] = key
        self._counter += 1
        heapq.heappush(self._open, (key, self._counter, s))

    def _pop_min(self) -> Optional[Tuple[Key, Voxel]]:
        """Peek the minimum valid OPEN entry, discarding stale (lazy-deleted) ones."""
        while self._open:
            key, _, s = self._open[0]
            best = self._open_best.get(s)
            if best is None or best != key:
                heapq.heappop(self._open)
                continue
            return key, s
        return None

    def initialize_search(self, start: Voxel, goal: Voxel, obstacles: Set[Voxel]) -> None:
        self._g.clear()
        self._rhs.clear()
        self._parent.clear()
        self._open.clear()
        self._open_best.clear()
        self._counter = 0
        self._start = start
        self._goal = goal
        self._obstacles = set(obstacles)
        self._previous_obstacles = set(obstacles)
        self._rhs[start] = 0.0
        self._parent[start] = None
        self._open_insert(start)

    def update_vertex(self, u: Voxel) -> None:
        if u != self._start:
            best = INF
            best_parent: Optional[Voxel] = None
            for p in self.raw_neighbors(u):
                c = self.cost(p, u)
                if c == INF:
                    continue
                val = self._g_of(p) + c
                if val < best:
                    best = val
                    best_parent = p
            self._rhs[u] = best
            self._parent[u] = best_parent
        # lazy-delete from OPEN; re-insert only if locally inconsistent
        if u in self._open_best:
            del self._open_best[u]
        if self._g_of(u) != self._rhs_of(u):
            self._open_insert(u)

    def update_obstacles(self, new_obstacles: Set[Voxel]) -> int:
        changed = self._previous_obstacles.symmetric_difference(new_obstacles)
        self._obstacles = set(new_obstacles)
        for cell in changed:
            self.update_vertex(cell)
            for n in self.raw_neighbors(cell):
                self.update_vertex(n)
        self._previous_obstacles = set(new_obstacles)
        return len(changed)

    def compute_shortest_path(self, started_at: float, max_steps: int) -> str:
        goal = self._goal
        assert goal is not None
        expanded = 0
        budget_ms = self.max_time_ms
        while True:
            top = self._pop_min()
            if top is None:
                break
            top_key, u = top
            if not (top_key < self.calculate_key(goal)
                    or self._rhs_of(goal) != self._g_of(goal)):
                break
            if budget_ms > 0.0 and (time.time() - started_at) * 1000.0 > budget_ms:
                return TIMEOUT
            if expanded >= max_steps:
                return MAX_STEPS_REACHED

            # consume u from OPEN
            heapq.heappop(self._open)
            self._open_best.pop(u, None)

            if self._g_of(u) > self._rhs_of(u):
                self._g[u] = self._rhs_of(u)
                for s in self.raw_neighbors(u):
                    self.update_vertex(s)
            else:
                self._g[u] = INF
                self.update_vertex(u)
                for s in self.raw_neighbors(u):
                    self.update_vertex(s)
            expanded += 1

        self._last_expanded = expanded
        return OK

    def reconstruct_path(self) -> Optional[List[Voxel]]:
        start, goal = self._start, self._goal
        if start is None or goal is None:
            return None
        if self._g_of(goal) == INF:
            return None
        path: List[Voxel] = []
        seen: Set[Voxel] = set()
        cur: Optional[Voxel] = goal
        while cur is not None:
            if cur in seen:
                return None  # parent loop
            seen.add(cur)
            if not self.in_bounds(cur) or not self.passable(cur):
                return None
            path.append(cur)
            if cur == start:
                path.reverse()
                return self._validate_path(path)
            cur = self._parent.get(cur)
            if len(path) > self.size_x * self.size_y * self.size_z:
                return None
        return None

    def _validate_path(self, path: List[Voxel]) -> Optional[List[Voxel]]:
        for i in range(len(path) - 1):
            d = [abs(path[i][k] - path[i + 1][k]) for k in range(3)]
            if max(d) != 1:
                return None
            if self.cost(path[i], path[i + 1]) == INF:
                return None
        return path

    # ---- public API (drop-in compatible with AStarImproved3D) ------------

    def _finish(
        self,
        path: List[Voxel],
        success: bool,
        reason: str,
        metrics: Dict[str, object],
        reuse_mode: str,
        started_at: float,
    ) -> PlanResult:
        metrics = dict(metrics)
        metrics.update({
            "algorithm": "LPA*",
            "success": success,
            "reason": reason,
            "path_length": len(path),
            "expanded_steps": int(getattr(self, "_last_expanded", 0)),
            "planning_time_ms": (time.time() - started_at) * 1000.0,
            "reuse_mode": reuse_mode,
        })
        self._last_path = path if success else []
        result = PlanResult(path=path, success=success, reason=reason, metrics=metrics)
        self.last_result = result
        return result

    def _search_to_result(
        self,
        metrics: Dict[str, object],
        reuse_mode: str,
        started_at: float,
        max_steps: int,
    ) -> PlanResult:
        self._last_expanded = 0
        reason = self.compute_shortest_path(started_at, max_steps)
        if reason in (TIMEOUT, MAX_STEPS_REACHED):
            return self._finish([], False, reason, metrics, reuse_mode, started_at)
        path = self.reconstruct_path()
        if path is None:
            if self._g_of(self._goal) == INF and self._rhs_of(self._goal) == INF:
                return self._finish([], False, NO_PATH, metrics, reuse_mode, started_at)
            return self._finish([], False, PATH_EXTRACTION_FAILED, metrics, reuse_mode, started_at)
        if self.smooth:
            path = self.smooth_path(path)
        return self._finish(path, True, OK, metrics, reuse_mode, started_at)

    def plan_with_info(
        self,
        start: Any,
        goal: Any,
        obstacles: Iterable[Any],
        max_steps: Optional[int] = None,
    ) -> PlanResult:
        t0 = time.time()
        filtered, ignored, raw = self.filter_obstacles(obstacles)
        metrics: Dict[str, object] = {
            "obstacle_count": len(filtered),
            "ignored_obstacle_count": ignored,
            "raw_obstacle_count": raw,
            "changed_obstacle_count": len(filtered),
        }
        reason, s, g = self._validate_plan(start, goal, filtered)
        if reason != OK:
            self._last_expanded = 0
            return self._finish([], False, reason, metrics, "RESET", t0)
        self.initialize_search(s, g, filtered)
        steps = self.max_steps if max_steps is None else max(1, int(max_steps))
        return self._search_to_result(metrics, "RESET", t0, steps)

    def replan_with_info(
        self,
        new_start: Any,
        new_obstacles: Iterable[Any],
        max_steps: Optional[int] = None,
    ) -> PlanResult:
        t0 = time.time()
        filtered, ignored, raw = self.filter_obstacles(new_obstacles)
        metrics: Dict[str, object] = {
            "obstacle_count": len(filtered),
            "ignored_obstacle_count": ignored,
            "raw_obstacle_count": raw,
        }
        reason, s = self._validate_replan(new_start, filtered)
        if reason != OK:
            self._last_expanded = 0
            metrics["changed_obstacle_count"] = 0
            return self._finish([], False, reason, metrics, "RESET", t0)

        changed = self._previous_obstacles.symmetric_difference(filtered)
        start_moved = s != self._start
        steps = self.max_steps if max_steps is None else max(1, int(max_steps))

        # RESET on any start move (re-root deferred) or an obstacle diff too large
        # to repair cheaply; otherwise REPAIR the existing search incrementally.
        if start_moved or len(changed) > self.max_changed_obstacles_for_repair:
            metrics["changed_obstacle_count"] = len(changed)
            self.initialize_search(s, self._goal, filtered)
            return self._search_to_result(metrics, "RESET", t0, steps)

        changed_count = self.update_obstacles(filtered)
        metrics["changed_obstacle_count"] = changed_count
        return self._search_to_result(metrics, "REPAIR", t0, steps)

    def plan(self, start: Voxel, goal: Voxel, obstacles: Iterable[Voxel]) -> List[Voxel]:
        return self.plan_with_info(start, goal, obstacles).path

    def replan(self, new_start: Voxel, new_obstacles: Iterable[Voxel]) -> List[Voxel]:
        return self.replan_with_info(new_start, new_obstacles).path

    # ---- input validation (mirrors AStarImproved3D order) ----------------

    def _validate_plan(
        self, start: Any, goal: Any, obstacles: Set[Voxel]
    ) -> Tuple[str, Optional[Voxel], Optional[Voxel]]:
        if not self.grid_is_valid():
            return EMPTY_GRID, None, None
        s = self.normalize_voxel(start)
        if s is None:
            return INVALID_START, None, None
        g = self.normalize_voxel(goal)
        if g is None:
            return INVALID_GOAL, s, None
        if not self.in_bounds(s):
            return START_OUT_OF_BOUNDS, s, g
        if not self.in_bounds(g):
            return GOAL_OUT_OF_BOUNDS, s, g
        if s in obstacles:
            return START_BLOCKED, s, g
        if g in obstacles:
            return GOAL_BLOCKED, s, g
        return OK, s, g

    def _validate_replan(
        self, new_start: Any, obstacles: Set[Voxel]
    ) -> Tuple[str, Optional[Voxel]]:
        if not self.grid_is_valid():
            return EMPTY_GRID, None
        if self._start is None or self._goal is None:
            return REPLAN_NOT_INITIALIZED, None
        s = self.normalize_voxel(new_start)
        if s is None:
            return INVALID_START, None
        if not self.in_bounds(s):
            return START_OUT_OF_BOUNDS, s
        if s in obstacles:
            return START_BLOCKED, s
        if self._goal in obstacles:
            return GOAL_BLOCKED, s
        return OK, s
