#!/usr/bin/env python3
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

Voxel = Tuple[int, int, int]

OK = "OK"
INVALID_START = "INVALID_START"
INVALID_GOAL = "INVALID_GOAL"
START_OUT_OF_BOUNDS = "START_OUT_OF_BOUNDS"
GOAL_OUT_OF_BOUNDS = "GOAL_OUT_OF_BOUNDS"
START_BLOCKED = "START_BLOCKED"
GOAL_BLOCKED = "GOAL_BLOCKED"
NO_PATH = "NO_PATH"
MAX_STEPS_REACHED = "MAX_STEPS_REACHED"
PATH_EXTRACTION_FAILED = "PATH_EXTRACTION_FAILED"
REPLAN_NOT_INITIALIZED = "REPLAN_NOT_INITIALIZED"
EMPTY_GRID = "EMPTY_GRID"
TIMEOUT = "TIMEOUT"

INF = float("inf")


@dataclass
class PlanResult:
    path: List[Voxel]
    success: bool
    reason: str
    metrics: Dict[str, object] = field(default_factory=dict)

class AStarImproved3D:
    def __init__(
        self,
        size_x: int,
        size_y: int,
        size_z: int,
        diagonal: bool = True,
        epsilon_start: float = 1.5,
        epsilon_final: float = 1.0,
        epsilon_decay: float = 0.2,
        max_time_ms: float = 200.0,
        max_steps: int = 150000,
        smooth: bool = True,
    ) -> None:
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.diagonal = diagonal

        self.epsilon_start = max(1.0, float(epsilon_start))
        self.epsilon_final = max(1.0, float(epsilon_final))
        self.epsilon_decay = max(0.01, float(epsilon_decay))
        if self.epsilon_final > self.epsilon_start:
            self.epsilon_final = self.epsilon_start
        self.max_time_ms = max(0.0, float(max_time_ms))
        self.max_steps = max(1, int(max_steps))
        self.smooth = smooth

        self._neighbor_offsets = self._build_neighbor_offsets(diagonal)

        self._g: Dict[Voxel, float] = {}
        self._v: Dict[Voxel, float] = {}
        self._h: Dict[Voxel, float] = {}
        self._parent: Dict[Voxel, Optional[Voxel]] = {}
        self._closed_iter: Dict[Voxel, int] = {}
        self._incons: Set[Voxel] = set()

        self._open: List[Tuple[float, int, Voxel]] = []
        self._open_best: Dict[Voxel, float] = {}
        self._counter = 0

        self._eps = self.epsilon_start
        self._eps_satisfied = INF
        self._search_iteration = 0
        self._start: Optional[Voxel] = None
        self._goal: Optional[Voxel] = None
        self._obstacles: Set[Voxel] = set()
        self._previous_obstacles: Set[Voxel] = set()
        self._last_path: List[Voxel] = []

        # Soft region preference: entering a penalised voxel adds extra cost so
        # A* prefers paths inside the allowed workspace without hard-blocking.
        self._penalty_cells: Set[Voxel] = set()
        self._penalty_weight = 0.0

        self.last_result: Optional[PlanResult] = None
        self.last_expanded_steps = 0
        self.last_elapsed_ms = 0.0
        self.last_changed_obstacle_count = 0
        self.last_ignored_obstacle_count = 0
        self.last_first_solution_time_ms: Optional[float] = None
        self.last_iterations = 0

    @staticmethod
    def _build_neighbor_offsets(diagonal: bool) -> List[Tuple[int, int, int]]:
        if not diagonal:
            return [
                (1, 0, 0),
                (-1, 0, 0),
                (0, 1, 0),
                (0, -1, 0),
                (0, 0, 1),
                (0, 0, -1),
            ]

        offsets: List[Tuple[int, int, int]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    offsets.append((dx, dy, dz))
        return offsets

    def is_valid_voxel(self, s: Any) -> bool:
        if not isinstance(s, (tuple, list)) or len(s) != 3:
            return False
        x, y, z = s
        return all(isinstance(v, int) and not isinstance(v, bool) for v in (x, y, z))

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
        result: List[Voxel] = []
        for dx, dy, dz in self._neighbor_offsets:
            nxt = (x + dx, y + dy, z + dz)
            if self.in_bounds(nxt) and self.passable(nxt):
                result.append(nxt)
        return result

    def set_penalty_cells(self, cells: Set[Voxel], weight: float) -> None:
        """Cells charged an extra per-step cost (soft region preference)."""
        self._penalty_cells = set(cells) if weight > 0.0 else set()
        self._penalty_weight = max(0.0, float(weight))

    def cost(self, a: Voxel, b: Voxel) -> float:
        if (not self.in_bounds(a)) or (not self.in_bounds(b)):
            return INF
        if (not self.passable(a)) or (not self.passable(b)):
            return INF

        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        base = math.sqrt(dx * dx + dy * dy + dz * dz)
        if self._penalty_weight > 0.0 and b in self._penalty_cells:
            base += self._penalty_weight
        return base

    def heuristic(self, a: Voxel, b: Voxel) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def path_cost(self, path: List[Voxel]) -> float:
        total = 0.0
        for a, b in zip(path[:-1], path[1:]):
            edge_cost = self.cost(a, b)
            if edge_cost == INF:
                return INF
            total += edge_cost
        return total

    def line_of_sight(self, a: Voxel, b: Voxel) -> bool:
        x0, y0, z0 = a
        x1, y1, z1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        dz = abs(z1 - z0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        sz = 1 if z1 > z0 else -1
        x, y, z = x0, y0, z0
        if not (self.in_bounds((x, y, z)) and self.passable((x, y, z))):
            return False
        if dx >= dy and dx >= dz:
            ey = 2 * dy - dx
            ez = 2 * dz - dx
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
            ex = 2 * dx - dy
            ez = 2 * dz - dy
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
            ex = 2 * dx - dz
            ey = 2 * dy - dz
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
        ignored_count = 0
        raw_count = 0

        if obstacles is None:
            return filtered, ignored_count, raw_count

        try:
            iterator = iter(obstacles)
        except TypeError:
            return filtered, 1, raw_count

        for obstacle in iterator:
            raw_count += 1
            voxel = self.normalize_voxel(obstacle)
            if voxel is None or not self.in_bounds(voxel):
                ignored_count += 1
                continue
            filtered.add(voxel)

        return filtered, ignored_count, raw_count

    def validate_plan_inputs(
        self,
        start: Any,
        goal: Any,
        obstacles: Iterable[Any],
    ) -> Tuple[bool, str, Optional[Voxel], Optional[Voxel], Set[Voxel], Dict[str, object]]:
        filtered_obstacles, ignored_count, raw_count = self.filter_obstacles(obstacles)
        metrics = {
            "obstacle_count": len(filtered_obstacles),
            "ignored_obstacle_count": ignored_count,
            "raw_obstacle_count": raw_count,
        }

        if not self.grid_is_valid():
            return False, EMPTY_GRID, None, None, filtered_obstacles, metrics

        start_voxel = self.normalize_voxel(start)
        if start_voxel is None:
            return False, INVALID_START, None, None, filtered_obstacles, metrics

        goal_voxel = self.normalize_voxel(goal)
        if goal_voxel is None:
            return False, INVALID_GOAL, start_voxel, None, filtered_obstacles, metrics

        if not self.in_bounds(start_voxel):
            return False, START_OUT_OF_BOUNDS, start_voxel, goal_voxel, filtered_obstacles, metrics

        if not self.in_bounds(goal_voxel):
            return False, GOAL_OUT_OF_BOUNDS, start_voxel, goal_voxel, filtered_obstacles, metrics

        if start_voxel in filtered_obstacles:
            return False, START_BLOCKED, start_voxel, goal_voxel, filtered_obstacles, metrics

        if goal_voxel in filtered_obstacles:
            return False, GOAL_BLOCKED, start_voxel, goal_voxel, filtered_obstacles, metrics

        return True, OK, start_voxel, goal_voxel, filtered_obstacles, metrics

    def validate_replan_inputs(
        self,
        new_start: Any,
        new_obstacles: Iterable[Any],
    ) -> Tuple[bool, str, Optional[Voxel], Optional[Voxel], Set[Voxel], Dict[str, object]]:
        filtered_obstacles, ignored_count, raw_count = self.filter_obstacles(new_obstacles)
        metrics = {
            "obstacle_count": len(filtered_obstacles),
            "ignored_obstacle_count": ignored_count,
            "raw_obstacle_count": raw_count,
        }

        if not self.grid_is_valid():
            return False, EMPTY_GRID, None, self._goal, filtered_obstacles, metrics

        if self._start is None or self._goal is None:
            return False, REPLAN_NOT_INITIALIZED, None, self._goal, filtered_obstacles, metrics

        start_voxel = self.normalize_voxel(new_start)
        if start_voxel is None:
            return False, INVALID_START, None, self._goal, filtered_obstacles, metrics

        if not self.in_bounds(start_voxel):
            return False, START_OUT_OF_BOUNDS, start_voxel, self._goal, filtered_obstacles, metrics

        if start_voxel in filtered_obstacles:
            return False, START_BLOCKED, start_voxel, self._goal, filtered_obstacles, metrics

        if self._goal in filtered_obstacles:
            return False, GOAL_BLOCKED, start_voxel, self._goal, filtered_obstacles, metrics

        return True, OK, start_voxel, self._goal, filtered_obstacles, metrics

    def _get_g(self, s: Voxel) -> float:
        return self._g.get(s, INF)

    def _get_v(self, s: Voxel) -> float:
        return self._v.get(s, INF)

    def _get_h(self, s: Voxel) -> float:
        if self._goal is None:
            return INF
        if s not in self._h:
            self._h[s] = self.heuristic(s, self._goal)
        return self._h[s]

    def _key(self, s: Voxel) -> float:
        return self._get_g(s) + self._eps * self._get_h(s)

    def _open_insert(self, s: Voxel) -> None:
        key = self._key(s)
        self._open_best[s] = key
        self._counter += 1
        heapq.heappush(self._open, (key, self._counter, s))

    def _cleanup_open(self) -> None:
        while self._open:
            key, _, state = self._open[0]
            best = self._open_best.get(state)
            if best is None or not math.isclose(best, key, rel_tol=1e-12, abs_tol=1e-12):
                heapq.heappop(self._open)
                continue
            break

    def _open_pop(self) -> Optional[Voxel]:
        self._cleanup_open()
        if not self._open:
            return None
        _, _, state = heapq.heappop(self._open)
        self._open_best.pop(state, None)
        return state

    def _open_min_key(self) -> float:
        self._cleanup_open()
        if not self._open:
            return INF
        return self._open[0][0]

    def _update_successors(self, state: Voxel) -> None:
        state_v = self._get_v(state)
        for succ in self.neighbors(state):
            new_g = state_v + self.cost(state, succ)
            if new_g >= self._get_g(succ):
                continue

            self._g[succ] = new_g
            self._parent[succ] = state

            if self._closed_iter.get(succ) == self._search_iteration:
                self._incons.add(succ)
            else:
                self._open_insert(succ)

    def _improve_path(
        self,
        start_time: float,
        max_time_ms: float,
        max_steps: int,
    ) -> Tuple[int, str]:
        expanded = 0

        while self._goal is not None and self._open_min_key() < self._get_g(self._goal):
            if expanded >= max_steps:
                return expanded, MAX_STEPS_REACHED
            if max_time_ms > 0.0 and (time.monotonic() - start_time) * 1000.0 >= max_time_ms:
                return expanded, TIMEOUT

            state = self._open_pop()
            if state is None:
                break

            if self._get_v(state) <= self._get_g(state):
                continue

            self._v[state] = self._get_g(state)
            self._closed_iter[state] = self._search_iteration
            expanded += 1
            self._update_successors(state)

        return expanded, OK

    def _rebuild_open(self) -> None:
        for state in self._incons:
            self._open_insert(state)
        self._incons.clear()

        states = list(self._open_best)
        self._open.clear()
        self._open_best.clear()
        for state in states:
            self._open_insert(state)

    def _reset_search(self, start: Voxel, goal: Voxel, obstacles: Set[Voxel]) -> None:
        self._g.clear()
        self._v.clear()
        self._h.clear()
        self._parent.clear()
        self._closed_iter.clear()
        self._incons.clear()
        self._open.clear()
        self._open_best.clear()

        self._eps = self.epsilon_start
        self._eps_satisfied = INF
        self._search_iteration = 1
        self._start = start
        self._goal = goal
        self._obstacles = set(obstacles)
        self._previous_obstacles = set(obstacles)
        self._last_path = []
        self.last_first_solution_time_ms = None
        self.last_iterations = 0

        self._g[start] = 0.0
        self._parent[start] = None
        self._open_insert(start)

    def _reconstruct_path(self) -> List[Voxel]:
        if self._start is None or self._goal is None:
            return []
        if self._get_g(self._goal) == INF:
            return []

        reverse_path: List[Voxel] = []
        current: Optional[Voxel] = self._goal
        seen: Set[Voxel] = set()
        max_len = max(1, self.size_x * self.size_y * self.size_z)

        for _ in range(max_len):
            if current is None:
                return []
            if current in seen:
                return []
            seen.add(current)
            reverse_path.append(current)
            if current == self._start:
                reverse_path.reverse()
                return reverse_path
            current = self._parent.get(current)

        return []

    def _ara_search(
        self,
        start_time: float,
        max_time_ms: float,
        max_steps: int,
    ) -> Tuple[List[Voxel], bool, str, int]:
        total_expanded = 0
        best_path: List[Voxel] = []
        final_reason = NO_PATH
        first_solution_time_ms: Optional[float] = None
        iterations = 0

        while self._eps >= self.epsilon_final:
            iterations += 1
            self.last_iterations = iterations
            expanded, reason = self._improve_path(
                start_time=start_time,
                max_time_ms=max_time_ms,
                max_steps=max(1, max_steps - total_expanded),
            )
            total_expanded += expanded

            if self._goal is not None and self._get_g(self._goal) < INF:
                path = self._reconstruct_path()
                if path:
                    best_path = path
                    self._last_path = path
                    self._eps_satisfied = self._eps
                    final_reason = OK
                    if first_solution_time_ms is None:
                        first_solution_time_ms = (time.monotonic() - start_time) * 1000.0
                        self.last_first_solution_time_ms = first_solution_time_ms
                else:
                    final_reason = PATH_EXTRACTION_FAILED

            if reason in (MAX_STEPS_REACHED, TIMEOUT):
                if best_path:
                    return best_path, True, OK, total_expanded
                return [], False, reason, total_expanded

            if best_path and self._eps <= self.epsilon_final:
                return best_path, True, OK, total_expanded

            next_eps = max(self.epsilon_final, self._eps - self.epsilon_decay)
            if math.isclose(next_eps, self._eps, rel_tol=1e-12, abs_tol=1e-12):
                break
            self._eps = next_eps
            self._search_iteration += 1
            self._rebuild_open()

            if total_expanded >= max_steps:
                if best_path:
                    return best_path, True, OK, total_expanded
                return [], False, MAX_STEPS_REACHED, total_expanded

            if max_time_ms > 0.0 and (time.monotonic() - start_time) * 1000.0 >= max_time_ms:
                if best_path:
                    return best_path, True, OK, total_expanded
                return [], False, TIMEOUT, total_expanded

            if self._open_min_key() == INF and not best_path:
                break

        self.last_first_solution_time_ms = first_solution_time_ms
        self.last_iterations = iterations
        if best_path:
            return best_path, True, OK, total_expanded
        return [], False, final_reason, total_expanded

    def _make_metrics(
        self,
        start: Optional[Voxel],
        goal: Optional[Voxel],
        success: bool,
        reason: str,
        path: List[Voxel],
        planning_time_ms: float,
        expanded_steps: int,
        obstacle_count: int,
        ignored_obstacle_count: int,
        changed_obstacle_count: int,
        max_steps: int,
        reused_previous_search: bool,
        raw_obstacle_count: int = 0,
    ) -> Dict[str, object]:
        return {
            "algorithm": "ARA*",
            "success": success,
            "reason": reason,
            "path_length": len(path),
            "path_cost": self.path_cost(path),
            "expanded_steps": expanded_steps,
            "epsilon_start": self.epsilon_start,
            "epsilon_final": self.epsilon_final,
            "epsilon_satisfied": self._eps_satisfied if self._eps_satisfied < INF else None,
            "planning_time_ms": planning_time_ms,
            "elapsed_ms": planning_time_ms,
            "open_size": len(self._open_best),
            "incons_size": len(self._incons),
            "obstacle_count": obstacle_count,
            "ignored_obstacle_count": ignored_obstacle_count,
            "raw_obstacle_count": raw_obstacle_count,
            "changed_obstacle_count": changed_obstacle_count,
            "first_solution_time_ms": self.last_first_solution_time_ms,
            "iterations": self.last_iterations,
            "start": start,
            "goal": goal,
            "grid_size": (self.size_x, self.size_y, self.size_z),
            "diagonal": self.diagonal,
            "max_steps": max_steps,
            "max_time_ms": self.max_time_ms,
            "reused_previous_search": reused_previous_search,
        }

    def _finish_result(
        self,
        path: List[Voxel],
        success: bool,
        reason: str,
        metrics: Dict[str, object],
    ) -> PlanResult:
        result = PlanResult(path=path, success=success, reason=reason, metrics=metrics)
        self.last_result = result
        self.last_expanded_steps = int(metrics.get("expanded_steps", 0))
        self.last_elapsed_ms = float(metrics.get("planning_time_ms", 0.0))
        self.last_changed_obstacle_count = int(metrics.get("changed_obstacle_count", 0))
        self.last_ignored_obstacle_count = int(metrics.get("ignored_obstacle_count", 0))
        return result

    def plan_with_info(
        self,
        start: Voxel,
        goal: Voxel,
        obstacles: Iterable[Voxel],
        max_steps: Optional[int] = None,
    ) -> PlanResult:
        started_at = time.monotonic()
        step_limit = self.max_steps if max_steps is None else max(1, int(max_steps))

        (
            valid,
            reason,
            start_voxel,
            goal_voxel,
            filtered_obstacles,
            validation_metrics,
        ) = self.validate_plan_inputs(start, goal, obstacles)

        if not valid:
            planning_time_ms = (time.monotonic() - started_at) * 1000.0
            metrics = self._make_metrics(
                start=start_voxel,
                goal=goal_voxel,
                success=False,
                reason=reason,
                path=[],
                planning_time_ms=planning_time_ms,
                expanded_steps=0,
                obstacle_count=int(validation_metrics.get("obstacle_count", 0)),
                ignored_obstacle_count=int(validation_metrics.get("ignored_obstacle_count", 0)),
                changed_obstacle_count=0,
                max_steps=step_limit,
                reused_previous_search=False,
                raw_obstacle_count=int(validation_metrics.get("raw_obstacle_count", 0)),
            )
            return self._finish_result([], False, reason, metrics)

        assert start_voxel is not None
        assert goal_voxel is not None

        self._reset_search(start_voxel, goal_voxel, filtered_obstacles)
        path, success, reason, expanded_steps = self._ara_search(
            start_time=started_at,
            max_time_ms=self.max_time_ms,
            max_steps=step_limit,
        )
        if not success and reason == OK:
            reason = NO_PATH

        raw_path_length = len(path)
        if self.smooth and path:
            path = self.smooth_path(path)
            self._last_path = path

        planning_time_ms = (time.monotonic() - started_at) * 1000.0
        metrics = self._make_metrics(
            start=start_voxel,
            goal=goal_voxel,
            success=success,
            reason=reason,
            path=path,
            planning_time_ms=planning_time_ms,
            expanded_steps=expanded_steps,
            obstacle_count=len(filtered_obstacles),
            ignored_obstacle_count=int(validation_metrics.get("ignored_obstacle_count", 0)),
            changed_obstacle_count=0,
            max_steps=step_limit,
            reused_previous_search=False,
            raw_obstacle_count=int(validation_metrics.get("raw_obstacle_count", 0)),
        )
        metrics["raw_path_length"] = raw_path_length
        return self._finish_result(path, success, reason, metrics)

    def replan_with_info(
        self,
        new_start: Voxel,
        new_obstacles: Iterable[Voxel],
        max_steps: Optional[int] = None,
    ) -> PlanResult:
        started_at = time.monotonic()
        step_limit = self.max_steps if max_steps is None else max(1, int(max_steps))

        (
            valid,
            reason,
            start_voxel,
            goal_voxel,
            filtered_obstacles,
            validation_metrics,
        ) = self.validate_replan_inputs(new_start, new_obstacles)

        if not valid:
            planning_time_ms = (time.monotonic() - started_at) * 1000.0
            metrics = self._make_metrics(
                start=start_voxel,
                goal=goal_voxel,
                success=False,
                reason=reason,
                path=[],
                planning_time_ms=planning_time_ms,
                expanded_steps=0,
                obstacle_count=int(validation_metrics.get("obstacle_count", 0)),
                ignored_obstacle_count=int(validation_metrics.get("ignored_obstacle_count", 0)),
                changed_obstacle_count=0,
                max_steps=step_limit,
                reused_previous_search=self._start is not None and self._goal is not None,
                raw_obstacle_count=int(validation_metrics.get("raw_obstacle_count", 0)),
            )
            return self._finish_result([], False, reason, metrics)

        assert start_voxel is not None
        assert goal_voxel is not None

        changed_obstacle_count = self._previous_obstacles.symmetric_difference(filtered_obstacles)
        if (
            start_voxel == self._start
            and filtered_obstacles == self._previous_obstacles
            and self._last_path
        ):
            planning_time_ms = (time.monotonic() - started_at) * 1000.0
            metrics = self._make_metrics(
                start=start_voxel,
                goal=goal_voxel,
                success=True,
                reason=OK,
                path=list(self._last_path),
                planning_time_ms=planning_time_ms,
                expanded_steps=0,
                obstacle_count=len(filtered_obstacles),
                ignored_obstacle_count=int(validation_metrics.get("ignored_obstacle_count", 0)),
                changed_obstacle_count=0,
                max_steps=step_limit,
                reused_previous_search=True,
                raw_obstacle_count=int(validation_metrics.get("raw_obstacle_count", 0)),
            )
            metrics["algorithm"] = "ARA*-cached"
            return self._finish_result(list(self._last_path), True, OK, metrics)

        self._reset_search(start_voxel, goal_voxel, filtered_obstacles)
        path, success, reason, expanded_steps = self._ara_search(
            start_time=started_at,
            max_time_ms=self.max_time_ms,
            max_steps=step_limit,
        )
        if not success and reason == OK:
            reason = NO_PATH

        raw_path_length = len(path)
        if self.smooth and path:
            path = self.smooth_path(path)
            self._last_path = path

        planning_time_ms = (time.monotonic() - started_at) * 1000.0
        metrics = self._make_metrics(
            start=start_voxel,
            goal=goal_voxel,
            success=success,
            reason=reason,
            path=path,
            planning_time_ms=planning_time_ms,
            expanded_steps=expanded_steps,
            obstacle_count=len(filtered_obstacles),
            ignored_obstacle_count=int(validation_metrics.get("ignored_obstacle_count", 0)),
            changed_obstacle_count=len(changed_obstacle_count),
            max_steps=step_limit,
            reused_previous_search=True,
            raw_obstacle_count=int(validation_metrics.get("raw_obstacle_count", 0)),
        )
        metrics["raw_path_length"] = raw_path_length
        return self._finish_result(path, success, reason, metrics)

    def plan(self, start: Voxel, goal: Voxel, obstacles: Iterable[Voxel]) -> List[Voxel]:
        return self.plan_with_info(start, goal, obstacles).path

    def replan(self, new_start: Voxel, new_obstacles: Iterable[Voxel]) -> List[Voxel]:
        return self.replan_with_info(new_start, new_obstacles).path
