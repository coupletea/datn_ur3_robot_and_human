# LPA* 3D guard planner — design spec

> Date: 2026-06-20
> Module: G (TCP voxel guard planner) + F wiring
> Status: approved design, ready for implementation plan
> Related: `docs/MODULE_G_LPA_star_3d_design.md` (original long proposal), `scripts/astar_improved_3d.py`, `scripts/planner_ab_replan_node.py`

## 1. Goal

Add an incremental TCP voxel guard planner **LPA\* (Lifelong Planning A\*)** as an
alternative to the existing `AStarImproved3D` (ARA\*). LPA\* reuses the previous
search and repairs only the affected cells when the human obstacle voxels change
locally, instead of replanning from scratch every frame.

Hard constraints:

- Keep `scripts/astar_improved_3d.py` unchanged (baseline / fallback).
- Add a new file `scripts/astar_lpa_3d.py`.
- Select planner via a config param; default keeps current behavior.
- LPA\* is **only** the TCP voxel guard. Whole-arm collision validation stays in
  MoveIt `/check_state_validity`, `group.plan()`, `trajectory_is_safe()`, and the
  execute-time monitor. Never execute on LPA\* output alone.

Non-goals for v1 (explicitly deferred): shadow mode, benchmark/replay harness,
weighted-ε tuning, D\* Lite, start-on-path repair, the ~20-field metrics set.

## 2. Decisions (locked)

| # | Decision | Choice |
|---|---|---|
| D1 | Public API | **Drop-in, identical to `AStarImproved3D`** (incl. `set_penalty_cells`). `replan_with_info` keeps goal internally — zero call-site changes in Module F. |
| D2 | v1 scope | **Minimal ship**: LPAStar3D + selection param + offline assert self-test. |
| D3 | Algorithm | LPA\* forward from start, ε = 1.0 (optimal). Weighted-ε / D\* Lite are later versions. |
| D4 | Default planner | `ara_star` — opt-in to `lpa_star`. No behavior change unless selected. |
| D5 | Launch | No `.launch` edits. New params have code defaults; enabling on hardware is a later, user-confirmed launch change. |

## 3. File & class

New file `scripts/astar_lpa_3d.py`. Reuse shared **module-level** types/constants by
importing from `astar_improved_3d`: `Voxel`, `PlanResult`, reason constants, `INF` —
these are already module-level, so the import needs **no edit** to `astar_improved_3d.py`.
Geometry shared with ARA\* (`line_of_sight`, `smooth_path`, neighbor offsets) lives as
methods on `AStarImproved3D`; LPAStar3D keeps its own thin copies of these so the
baseline file stays untouched (D5 / §1). Lifting them to shared helpers is a possible
later cleanup, not part of v1.

### Public API (drop-in)

```python
class LPAStar3D:
    def __init__(self, size_x, size_y, size_z, diagonal=True,
                 max_time_ms=50.0, max_steps=50000, smooth=True,
                 epsilon=1.0,
                 start_reuse_radius_voxels=1,
                 max_changed_obstacles_for_repair=500): ...

    def plan_with_info(self, start, goal, obstacles, max_steps=None) -> PlanResult: ...
    def replan_with_info(self, new_start, new_obstacles, max_steps=None) -> PlanResult: ...
    def plan(self, start, goal, obstacles) -> List[Voxel]: ...
    def replan(self, new_start, new_obstacles) -> List[Voxel]: ...
    def set_penalty_cells(self, cells, weight) -> None: ...
```

`replan_with_info` carries `goal` from the prior `plan_with_info` (same semantics as
ARA\*). Module F already routes goal-change → `plan_with_info`, same-goal →
`replan_with_info`, so no call site changes.

## 4. Internal state

```python
self._g, self._rhs: Dict[Voxel, float]
self._parent: Dict[Voxel, Optional[Voxel]]
self._open: List[Tuple[Tuple[float, float], int, Voxel]]   # heap
self._open_best: Dict[Voxel, Tuple[float, float]]          # lazy-delete bookkeeping
self._counter: int
self._start, self._goal: Optional[Voxel]
self._obstacles, self._previous_obstacles: Set[Voxel]
self._penalty_cells: Set[Voxel]
self._penalty_weight: float
self._last_path: List[Voxel]
```

## 5. Core functions (standard LPA*)

- `calculate_key(s)` → `(min(g,rhs) + epsilon * h(s, goal), min(g,rhs))`.
- `heuristic(a, b)` — same metric as `AStarImproved3D` (admissible, ε=1 ⇒ optimal).
- `cost(a, b)` — base step cost (`1 / √2 / √3` per diagonal move) `+ penalty_weight`
  when `b in penalty_cells`. Penalty folded into edge cost; penalty cells are static
  (set once at startup), so they do not complicate repair.
- `neighbors(s)` — passable, in-bounds (for path movement).
- `raw_neighbors(s)` — in-bounds ignoring obstacles (for change propagation).
- `update_vertex(u)`: if `u != start`, `rhs(u) = min over preds p of g(p)+cost(p,u)`
  and set `parent(u)` to the argmin; remove `u` from OPEN; if `g(u) != rhs(u)`,
  push with `calculate_key(u)`.
- `compute_shortest_path()`: pop while
  `OPEN.top_key < calculate_key(goal)` **or** `rhs(goal) != g(goal)`.
  Overconsistent (`g > rhs`) → `g = rhs`, update successors.
  Underconsistent (`g <= rhs`) → `g = INF`, update self + successors.
  Honor `max_time_ms` → `TIMEOUT`, `max_steps` → `MAX_STEPS_REACHED`.
- `update_obstacles(new)`: `changed = previous_obstacles XOR new`; for each changed
  cell and its `raw_neighbors`, call `update_vertex`. Record `changed_obstacle_count`
  and `affected_vertex_count`.
- `reconstruct_path()`: walk `parent` from goal to start; reverse. Detect parent
  loop / a voxel inside an obstacle / over-length walk → `PATH_EXTRACTION_FAILED`.
- `smooth_path()` — LPAStar3D's own copy of the ARA\* smoothing logic, applied when
  `smooth=True` (keeps `astar_improved_3d.py` untouched per §1).

## 6. Reset vs repair vs cache

Decided at the top of `replan_with_info` (`plan_with_info` always resets). Goal
changes are routed to `plan_with_info` by Module F, so `replan_with_info` only sees
the same goal.

| Condition | Action (`reuse_mode`) |
|---|---|
| first call / not initialized | validation returns `REPLAN_NOT_INITIALIZED` |
| `new_start != self._start` (any move) | `RESET` (start re-root deferred to v2; RESET is always correct) |
| `changed_obstacle_count > max_changed_obstacles_for_repair` | `RESET` (repair costlier than reset) |
| same start, diff within threshold | `REPAIR`: `update_obstacles`, `compute_shortest_path` |

**v1 simplifications** (ponytail, vs the original long proposal): no path **CACHE**
mode — a no-change replan just re-runs `compute_shortest_path`, which returns
immediately when everything is consistent. No start re-root — any start movement
RESETs. `start_reuse_radius_voxels` is accepted but reserved for v2.

`initialize_search(start, goal, obstacles)` clears `g/rhs/parent/open`, sets
`rhs(start)=0`, `g(start)=INF`, pushes start.

## 7. Safety & fallback

LPA\* must never return a fake `success=True`. On internal anomaly — parent loop,
OPEN exhausted while goal still inconsistent, reconstructed path crossing an
obstacle — log `LPA_INTERNAL_ERROR`, perform **one** `initialize_search` retry, and
if it still fails return `success=False`. Module F then handles failure exactly as
today (detour → HOLD → bounded retry).

Downstream gates are unchanged and remain authoritative:
`goal_state_in_collision(joint_map)` → MoveIt `group.plan()` →
`trajectory_is_safe(plan)` → execute-time clearance monitor. LPA\* answers only
"does a TCP voxel path exist?".

## 8. Module F wiring (code only, no launch)

In `__init__`:

```python
self.guard_planner_type = rospy.get_param("~guard_planner_type", "ara_star")  # "ara_star" | "lpa_star"
self.lpa_epsilon = float(rospy.get_param("~lpa_epsilon", 1.0))
self.lpa_start_reuse_radius_voxels = int(rospy.get_param("~lpa_start_reuse_radius_voxels", 1))
self.lpa_max_changed_obstacles_for_repair = int(rospy.get_param("~lpa_max_changed_obstacles_for_repair", 500))
```

At the planner construction site (`planner_ab_replan_node.py:620`):

```python
if self.guard_planner_type == "lpa_star":
    self.astar = LPAStar3D(
        size_x=self.map_size_x, size_y=self.map_size_y, size_z=self.map_size_z,
        diagonal=True, max_time_ms=self.ara_max_time_ms, max_steps=self.ara_max_steps,
        epsilon=self.lpa_epsilon,
        start_reuse_radius_voxels=self.lpa_start_reuse_radius_voxels,
        max_changed_obstacles_for_repair=self.lpa_max_changed_obstacles_for_repair,
    )
else:
    self.astar = AStarImproved3D(...)  # unchanged
```

New import `from astar_lpa_3d import LPAStar3D`. All other call sites
(`plan_with_info`, `replan_with_info`, `set_penalty_cells`) stay identical because
the API is drop-in. `self.astar` name kept.

## 9. Metrics (minimal)

`PlanResult.metrics` for LPA\*:

```python
{
  "algorithm": "LPA*",
  "success": bool,
  "reason": str,
  "path_length": int,
  "expanded_steps": int,
  "planning_time_ms": float,
  "obstacle_count": int,
  "changed_obstacle_count": int,
  "reuse_mode": "RESET" | "REPAIR",
}
```

External reason codes match `AStarImproved3D` (reuse existing constants). No new
reason strings leak to Module F; the reset/repair distinction lives in `reuse_mode`.

**Known limitation:** unlike ARA\* (anytime), LPA\* on budget exhaustion returns
`TIMEOUT` with **no** path rather than a best-so-far path. With a large grid,
`~ara_max_time_ms` must be set generously. REPAIR over a large obstacle diff can
cost more than a fresh RESET — hence the `max_changed_obstacles_for_repair`
threshold (calibrate on the real grid).

## 10. Test (offline, no ROS)

`python3 scripts/test_astar_lpa_3d.py`, assert-based (separate test file, not a
`--selftest` flag — avoids production importing test code):

1. Empty small grid → path found; `path[0]==start`, `path[-1]==goal`, every step is a
   valid neighbor, no voxel in obstacle set.
2. Static obstacle → path avoids it; **path cost equals `AStarImproved3D` cost** on
   the same input (ε=1.0 optimality cross-check).
3. Start blocked / goal blocked → correct reason (`START_BLOCKED` / `GOAL_BLOCKED`).
4. Repair: plan, shift one obstacle voxel, `replan_with_info` → valid path and
   `reuse_mode == "REPAIR"`.
5. Large obstacle diff over the threshold → `reuse_mode == "RESET"`.

## 11. Out of scope (later versions)

- Shadow mode (run both, log `GUARD_DISAGREE`).
- Benchmark/replay harness (CSV: planning_time, expanded_steps, reuse_mode...).
- Weighted LPA\* (ε > 1) and Anytime variants.
- D\* Lite (backward search anchored at goal) for continuously moving start.
- Start-on-path incremental reuse.

## 12. Files touched

- Add: `scripts/astar_lpa_3d.py`, `scripts/test_astar_lpa_3d.py`.
- Edit: `scripts/planner_ab_replan_node.py` (import + factory + 4 params + loginfo).
- `scripts/astar_improved_3d.py`: **not edited** in v1 (import-only reuse of its
  module-level symbols).
- Docs: update `PROJECT_STRUCTURE.md` (new file) and `docs/MODULE_G_*` after code.
