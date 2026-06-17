# Speed-Scaled A* Obstacle Padding + In-Motion Reroute — Design

- Date: 2026-06-17
- Module: Planner / A-B trajectory / replan (`scripts/planner_ab_replan_node.py`, MODULE_F)
- Status: Approved design, pending implementation plan

## Problem

Two gaps in the current planner:

1. **No speed-dependent obstacle padding.** The A* obstacle inflation in
   `active_human_voxels()` uses a fixed radius (`human_inflate_radius` and
   `base_clearance`). It does not grow when the robot moves faster, so the
   planner routes the same distance from a human at 10% speed and at 50% speed.
   Faster motion needs more clearance (stopping-distance buffer).

2. **No mid-segment reroute.** The A* guard + `trajectory_is_safe` run only at
   plan time, before each waypoint. During execution only the emergency-stop
   distance check runs (`execute_plan` monitor loop). If a human moves onto the
   upcoming part of the path after a segment has started, the robot does not
   re-evaluate the route until the segment ends (or it emergency-stops). The
   obstacle must already be near the waypoint target at plan time to be caught.

The robot stops at each of the 7 A→B waypoints, so padding evaluated only at
plan time (robot stationary, speed ≈ 0) would almost never fire — the two gaps
are coupled and must be solved together.

## Goal

While the robot is moving, scale the A* obstacle inflation by the **measured
live TCP speed**, re-check the route in motion, and reroute — gracefully when
there is room, with a hard stop when too close.

## Scope

In scope:

- Speed-scaled inflation of A* human voxels (`active_human_voxels`).
- Live TCP speed measurement in the execution monitor loop.
- Two-tier in-motion reaction (graceful reroute vs hard stop).
- Reuse of existing detour + breadcrumb machinery for the replan.

Out of scope:

- MoveIt collision-object (PlanningScene) radius scaling — `active_human_voxels`
  is the only target. The `skeleton_obstacle_builder` collision radii are
  unchanged.
- `trajectory_is_safe` / execution clearance margin redesign — the existing
  `execution_speed_safety_margin` stays as is.
- `moveit_servo` / live trajectory blending. MoveIt cannot blend trajectories;
  a trajectory swap requires a momentary stop. We do not pull in servoing.

## Key Constraint: "Plan While Moving"

`group.execute(plan, wait=False)` streams a fixed trajectory to the UR driver.
To follow a different path the node must `group.stop()` then `execute(new_plan)`,
and `new_plan` must start from the robot's current (rest) state. There is no
live blend without `moveit_servo`.

So "plan while moving" splits into:

- **Decision** — the A* re-check (current TCP voxel → goal voxel, speed-inflated
  obstacles, ~50 ms). Runs while the robot keeps moving. No stop.
- **Swap** — replacing the trajectory. Requires a momentary stop, because the
  new MoveIt plan starts from rest.

**Chosen swap timing (Tier 1, graceful): swap at next waypoint.** Decide in
motion, keep moving to the immediate next (short) sub-waypoint, replan from
there. With fine waypoints the swap-stop coincides with a stop the robot was
going to make anyway → approximates "plan while moving" with no extra dwell.
Reroute is delayed by at most one short segment.

## Design

### 1. Live TCP speed

In the `execute_plan` execution-monitor loop (already runs at
`execution_monitor_rate`, default 20 Hz, already fetches the current pose), the
node computes TCP speed from successive end-effector position deltas divided by
the loop dt, then EMA-smooths it:

```
speed_raw = dist(pos_now, pos_prev) / dt
speed = alpha * speed_prev + (1 - alpha) * speed_raw      # alpha = smoothing
self._current_tcp_speed = speed
```

Stored on `self._current_tcp_speed`. Reset to 0 when not executing. One pose
fetch per tick (reuse the pose already read for emergency-stop / breadcrumb).

### 2. Speed → padding

New parameters (all default to **off**, so behavior is identical to today until
tuned on hardware — this is the calibration knob):

| Param | Default | Meaning |
| --- | --- | --- |
| `astar_speed_padding_gain` | `0.0` | metres of padding per (m/s) above deadband |
| `astar_speed_padding_deadband_mps` | `0.05` | speed below which padding is 0 |
| `astar_max_speed_padding_m` | `0.05` | hard cap on padding (metres) |
| `astar_speed_padding_smoothing_alpha` | `0.7` | EMA smoothing on measured speed |

```
pad_m      = clamp(gain * (speed - deadband), 0.0, max_pad)
pad_voxels = ceil(pad_m / voxel_size)
```

`gain = 0.0` → `pad_m = 0` always → padding disabled.

### 3. Inflate A* obstacles

`active_human_voxels()` inflation radius becomes:

```
radius = max(human_inflate_radius, ceil(base_clearance / voxel_size)) + pad_voxels
```

`pad_voxels` comes from the **effective** speed padding (see latching, §5).
At plan time with the robot at rest and no latch, `pad_voxels = 0` → unchanged
(backward compatible). The bone-obstacle inflation (`_bone_obstacle_voxels`)
keeps its own `bone_inflate_radius` and is not speed-scaled in this change.

### 4. Two-tier in-motion re-check

The segment goal voxel is stored on `self` before `execute` (e.g.
`self._exec_goal_voxel`). In the monitor loop, every `astar_recheck_period_sec`
(new param, default `0.2` s):

- **Tier 2 — too close (hard stop):** the existing emergency-stop distance
  check, using the speed-scaled clearance already present
  (`base_clearance() + execution_speed_safety_margin`). On violation:
  `group.stop()`, publish `EXECUTION_STOPPED_HUMAN_TOO_CLOSE`, return `ABORT`.
  This is the safety floor and is unchanged in spirit.

- **Tier 1 — route ahead blocked (graceful):** re-run A* from the current TCP
  voxel → `self._exec_goal_voxel` (the current segment's goal) with the
  speed-inflated obstacles.
  - If A* finds a path → continue.
  - If A* is blocked **and** Tier 2 is not violated → **do not stop**. Latch the
    current `pad_voxels` into `self._reroute_pad_voxels`, set
    `self._reroute_pending = True`, publish `ASTAR_EXEC_REPLAN`, and let the
    current (short) segment finish to its waypoint. The reroute is applied to
    the **next** leg's plan, which routes around the latched speed-inflated
    obstacles. Tier 2 remains the in-segment safety net while the robot finishes
    this short hop.

Control-flow note: each `execute_plan` drives one whole segment (current pose →
one waypoint). There is no sub-waypoint inside a segment. "Swap at next
waypoint" therefore means: finish the current short segment, then the next
`run_waypoint` → `plan_to_joint_map` reroutes using the latch. Fine waypoints
(`resampled_waypoint_count`) keep each segment short so this delay and the
swap-stop are small, and so a blocked-route segment is brief enough that Tier 2
covers it.

A* re-check cost is bounded by `ara_max_time_ms` (50 ms) and runs at ~5 Hz, so
it overlaps motion cheaply.

### 5. Latch in-motion speed for the rest-replan

Because Tier 1 replans at the next waypoint (robot at rest, measured speed ≈ 0),
the in-motion padding would otherwise vanish. The Tier-1 flag latches the
padding measured in motion:

- `self._reroute_pad_voxels` holds the latched `pad_voxels`.
- `active_human_voxels()` uses `max(live pad_voxels, latched pad_voxels)` while a
  reroute is pending, so the rest-replan's A* guard sees the speed-inflated
  obstacles.
- The latch is cleared once the replan for that waypoint completes (success or
  hard fail).

This honours the "measured TCP speed" decision: measured in motion, applied to
both the trigger and the reroute margin.

### 6. Control flow integration

`execute_plan` returns a small status instead of a bare bool:

- `SUCCESS` — segment finished.
- `FAILED` — MoveIt execute returned false / hard failure (today's `False`).
- `ABORT` — Tier-2 emergency stop (today's emergency-stop `False`).
- `REPLAN` — Tier-1 reroute pending; segment finished at the waypoint and the
  next plan should route around the latched obstacles.

`run_waypoint`:

- On `SUCCESS` → proceed.
- On `REPLAN` → loop and replan the **same** upcoming waypoint with the latched
  speed-inflated obstacles, reusing the existing breadcrumb warm-start hop and
  detour machinery (`_try_breadcrumb_hop`, `run_detour_if_hand_blocks_waypoint`).
- On `ABORT` / `FAILED` → behave as today (waypoint fails → cycle retries from A
  after the existing 2 s wait).

A minimal enum or string constants are acceptable; the existing call sites that
check `if not ok` map to `status != SUCCESS`.

### 7. Finer waypoints / memorized path

No new code:

- Finer waypoints: raise `resampled_waypoint_count` (already wired through
  `resample_joint_path`). Shorter segments → faster Tier-1 decisions and a
  smaller swap-stop. Documented as the knob to turn.
- Memorized path: the existing `BreadcrumbCache` records traversed poses and
  warm-starts the replan; reused unchanged.

## Safety

The change is purely additive:

- Existing emergency stop (Tier 2) is preserved and now also acts on the
  in-motion A* re-check.
- `trajectory_is_safe`, detour, breadcrumb, and MoveIt scene gating are
  unchanged.
- The new A* re-check is an extra guard that can only stop or reroute the robot,
  never relax an existing check.
- Default `astar_speed_padding_gain = 0.0` → no behavioural change until
  explicitly tuned.

Validation:

- Static: `python3 -m py_compile scripts/planner_ab_replan_node.py`.
- Unit self-check: a small `demo()`/`__main__`-style assert test for the pure
  speed→padding math (`clamp(gain*(speed-deadband),0,max)` and the voxel
  conversion), runnable without ROS.
- Behavioural: with `gain=0`, confirm `active_human_voxels` output is identical
  to current (regression). With `gain>0` and a synthetic speed, confirm
  `pad_voxels` grows and `active_human_voxels` set expands.

## ROS Interfaces

No new topics, services, params-with-remaps, frames, or node names.

- New status strings on the existing `/ur3_fixed_joint_path_status`:
  `ASTAR_EXEC_REPLAN` (Tier-1 reroute pending). Existing
  `EXECUTION_STOPPED_HUMAN_TOO_CLOSE` reused for Tier-2.
- New private params (read with `rospy.get_param`, all defaulted in code):
  `astar_speed_padding_gain`, `astar_speed_padding_deadband_mps`,
  `astar_max_speed_padding_m`, `astar_speed_padding_smoothing_alpha`,
  `astar_recheck_period_sec`.

**Launch files: no edits required.** Params have code defaults; the feature is
off by default. Enabling it later by adding params to `system.launch` /
`system_back.launch` / `dual_kinect_system.launch` is optional and requires
explicit user confirmation before any `.launch` edit.

## Files Affected

- `scripts/planner_ab_replan_node.py` — speed measurement, padding math,
  `active_human_voxels` speed term, `execute_plan` status + Tier-1/Tier-2
  re-check, `run_waypoint` REPLAN handling, latch state.
- `docs/MODULE_F_planner_ab_replan_node.md` — new params, statuses, behaviour.
- `ur3_start_checklist.md` — add `ASTAR_EXEC_REPLAN` and the speed-padding
  params to the status / config references.
- `PROJECT_STRUCTURE.md` — no change (no files added/removed/renamed).

## Open Risks / TODO

- Speed-padding defaults (gain, deadband, cap) are placeholders; they must be
  tuned on the real robot (hardware calibration knob, not a model-derived
  value). The 10%→0 / 50%→3-5 cm target is the tuning anchor.
- TCP speed from pose deltas is sensitive to the monitor loop jitter; EMA
  smoothing mitigates it. If noisy on hardware, fall back to joint-velocity ×
  Jacobian (deferred until measured).
- Tier-1 reroute delay equals one sub-waypoint segment; if too slow, raise
  `resampled_waypoint_count` or revisit option B (immediate stop-and-swap).
