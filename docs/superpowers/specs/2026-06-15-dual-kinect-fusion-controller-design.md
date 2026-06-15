# Design Spec — Dual-Kinect Fusion + Duty-Cycle Controller

- **Date:** 2026-06-15
- **Module:** Perception fusion / runtime orchestration (logical modules 1, 7)
- **Status:** Approved design, pending implementation plan
- **Replaces:** deleted `scripts/multi_kinect_skeleton_controller.py` (old MODULE_C, 1008-line heavy fusion)

---

## 1. Problem

UR3 HRC currently runs single Kinect (front *or* back) via `system.launch` / `system_back.launch`.
`dual_kinect_system.launch` starts **both** trackers in namespaces `kinect_front` and `kinect_back`,
each publishing its own skeleton, but the fusion node that combined them was removed. Result:
`/human_skeleton_base` has **no publisher**, so downstream (obstacle builder → MoveIt scene → planner)
gets nothing in dual mode.

Goal: combine the two cameras to get **occlusion-robust** human joint coordinates — when the front
view is blocked, the back fills it and vice-versa — then publish the fused skeleton on
`/human_skeleton_base` so the rest of the pipeline runs **identically to single-camera mode**.

Hardware constraint: both Kinects are on one laptop (i5-11500H, RTX 3050, 16 GB). The machine
cannot run both cameras at full FPS simultaneously, so camera load must be managed.

## 2. Decisions (locked with user)

1. **Fusion level: minimal occlusion-fill.** No Kalman, no conflict map, no side-swap, no quality
   scoring (all of which the old 1008-line node had). Per-joint combine only.
2. **FPS handling: adaptive duty-cycle.** A scheduler commands each camera's tracker loop rate at
   runtime. Cameras are never turned off — they switch between HIGH and LOW rate.
3. **Duty trigger: occlusion-driven (hybrid).** Front is the primary (HIGH by default); back is the
   backup (LOW by default). When the front loses tracked joints (occlusion or lost human), the back
   is boosted to HIGH to fill. When the front recovers, the back returns to LOW after a hysteresis
   hold.
4. **Node structure: one node, two internal components** (`Fuser`, `DutyScheduler`). The duty trigger
   is derived from the same decoded per-camera skeletons the fuser uses, so a single node avoids
   duplicate decoding and extra topics.

## 3. Existing facts the design relies on

- Each tracker already TF-transforms its skeleton into the common `target_frame` (`base_link`) and
  publishes it on its namespaced `human_skeleton_base` topic. **Fusion therefore happens in one
  shared frame** — a per-joint combine, no extra TF math in the controller.
- Encoding helpers in `scripts/data_skeleton.py`:
  - `pose_array_to_numeric_joint_dict(msg, fallback_order)` → `{joint_id: (x,y,z)}`, dropping NaN
    / `orientation.w <= 0` slots.
  - `skeleton_dict_to_pose_array(skeleton, frame, stamp, order)` → fixed-slot PoseArray, NaN for
    missing joints.
- **Output schema contract (must be preserved):** the deleted controller decoded each input with
  `pose_array_to_numeric_joint_dict(msg, tracked_joint_ids)` and encoded output as one `Pose` per
  `tracked_joint_ids` (in order). Downstream `skeleton_obstacle_builder.py` decodes
  `/human_skeleton_base` the same way (`pose_array_to_numeric_joint_dict(msg, tracked_joint_ids)`).
  In `dual_kinect_system.launch`, `tracked_joint_ids = 0,11,12,13,14,15,16,23,24` (9 = POSE_LANDMARK_IDS).
  The new controller MUST reproduce this decode/encode contract exactly so downstream is byte-compatible.
- The tracker sets its loop rate once from `~rate_hz` at init (`spin()` uses `rospy.Rate(self.rate_hz)`).
  There is currently **no** runtime rate control.

## 4. Architecture

New additive node: `scripts/dual_kinect_fusion_controller.py`.

```
/kinect_front/human_skeleton_base (PoseArray) ─┐
                                               ├─►┌──────────────────────────────┐
/kinect_back/human_skeleton_base  (PoseArray) ─┘  │ dual_kinect_fusion_controller │
                                                  │  ┌──────────┐  ┌────────────┐ │
                                                  │  │  Fuser   │  │DutyScheduler│ │
                                                  │  └──────────┘  └────────────┘ │
                                                  └──────────────────────────────┘
        ┌──────────────────────────┬──────────────────────────┬───────────────────────┐
        ▼                          ▼                          ▼                       ▼
 /human_skeleton_base    /human_skeleton_fusion_status  /kinect_front/tracker   /kinect_back/tracker
 (PoseArray, base_link)  (String: mode + duty state)    _rate_cmd (Float32)     _rate_cmd (Float32)
        │
        ▼  (unchanged downstream)
 skeleton_obstacle_builder → moveit_scene_manager → planner_ab_replan_node → UR3
```

- **Trigger model:** timer-driven at `fusion_rate_hz` (~15–20 Hz). Each input callback only caches the
  latest message + arrival time per camera; the timer reads the caches, fuses, runs the scheduler,
  and publishes. This decouples output cadence from the two asymmetric input rates.
- Single-camera launches (`system.launch`, `system_back.launch`) do **not** run this node and are
  unaffected.

## 5. Component: Fuser (minimal occlusion-fill)

Inputs: latest cached front skeleton and back skeleton (numeric joint dicts in `base_link`), each
with an arrival timestamp. Per tracked joint id:

| Front joint | Back joint | Fused output |
|-------------|------------|--------------|
| valid + fresh | valid + fresh | average of the two **unless** `dist(front,back) > max_merge_dist` → take front (primary) instead of averaging across a bad reading |
| valid + fresh | stale / missing | front |
| stale / missing | valid + fresh | **back (occlusion fill)** |
| stale | stale | drop joint (NaN in output) |

- **Freshness** per camera = `now - arrival_time <= max_input_age_sec`. A whole stale camera
  contributes no joints.
- All joints dropped (both cameras stale / empty) → publish an **empty skeleton** (all NaN). This lets
  the existing downstream obstacle-removal timeout clear the collision object — no stale ghost obstacle.
- No Kalman, no smoothing, no side-swap, no conflict map. The only guard is the cheap
  `max_merge_dist` rule above.
- Output is encoded with the **preserved schema contract** (Section 3): one pose per `tracked_joint_ids`.

## 6. Component: DutyScheduler (occlusion-driven)

State machine over the **front** camera's coverage of the tracked joint set:

- On startup the scheduler commands `front = HIGH` once.
- Default steady state: `front = HIGH`, `back = LOW` (backup, never off). The front stays HIGH for the
  whole of this scope; `~front_low_rate` is defined but **reserved** for the future idle-throttle
  enhancement (Section 13) and is not commanded here.
- Compute `front_missing = |tracked_set| - (count of front's valid+fresh tracked joints)`.
- **Boost rule:** if `front_missing >= miss_threshold` (or front camera entirely stale) sustained for
  `confirm_frames` consecutive ticks → command `back = HIGH`.
- **Release rule:** once front coverage is full again, hold `back = HIGH` for `boost_hold_sec`
  (hysteresis to prevent flapping), then command `back = LOW`.
- Rate commands are published per camera **only on change** (idempotent; a `Float32` of the target
  Hz on each `tracker_rate_cmd` topic).

Default rates (params, tune on real hardware):

| Camera | LOW | HIGH | Note |
|--------|-----|------|------|
| front  | 4   | 10   | rgb-only + CUDA YOLO async, primary |
| back   | 2   | 6    | depth + CPU pipeline, HIGH capped lower |

The front is normally kept HIGH (it is the primary); the duty-cycle mainly governs the **back** to
save CPU while idle and spend it only when the front needs occlusion fill. (A future option — also
dropping the front to LOW when the human is far/idle — is out of scope for this spec.)

## 7. Tracker change (MODULE_A — additive, justified)

The duty-cycle requires **runtime** rate control, which the tracker does not have. Minimal additive
change to `kinect_skeleton_tracker.py`:

- New param `~rate_cmd_topic` (default empty / unset).
- New params `~rate_min` (default 1.0) and `~rate_max` (default 60.0) to clamp commands.
  (60.0 ensures existing single-camera launches with `~rate_hz=30` are not clamped down.)
- If `~rate_cmd_topic` is set, subscribe to it (`Float32`); on message, clamp to `[rate_min, rate_max]`
  and update the value driving the loop `Rate` in `spin()`.
- If unset (single-camera launches, or dual launch before wiring), behaviour is **identical to today**
  — the launch `~rate_hz` is used and no external control exists.

This is purely additive: no existing topic, param, node name, or message type changes. Safety:
the clamp prevents a 0 Hz (stall) or runaway rate command.

## 8. ROS interfaces (new — documented per AGENTS rule 7)

**Published by controller:**

- `/human_skeleton_base` — `geometry_msgs/PoseArray`, frame `base_link`. **Unchanged contract**;
  downstream byte-compatible with old fused output.
- `/human_skeleton_fusion_status` — `std_msgs/String`. Mode in
  `{BOTH, FILL_FROM_BACK, FRONT_ONLY, BACK_ONLY, NO_INPUT}` plus duty state (`BACK_LOW` / `BACK_HIGH`)
  and per-camera joint counts. Debug/observability only.
- `/kinect_front/tracker_rate_cmd`, `/kinect_back/tracker_rate_cmd` — `std_msgs/Float32`, target Hz.

**Subscribed by controller:**

- `/kinect_front/human_skeleton_base`, `/kinect_back/human_skeleton_base` — `PoseArray`.

**New on tracker (MODULE_A):**

- `~rate_cmd_topic` subscriber — `std_msgs/Float32`.

**Controller params:**

| Param | Default | Purpose |
|-------|---------|---------|
| `~front_skeleton_base_topic` | `/kinect_front/human_skeleton_base` | input |
| `~back_skeleton_base_topic` | `/kinect_back/human_skeleton_base` | input |
| `~output_skeleton_base_topic` | `/human_skeleton_base` | fused output |
| `~fusion_status_topic` | `/human_skeleton_fusion_status` | status |
| `~front_rate_cmd_topic` | `/kinect_front/tracker_rate_cmd` | duty cmd |
| `~back_rate_cmd_topic` | `/kinect_back/tracker_rate_cmd` | duty cmd |
| `~tracked_joint_ids` | `0,11,12,13,14,15,16,23,24` | fusion + encode order |
| `~target_frame` | `base_link` | output frame |
| `~fusion_rate_hz` | `20.0` | output timer |
| `~max_input_age_sec` | `0.35` | per-cam staleness |
| `~max_merge_dist` | `0.20` (m) | average-vs-prefer-front guard |
| `~miss_threshold` | `2` | front missing joints → boost back |
| `~confirm_frames` | `3` | sustained ticks before boost |
| `~boost_hold_sec` | `1.5` | hysteresis before back → LOW |
| `~front_high_rate` | `10` | front rate (steady HIGH this scope) |
| `~front_low_rate` | `4` | reserved (future idle-throttle, Section 13) |
| `~back_high_rate` / `~back_low_rate` | `6` / `2` | back duty rates |
| `~empty_publish_on_no_input` | `true` | publish NaN skeleton when both stale |

## 9. Safety (AGENTS rule 10 — preserved / strengthened)

- Occlusion-fill **never invents** joints; it only forwards real per-camera observations.
- Both-stale → empty (NaN) skeleton → existing `timeout_remove_sec` in obstacle builder clears the
  obstacle. No stale ghost obstacle is left in the MoveIt scene.
- Duty-cycle keeps both cameras at ≥ LOW rate, so dual-view occlusion coverage is always alive.
- Tracker rate command is clamped to `[rate_min, rate_max]` — no stall, no runaway.
- `max_merge_dist` guard prevents averaging the human position across a divergent (likely wrong)
  reading from one camera.
- No existing safety logic (human filtering, padding, scene confirmation, ARA* guard, replan guards)
  is touched — those all live downstream of `/human_skeleton_base` and see the same message type.

## 10. Module boundaries (AGENTS rule 5)

- Controller does perception **fusion** + rate **orchestration** only. It does not control robot
  execution, build collision objects, or run MoveIt.
- Fuser and DutyScheduler are separate classes with one responsibility each; the node wires them.
- Tracker change is confined to acquisition rate; it adds no fusion logic.

## 11. Launch impact — 🔒 GATED

`dual_kinect_system.launch` must be updated to (a) re-add the controller node, (b) pass the rate-cmd
topic params to each tracker, (c) wire controller params. **This is a protected launch file.** No edit
will be made without explicit user confirmation (AGENTS rule 4 / CLAUDE Launch File Protection).
The implementation plan will produce the launch change as a **markdown proposal** first and stop for
confirmation. Single-camera launches are not modified.

## 12. Files and docs affected

- **Add:** `scripts/dual_kinect_fusion_controller.py`.
- **Modify:** `scripts/kinect_skeleton_tracker.py` (runtime rate subscriber — additive).
- **Launch (gated):** `launch/dual_kinect_system.launch` (proposal only until confirmed).
- **Docs to update in the same change set:**
  - `docs/MODULE_C_multi_kinect_skeleton_controller.md` → rewrite for the new controller (rename
    intent noted; keep or supersede old MODULE_C).
  - `docs/MODULE_A_kinect_skeleton_tracker.md` → document `~rate_cmd_topic`, `~rate_min/max`.
  - `PROJECT_STRUCTURE.md` → new file entry, dual-kinect block diagram, topics/TF, launch section.
  - `docs/SYSTEM_FLOW_DIAGRAMS.md`, `docs/MODULES_INDEX.md` → reflect new node + interfaces.

## 13. Out of scope (YAGNI)

- Kalman filtering, conflict maps, side-swap detection, quality scoring (deliberately dropped).
- Front-camera idle throttling based on human proximity (possible later enhancement).
- Eye-to-hand recalibration (TF static transforms stay as-is in launch).
- Any change to downstream obstacle / MoveIt / planner logic.

## 14. Validation (to run during implementation)

- Unit: Fuser table cases (both-fresh average, merge-distance prefer-front, single-cam fill, both-stale
  empty) and DutyScheduler transitions (boost on sustained miss, hysteresis release).
- Schema regression: fused `/human_skeleton_base` decodes identically via
  `pose_array_to_numeric_joint_dict(msg, tracked_joint_ids)` (extend existing
  `test/test_pose_array_schema.py`).
- Integration (manual, after launch confirmation): occlude front by hand → confirm back boosts to HIGH
  and occluded joints persist in `/human_skeleton_base`; confirm single-cam launches still run unchanged.
```
