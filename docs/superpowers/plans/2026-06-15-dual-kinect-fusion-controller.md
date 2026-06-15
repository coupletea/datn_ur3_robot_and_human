# Dual-Kinect Fusion + Duty-Cycle Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-add a dual-Kinect controller that fuses the front/back skeleton streams into `/human_skeleton_base` (occlusion-robust) and adaptively duty-cycles the back camera to manage laptop CPU load.

**Architecture:** One new node `dual_kinect_fusion_controller.py` with two pure components — `SkeletonFuser` (per-joint minimal occlusion-fill) and `DutyScheduler` (occlusion-driven HIGH/LOW rate commands). Both Kinect trackers already publish skeletons in `base_link`, so fusion is a per-joint combine. A small additive change to `kinect_skeleton_tracker.py` lets the scheduler change a tracker's loop rate at runtime via a `Float32` topic.

**Tech Stack:** ROS Noetic (rospy), `geometry_msgs/PoseArray`, `std_msgs/{Float32,String}`, Python 3, `unittest`. Reuses `scripts/data_skeleton.py` encode/decode helpers.

**Spec:** `docs/superpowers/specs/2026-06-15-dual-kinect-fusion-controller-design.md`

**Repo notes:**
- `test/` and `docs/` are gitignored — write/run tests and docs locally, but **do not** `git add` them. Commits include only `scripts/*` and `CMakeLists.txt`.
- Run a single test: `python3 -m pytest test/test_dual_kinect_fusion_controller.py -v` (ROS env must be sourced so `geometry_msgs` imports).
- `launch/dual_kinect_system.launch` is a **protected** file — Task 8 produces a markdown proposal only and stops for user confirmation.

---

## File Structure

- **Create** `scripts/dual_kinect_fusion_controller.py` — the node + `SkeletonFuser` + `DutyScheduler`.
- **Modify** `scripts/kinect_skeleton_tracker.py` — add `clamp_rate()` helper, runtime rate subscriber, dynamic loop rate in `spin()`.
- **Modify** `CMakeLists.txt` — swap the deleted controller for the new node in `catkin_install_python`.
- **Create** `test/test_dual_kinect_fusion_controller.py` — unit tests (local only).
- **Update docs** (local only): `docs/MODULE_C_*`, `docs/MODULE_A_*`, `PROJECT_STRUCTURE.md`, `docs/SYSTEM_FLOW_DIAGRAMS.md`, `docs/MODULES_INDEX.md`.
- **Proposal only** `docs/superpowers/plans/launch-proposal-dual-kinect.md` — the gated launch change.

---

### Task 0: Feature branch

- [ ] **Step 1: Branch off main**

```bash
cd /home/trungvdt/catkin_ws/src/ur3_hrc_planner
git checkout main
git checkout -b feat/dual-kinect-fusion-controller
```

Expected: `Switched to a new branch 'feat/dual-kinect-fusion-controller'`. Leave the existing
unstaged working-tree changes (`dual_kinect_system.launch`, deleted old controller, `system_back.launch`) untouched.

---

### Task 1: Tracker runtime rate control — clamp helper (pure, TDD first)

**Files:**
- Modify: `scripts/kinect_skeleton_tracker.py`
- Test: `test/test_tracker_rate_control.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_tracker_rate_control.py`:

```python
#!/usr/bin/env python3
import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from kinect_skeleton_tracker import clamp_rate


class ClampRateTest(unittest.TestCase):
    def test_within_range_unchanged(self):
        self.assertEqual(clamp_rate(7.0, 1.0, 15.0), 7.0)

    def test_below_min_clamped_up(self):
        self.assertEqual(clamp_rate(0.0, 1.0, 15.0), 1.0)

    def test_above_max_clamped_down(self):
        self.assertEqual(clamp_rate(99.0, 1.0, 15.0), 15.0)

    def test_string_coerced(self):
        self.assertEqual(clamp_rate("5", 1.0, 15.0), 5.0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_tracker_rate_control.py -v`
Expected: FAIL — `ImportError: cannot import name 'clamp_rate'`.

- [ ] **Step 3: Add the helper**

In `scripts/kinect_skeleton_tracker.py`, change the std_msgs import line:

```python
from std_msgs.msg import String
```

to:

```python
from std_msgs.msg import Float32, String
```

Then add this module-level function just after the imports block (after the `Point3D`/`ARM_BONES`
definitions near the top, before `def _int_list_param`):

```python
def clamp_rate(value, rate_min: float, rate_max: float) -> float:
    """Clamp a requested loop rate (Hz) into [rate_min, rate_max]."""
    return max(float(rate_min), min(float(rate_max), float(value)))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_tracker_rate_control.py -v`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit (code only — test is gitignored)**

```bash
git add scripts/kinect_skeleton_tracker.py
git commit -m "feat(tracker): add clamp_rate helper for runtime rate control

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Tracker runtime rate control — subscriber + dynamic loop

**Files:**
- Modify: `scripts/kinect_skeleton_tracker.py` (init `~line 384`, end of `__init__`, `spin()` `~line 1130`)

> No new unit test here — runtime behaviour needs a live ROS node. Manual validation in Task 8.
> The pure clamp logic is already covered by Task 1.

- [ ] **Step 1: Add rate-control state in `__init__`**

In `KinectSkeletonTracker.__init__`, immediately after this existing line:

```python
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
```

insert:

```python
        self.rate_cmd_topic = str(rospy.get_param("~rate_cmd_topic", "")).strip()
        self.rate_min = float(rospy.get_param("~rate_min", 1.0))
        self.rate_max = float(rospy.get_param("~rate_max", 15.0))
        self._rate_lock = threading.Lock()
        self._target_rate_hz = clamp_rate(self.rate_hz, self.rate_min, self.rate_max)
```

- [ ] **Step 2: Add the rate-command subscriber at the end of `__init__`**

At the very end of `__init__` (after the final `rospy.loginfo(...)` recovery log block, before the
method ends), add:

```python
        if self.rate_cmd_topic:
            rospy.Subscriber(self.rate_cmd_topic, Float32, self._on_rate_cmd, queue_size=1)
            rospy.loginfo(
                "[%s] runtime rate control on %s (clamp %.1f-%.1f Hz)",
                self.camera_name,
                self.rate_cmd_topic,
                self.rate_min,
                self.rate_max,
            )
```

- [ ] **Step 3: Add the callback method**

Add this method to `KinectSkeletonTracker` (e.g. just before `def spin`):

```python
    def _on_rate_cmd(self, msg) -> None:
        new_rate = clamp_rate(msg.data, self.rate_min, self.rate_max)
        with self._rate_lock:
            changed = abs(new_rate - self._target_rate_hz) > 1e-6
            self._target_rate_hz = new_rate
        if changed:
            rospy.loginfo("[%s] tracker rate command -> %.2f Hz", self.camera_name, new_rate)
```

- [ ] **Step 4: Make `spin()` apply the rate dynamically**

In `spin()`, replace this opening:

```python
    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
```

with:

```python
    def spin(self) -> None:
        applied_rate_hz = self._target_rate_hz
        rate = rospy.Rate(applied_rate_hz)
        while not rospy.is_shutdown():
            with self._rate_lock:
                target_rate_hz = self._target_rate_hz
            if abs(target_rate_hz - applied_rate_hz) > 1e-6:
                applied_rate_hz = target_rate_hz
                rate = rospy.Rate(applied_rate_hz)
                rospy.loginfo("[%s] tracker loop now %.2f Hz", self.camera_name, applied_rate_hz)
```

(The existing loop body — `needs_camera = ...` through `rate.sleep()` — stays unchanged below.)

- [ ] **Step 5: Byte-compile check**

Run: `python3 -m py_compile scripts/kinect_skeleton_tracker.py`
Expected: no output (success).

- [ ] **Step 6: Re-run the clamp test (regression)**

Run: `python3 -m pytest test/test_tracker_rate_control.py -v`
Expected: PASS (4 passed).

- [ ] **Step 7: Commit**

```bash
git add scripts/kinect_skeleton_tracker.py
git commit -m "feat(tracker): runtime loop-rate control via ~rate_cmd_topic

Additive Float32 subscriber, default off; single-camera launches unchanged.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: `SkeletonFuser` (pure, TDD)

**Files:**
- Create: `scripts/dual_kinect_fusion_controller.py`
- Test: `test/test_dual_kinect_fusion_controller.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_dual_kinect_fusion_controller.py`:

```python
#!/usr/bin/env python3
import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from dual_kinect_fusion_controller import SkeletonFuser

TRACKED = [0, 11, 12, 13, 14, 15, 16, 23, 24]


class SkeletonFuserTest(unittest.TestCase):
    def setUp(self):
        self.fuser = SkeletonFuser(TRACKED, max_merge_dist=0.20)

    def test_both_fresh_averaged(self):
        front = {11: (0.0, 0.0, 0.0)}
        back = {11: (0.10, 0.0, 0.0)}
        fused, mode = self.fuser.fuse(front, back)
        self.assertAlmostEqual(fused[11][0], 0.05)
        self.assertEqual(mode, SkeletonFuser.MODE_BOTH)

    def test_divergent_prefers_front(self):
        front = {11: (0.0, 0.0, 0.0)}
        back = {11: (0.50, 0.0, 0.0)}  # 0.5 m > max_merge_dist
        fused, _mode = self.fuser.fuse(front, back)
        self.assertEqual(fused[11], (0.0, 0.0, 0.0))

    def test_back_fills_occluded_joint(self):
        front = {11: (0.0, 0.0, 0.0)}  # joint 13 missing in front
        back = {11: (0.0, 0.0, 0.0), 13: (1.0, 1.0, 1.0)}
        fused, mode = self.fuser.fuse(front, back)
        self.assertEqual(fused[13], (1.0, 1.0, 1.0))
        self.assertEqual(mode, SkeletonFuser.MODE_FILL_FROM_BACK)

    def test_front_only(self):
        fused, mode = self.fuser.fuse({11: (0.0, 0.0, 0.0)}, {})
        self.assertIn(11, fused)
        self.assertEqual(mode, SkeletonFuser.MODE_FRONT_ONLY)

    def test_back_only(self):
        fused, mode = self.fuser.fuse({}, {11: (0.0, 0.0, 0.0)})
        self.assertIn(11, fused)
        self.assertEqual(mode, SkeletonFuser.MODE_BACK_ONLY)

    def test_no_input_empty(self):
        fused, mode = self.fuser.fuse({}, {})
        self.assertEqual(fused, {})
        self.assertEqual(mode, SkeletonFuser.MODE_NO_INPUT)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_dual_kinect_fusion_controller.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'dual_kinect_fusion_controller'`.

- [ ] **Step 3: Create the module with imports, helpers, and `SkeletonFuser`**

Create `scripts/dual_kinect_fusion_controller.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import sys
from typing import Dict, Iterable, List, Tuple

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, String

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from data_skeleton import pose_array_to_numeric_joint_dict, skeleton_dict_to_pose_array

Point3D = Tuple[float, float, float]
Skeleton = Dict[int, Point3D]

DEFAULT_TRACKED_JOINT_IDS = [0, 11, 12, 13, 14, 15, 16, 23, 24]


def _int_list_param(name: str, default: Iterable[int]) -> List[int]:
    value = rospy.get_param(name, list(default))
    if isinstance(value, str):
        return [int(item.strip()) for item in value.split(",") if item.strip()]
    return [int(item) for item in value]


def _point_distance(a: Point3D, b: Point3D) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


class SkeletonFuser:
    """Minimal per-joint occlusion-fill fusion of two base-frame skeletons."""

    MODE_BOTH = "BOTH"
    MODE_FILL_FROM_BACK = "FILL_FROM_BACK"
    MODE_FRONT_ONLY = "FRONT_ONLY"
    MODE_BACK_ONLY = "BACK_ONLY"
    MODE_NO_INPUT = "NO_INPUT"

    def __init__(self, tracked_joint_ids: Iterable[int], max_merge_dist: float) -> None:
        self.tracked_joint_ids = [int(j) for j in tracked_joint_ids]
        self.max_merge_dist = float(max_merge_dist)

    def fuse(self, front: Skeleton, back: Skeleton) -> Tuple[Skeleton, str]:
        fused: Skeleton = {}
        used_back_fill = False
        for jid in self.tracked_joint_ids:
            f = front.get(jid)
            b = back.get(jid)
            if f is not None and b is not None:
                if _point_distance(f, b) > self.max_merge_dist:
                    fused[jid] = f  # divergent -> trust primary (front)
                else:
                    fused[jid] = (
                        (f[0] + b[0]) * 0.5,
                        (f[1] + b[1]) * 0.5,
                        (f[2] + b[2]) * 0.5,
                    )
            elif f is not None:
                fused[jid] = f
            elif b is not None:
                fused[jid] = b
                used_back_fill = True
        return fused, self._mode(front, back, used_back_fill)

    def _mode(self, front: Skeleton, back: Skeleton, used_back_fill: bool) -> str:
        has_f, has_b = bool(front), bool(back)
        if has_f and has_b:
            return self.MODE_FILL_FROM_BACK if used_back_fill else self.MODE_BOTH
        if has_f:
            return self.MODE_FRONT_ONLY
        if has_b:
            return self.MODE_BACK_ONLY
        return self.MODE_NO_INPUT
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_dual_kinect_fusion_controller.py -v`
Expected: PASS (6 passed).

- [ ] **Step 5: Commit**

```bash
git add scripts/dual_kinect_fusion_controller.py
git commit -m "feat(fusion): add SkeletonFuser occlusion-fill core

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: `DutyScheduler` (pure, TDD)

**Files:**
- Modify: `scripts/dual_kinect_fusion_controller.py`
- Test: `test/test_dual_kinect_fusion_controller.py`

- [ ] **Step 1: Add failing tests**

Append to `test/test_dual_kinect_fusion_controller.py` (before the final `if __name__` block):

```python
from dual_kinect_fusion_controller import DutyScheduler


class DutySchedulerTest(unittest.TestCase):
    def _make(self):
        return DutyScheduler(
            total_tracked=9,
            miss_threshold=2,
            confirm_frames=3,
            boost_hold_sec=1.5,
            front_high_rate=10.0,
            back_high_rate=6.0,
            back_low_rate=2.0,
        )

    def test_startup_commands_front_high_back_low(self):
        sched = self._make()
        cmds, state = sched.update(front_valid_count=9, now_sec=0.0)
        self.assertEqual(cmds["front"], 10.0)
        self.assertEqual(cmds["back"], 2.0)
        self.assertEqual(state, DutyScheduler.STATE_LOW)

    def test_boost_after_sustained_miss(self):
        sched = self._make()
        sched.update(front_valid_count=9, now_sec=0.0)  # startup
        # front sees only 6 of 9 -> missing 3 (>= miss_threshold 2)
        c1, _ = sched.update(front_valid_count=6, now_sec=0.1)
        self.assertNotIn("back", c1)  # streak 1, not yet confirmed
        c2, _ = sched.update(front_valid_count=6, now_sec=0.2)
        self.assertNotIn("back", c2)  # streak 2
        c3, state = sched.update(front_valid_count=6, now_sec=0.3)
        self.assertEqual(c3["back"], 6.0)  # streak 3 == confirm_frames -> HIGH
        self.assertEqual(state, DutyScheduler.STATE_HIGH)

    def test_hysteresis_release_after_hold(self):
        sched = self._make()
        sched.update(9, 0.0)
        for t in (0.1, 0.2, 0.3):
            sched.update(6, t)  # boost to HIGH
        # front recovers; hold for boost_hold_sec before dropping
        c_recover, state = sched.update(9, 0.4)
        self.assertNotIn("back", c_recover)
        self.assertEqual(state, DutyScheduler.STATE_HIGH)
        c_hold, _ = sched.update(9, 1.0)  # still within hold (0.4 + 1.5 = 1.9)
        self.assertNotIn("back", c_hold)
        c_drop, state = sched.update(9, 2.0)  # past 1.9 -> drop to LOW
        self.assertEqual(c_drop["back"], 2.0)
        self.assertEqual(state, DutyScheduler.STATE_LOW)
```

- [ ] **Step 2: Run to verify it fails**

Run: `python3 -m pytest test/test_dual_kinect_fusion_controller.py::DutySchedulerTest -v`
Expected: FAIL — `ImportError: cannot import name 'DutyScheduler'`.

- [ ] **Step 3: Add `DutyScheduler` to the module**

Append to `scripts/dual_kinect_fusion_controller.py` (after `SkeletonFuser`):

```python
class DutyScheduler:
    """Occlusion-driven HIGH/LOW rate scheduler. Front stays HIGH; back boosts on front occlusion."""

    STATE_LOW = "BACK_LOW"
    STATE_HIGH = "BACK_HIGH"

    def __init__(
        self,
        total_tracked: int,
        miss_threshold: int,
        confirm_frames: int,
        boost_hold_sec: float,
        front_high_rate: float,
        back_high_rate: float,
        back_low_rate: float,
    ) -> None:
        self.total_tracked = int(total_tracked)
        self.miss_threshold = int(miss_threshold)
        self.confirm_frames = int(confirm_frames)
        self.boost_hold_sec = float(boost_hold_sec)
        self.front_high_rate = float(front_high_rate)
        self.back_high_rate = float(back_high_rate)
        self.back_low_rate = float(back_low_rate)
        self.back_state = self.STATE_LOW
        self._miss_streak = 0
        self._release_at = None
        self._front_started = False

    def update(self, front_valid_count: int, now_sec: float) -> Tuple[Dict[str, float], str]:
        cmds: Dict[str, float] = {}
        if not self._front_started:
            cmds["front"] = self.front_high_rate
            cmds["back"] = self.back_low_rate
            self._front_started = True

        missing = self.total_tracked - int(front_valid_count)
        occluded = missing >= self.miss_threshold
        self._miss_streak = self._miss_streak + 1 if occluded else 0

        if self.back_state == self.STATE_LOW:
            if self._miss_streak >= self.confirm_frames:
                self.back_state = self.STATE_HIGH
                self._release_at = None
                cmds["back"] = self.back_high_rate
        else:  # STATE_HIGH
            if occluded:
                self._release_at = None
            elif self._release_at is None:
                self._release_at = now_sec + self.boost_hold_sec
            elif now_sec >= self._release_at:
                self.back_state = self.STATE_LOW
                self._release_at = None
                cmds["back"] = self.back_low_rate

        return cmds, self.back_state
```

- [ ] **Step 4: Run to verify it passes**

Run: `python3 -m pytest test/test_dual_kinect_fusion_controller.py -v`
Expected: PASS (9 passed total).

- [ ] **Step 5: Commit**

```bash
git add scripts/dual_kinect_fusion_controller.py
git commit -m "feat(fusion): add occlusion-driven DutyScheduler

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: Output schema round-trip test (interface stability)

**Files:**
- Test: `test/test_dual_kinect_fusion_controller.py`

> Guards the spec's "output `/human_skeleton_base` byte-compatible with downstream decode" requirement.

- [ ] **Step 1: Add the failing/round-trip test**

Append to `test/test_dual_kinect_fusion_controller.py` (before the final `if __name__` block):

```python
from data_skeleton import pose_array_to_numeric_joint_dict, skeleton_dict_to_pose_array


class FusedOutputSchemaTest(unittest.TestCase):
    def test_fused_encode_decodes_to_same_joints(self):
        fused = {0: (0.1, 0.2, 0.3), 11: (1.1, 1.2, 1.3), 24: (2.4, 2.5, 2.6)}
        msg = skeleton_dict_to_pose_array(fused, "base_link", None, TRACKED)
        self.assertEqual(len(msg.poses), len(TRACKED))  # one slot per tracked joint
        decoded = pose_array_to_numeric_joint_dict(msg, TRACKED)
        self.assertEqual(set(decoded.keys()), set(fused.keys()))
        for jid, point in fused.items():
            self.assertAlmostEqual(decoded[jid][0], point[0])
            self.assertAlmostEqual(decoded[jid][2], point[2])
```

- [ ] **Step 2: Run to verify it passes**

Run: `python3 -m pytest test/test_dual_kinect_fusion_controller.py::FusedOutputSchemaTest -v`
Expected: PASS (1 passed). (No code change needed — this asserts the existing `data_skeleton`
contract the node will rely on. If it fails, stop: the encode/decode helpers diverge from the spec.)

- [ ] **Step 3: Commit (test is gitignored — nothing to commit)**

No `git add` (test/ is gitignored). Move on.

---

### Task 6: Wire the `DualKinectFusionController` node

**Files:**
- Modify: `scripts/dual_kinect_fusion_controller.py`

> The node body needs a live ROS master to unit-test meaningfully; logic is already covered by
> Tasks 3–5. Validate with `py_compile` here and live in Task 8.

- [ ] **Step 1: Append the node class and `main()`**

Append to `scripts/dual_kinect_fusion_controller.py`:

```python
class DualKinectFusionController:
    def __init__(self) -> None:
        rospy.init_node("dual_kinect_fusion_controller")

        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.tracked_joint_ids = _int_list_param("~tracked_joint_ids", DEFAULT_TRACKED_JOINT_IDS)
        self.front_topic = rospy.get_param("~front_skeleton_base_topic", "/kinect_front/human_skeleton_base")
        self.back_topic = rospy.get_param("~back_skeleton_base_topic", "/kinect_back/human_skeleton_base")
        self.output_topic = rospy.get_param("~output_skeleton_base_topic", "/human_skeleton_base")
        self.status_topic = rospy.get_param("~fusion_status_topic", "/human_skeleton_fusion_status")
        self.front_rate_cmd_topic = rospy.get_param("~front_rate_cmd_topic", "/kinect_front/tracker_rate_cmd")
        self.back_rate_cmd_topic = rospy.get_param("~back_rate_cmd_topic", "/kinect_back/tracker_rate_cmd")

        self.fusion_rate_hz = float(rospy.get_param("~fusion_rate_hz", 20.0))
        self.max_input_age_sec = float(rospy.get_param("~max_input_age_sec", 0.35))
        self.empty_publish_on_no_input = bool(rospy.get_param("~empty_publish_on_no_input", True))

        self.fuser = SkeletonFuser(
            self.tracked_joint_ids,
            float(rospy.get_param("~max_merge_dist", 0.20)),
        )
        self.scheduler = DutyScheduler(
            total_tracked=len(self.tracked_joint_ids),
            miss_threshold=int(rospy.get_param("~miss_threshold", 2)),
            confirm_frames=int(rospy.get_param("~confirm_frames", 3)),
            boost_hold_sec=float(rospy.get_param("~boost_hold_sec", 1.5)),
            front_high_rate=float(rospy.get_param("~front_high_rate", 10.0)),
            back_high_rate=float(rospy.get_param("~back_high_rate", 6.0)),
            back_low_rate=float(rospy.get_param("~back_low_rate", 2.0)),
        )

        self._front_msg = None
        self._front_time = None
        self._back_msg = None
        self._back_time = None

        self.output_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.front_rate_pub = rospy.Publisher(self.front_rate_cmd_topic, Float32, queue_size=1, latch=True)
        self.back_rate_pub = rospy.Publisher(self.back_rate_cmd_topic, Float32, queue_size=1, latch=True)

        rospy.Subscriber(self.front_topic, PoseArray, self._on_front, queue_size=1)
        rospy.Subscriber(self.back_topic, PoseArray, self._on_back, queue_size=1)

        rospy.loginfo(
            "dual_kinect_fusion_controller up: out=%s fuse_hz=%.1f max_age=%.2fs tracked=%d",
            self.output_topic,
            self.fusion_rate_hz,
            self.max_input_age_sec,
            len(self.tracked_joint_ids),
        )

    def _on_front(self, msg) -> None:
        self._front_msg = msg
        self._front_time = rospy.Time.now()

    def _on_back(self, msg) -> None:
        self._back_msg = msg
        self._back_time = rospy.Time.now()

    def _fresh_skeleton(self, msg, stamp, now) -> Skeleton:
        if msg is None or stamp is None:
            return {}
        if (now - stamp).to_sec() > self.max_input_age_sec:
            return {}
        return pose_array_to_numeric_joint_dict(msg, self.tracked_joint_ids)

    def _publish_rate_cmds(self, cmds) -> None:
        if "front" in cmds:
            self.front_rate_pub.publish(Float32(data=cmds["front"]))
        if "back" in cmds:
            self.back_rate_pub.publish(Float32(data=cmds["back"]))

    def on_timer(self, _event) -> None:
        now = rospy.Time.now()
        front = self._fresh_skeleton(self._front_msg, self._front_time, now)
        back = self._fresh_skeleton(self._back_msg, self._back_time, now)

        fused, mode = self.fuser.fuse(front, back)
        if fused or self.empty_publish_on_no_input:
            self.output_pub.publish(
                skeleton_dict_to_pose_array(fused, self.target_frame, now, self.tracked_joint_ids)
            )

        cmds, back_state = self.scheduler.update(len(front), now.to_sec())
        self._publish_rate_cmds(cmds)

        self.status_pub.publish(
            String(
                data="mode=%s duty=%s front_joints=%d back_joints=%d fused=%d"
                % (mode, back_state, len(front), len(back), len(fused))
            )
        )

    def spin(self) -> None:
        rospy.Timer(rospy.Duration(1.0 / self.fusion_rate_hz), self.on_timer)
        rospy.spin()


def main() -> None:
    DualKinectFusionController().spin()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Make executable + byte-compile**

```bash
chmod +x scripts/dual_kinect_fusion_controller.py
python3 -m py_compile scripts/dual_kinect_fusion_controller.py
```
Expected: no output.

- [ ] **Step 3: Re-run the unit suite (regression)**

Run: `python3 -m pytest test/test_dual_kinect_fusion_controller.py -v`
Expected: PASS (all green).

- [ ] **Step 4: Commit**

```bash
git add scripts/dual_kinect_fusion_controller.py
git commit -m "feat(fusion): dual_kinect_fusion_controller node (timer fuse + duty cmds)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 7: CMakeLists install entry

**Files:**
- Modify: `CMakeLists.txt:215-220`

- [ ] **Step 1: Swap the deleted controller for the new node**

In `CMakeLists.txt`, in the `catkin_install_python(PROGRAMS ...)` block, replace:

```cmake
  scripts/multi_kinect_skeleton_controller.py
```

with:

```cmake
  scripts/dual_kinect_fusion_controller.py
```

(Leave the other four script entries untouched.)

- [ ] **Step 2: Build check**

```bash
cd /home/trungvdt/catkin_ws
catkin_make --pkg ur3_hrc_planner 2>&1 | tail -20
```
Expected: build completes without "file not found" for the install list. (If `catkin_make` is not
the project's build tool, use `catkin build ur3_hrc_planner` instead.)

- [ ] **Step 3: Commit**

```bash
cd /home/trungvdt/catkin_ws/src/ur3_hrc_planner
git add CMakeLists.txt
git commit -m "build: install dual_kinect_fusion_controller, drop removed controller

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 8: Launch wiring — PROPOSAL ONLY (🔒 gated)

**Files:**
- Create: `docs/superpowers/plans/launch-proposal-dual-kinect.md` (proposal doc, not the launch file)
- **Do NOT edit** `launch/dual_kinect_system.launch` without explicit user confirmation.

- [ ] **Step 1: Write the proposal doc**

Create `docs/superpowers/plans/launch-proposal-dual-kinect.md` containing the exact intended change:

  - **Add** to each tracker group a rate-command param so the scheduler can drive it:
    - `kinect_front` node: `<param name="rate_cmd_topic" value="/kinect_front/tracker_rate_cmd" />`,
      `<param name="rate_min" value="1.0" />`, `<param name="rate_max" value="12.0" />`.
    - `kinect_back` node: `<param name="rate_cmd_topic" value="/kinect_back/tracker_rate_cmd" />`,
      `<param name="rate_min" value="1.0" />`, `<param name="rate_max" value="8.0" />`.
  - **Re-add** the controller node (replacing the `[REMOVED]` comment block at lines ~291-294):

    ```xml
    <node pkg="ur3_hrc_planner"
          type="dual_kinect_fusion_controller.py"
          name="dual_kinect_fusion_controller"
          required="false"
          output="screen">
      <param name="target_frame" value="$(arg target_frame)" />
      <param name="tracked_joint_ids" value="$(arg tracked_joint_ids)" />
      <param name="front_skeleton_base_topic" value="$(arg front_skeleton_base_topic)" />
      <param name="back_skeleton_base_topic" value="$(arg back_skeleton_base_topic)" />
      <param name="output_skeleton_base_topic" value="$(arg human_skeleton_base_topic)" />
      <param name="front_rate_cmd_topic" value="/kinect_front/tracker_rate_cmd" />
      <param name="back_rate_cmd_topic" value="/kinect_back/tracker_rate_cmd" />
      <param name="fusion_rate_hz" value="20.0" />
      <param name="max_input_age_sec" value="0.35" />
      <param name="max_merge_dist" value="0.20" />
      <param name="miss_threshold" value="2" />
      <param name="confirm_frames" value="3" />
      <param name="boost_hold_sec" value="1.5" />
      <param name="front_high_rate" value="10.0" />
      <param name="back_high_rate" value="6.0" />
      <param name="back_low_rate" value="2.0" />
    </node>
    ```

  - State affected nodes/topics/params using the AGENTS.md "Launch File Confirmation Template".

- [ ] **Step 2: Present the proposal to the user and STOP**

Use the AGENTS.md confirmation template (file, reason, planned change, affected
nodes/topics/params/TF/serials). **Wait for explicit user confirmation.** Only after a clear "yes"
edit `launch/dual_kinect_system.launch` and commit it separately:

```bash
git add launch/dual_kinect_system.launch
git commit -m "feat(launch): wire dual_kinect_fusion_controller + tracker rate cmds

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 9: Documentation sync (local only — gitignored, no commit)

**Files (all gitignored — edit, do not `git add`):**
- `docs/MODULE_C_multi_kinect_skeleton_controller.md`
- `docs/MODULE_A_kinect_skeleton_tracker.md`
- `PROJECT_STRUCTURE.md`
- `docs/SYSTEM_FLOW_DIAGRAMS.md`
- `docs/MODULES_INDEX.md`

- [ ] **Step 1: Rewrite MODULE_C for the new controller**

Replace the old fusion description with: node name `dual_kinect_fusion_controller`, file
`scripts/dual_kinect_fusion_controller.py`, components `SkeletonFuser` (occlusion-fill table from the
spec) + `DutyScheduler` (occlusion-driven HIGH/LOW). Document I/O: subscribes
`/kinect_front/human_skeleton_base`, `/kinect_back/human_skeleton_base`; publishes
`/human_skeleton_base`, `/human_skeleton_fusion_status`, `/kinect_{front,back}/tracker_rate_cmd`.
List all params from Task 6. Note what was dropped vs the old node (Kalman, conflict map, side-swap,
quality score).

- [ ] **Step 2: Update MODULE_A**

Add a "Runtime rate control" section: new params `~rate_cmd_topic`, `~rate_min`, `~rate_max`; the
`Float32` subscriber; default-off behaviour preserving single-camera launches; the
`[rate_min, rate_max]` clamp as a safety bound.

- [ ] **Step 3: Update PROJECT_STRUCTURE.md**

In the file tree and the dual-Kinect block diagram, replace `multi_kinect_skeleton_controller.py`
with `dual_kinect_fusion_controller.py`; update the Topics/TF and Launch sections with the new topics
and the rate-cmd wiring.

- [ ] **Step 4: Update SYSTEM_FLOW_DIAGRAMS.md and MODULES_INDEX.md**

Reflect the new node name, the two internal components, and the new topics in the dual-Kinect flow.

- [ ] **Step 5: No commit**

These paths are gitignored. Confirm with `git status --short` that no doc files appear staged.

---

## Self-Review

**1. Spec coverage**

- §2 minimal occlusion-fill → Task 3 (`SkeletonFuser` + table). ✓
- §2 adaptive duty-cycle → Tasks 2, 4 (runtime rate + `DutyScheduler`). ✓
- §3 occlusion-driven trigger → Task 4 boost/hysteresis. ✓
- §4 one node, two components → Tasks 3,4,6. ✓
- §5 Fuser table incl. `max_merge_dist` guard + both-stale empty → Task 3 tests + Task 6 `empty_publish_on_no_input`. ✓
- §6 duty defaults / hysteresis → Task 4 + Task 6 params. ✓
- §7 tracker additive rate control, default off → Tasks 1,2. ✓
- §8 interfaces/params → Task 6 (node) + Task 8 (launch wiring). ✓
- §8 output schema unchanged → Task 5 round-trip. ✓
- §9 safety (never invent joints, both-stale removal, clamp) → Tasks 1,3,6. ✓
- §11 launch gated → Task 8 proposal-only. ✓
- §12 file/doc impact → Tasks 5,7,9. ✓

**2. Placeholder scan:** No TBD/TODO; every code step shows full code; doc steps name concrete sections to edit. ✓

**3. Type consistency:** `SkeletonFuser.fuse(front, back) -> (fused, mode)`, `DutyScheduler.update(front_valid_count, now_sec) -> (cmds, state)`, `clamp_rate(value, rate_min, rate_max)` used identically across node and tests. Mode/state constants (`MODE_*`, `STATE_*`) referenced by the same names. `tracked_joint_ids` order drives both fuse and encode. ✓

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-15-dual-kinect-fusion-controller.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — fresh subagent per task, review between tasks, fast iteration.

**2. Inline Execution** — execute tasks in this session via executing-plans, batch with checkpoints.

**Note:** Task 8 (launch) hard-stops for your confirmation regardless of execution mode.

**Which approach?**
