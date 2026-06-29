#!/usr/bin/env python3
from __future__ import annotations

import copy
import csv
import sys
import math
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rospy
import moveit_commander
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import (
    GetPositionFK,
    GetPositionFKRequest,
    GetStateValidity,
    GetStateValidityRequest,
)
from moveit_commander import MoveGroupCommander, RobotCommander
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from astar_improved_3d import AStarImproved3D, PlanResult
from astar_lpa_3d import LPAStar3D
from data_skeleton import pose_array_to_numeric_joint_dict


JointMap = Dict[str, float]
Voxel = Tuple[int, int, int]

BODY_JOINT_IDS = [0, 11, 12, 13, 14, 15, 16, 23, 24]
LEFT_HAND_JOINT_IDS = [100 + index for index in range(21)]
RIGHT_HAND_JOINT_IDS = [200 + index for index in range(21)]
DEFAULT_TRACKED_JOINT_IDS = BODY_JOINT_IDS + LEFT_HAND_JOINT_IDS + RIGHT_HAND_JOINT_IDS
DEFAULT_SELECTED_BODY_JOINT_IDS = [11, 12, 13, 14, 15, 16, 23, 24]
DEFAULT_SELECTED_LEFT_HAND_JOINT_IDS = [100, 104, 105, 108, 109, 112, 113, 116, 117, 120]
DEFAULT_SELECTED_RIGHT_HAND_JOINT_IDS = [200, 204, 205, 208, 209, 212, 213, 216, 217, 220]

# Bone connections (joint_id_a, joint_id_b) for inflating obstacles along the
# limb/torso segments between joints, not only around the joints themselves.
# Matches the cylinder pairs used by skeleton_obstacle_builder (arms + torso).
DEFAULT_BONE_PAIRS: List[Tuple[int, int]] = [
    (11, 13),  # left shoulder -> left elbow
    (13, 15),  # left elbow -> left wrist
    (12, 14),  # right shoulder -> right elbow
    (14, 16),  # right elbow -> right wrist
    (11, 12),  # shoulders
    (11, 23),  # left shoulder -> left hip
    (12, 24),  # right shoulder -> right hip
    (23, 24),  # hips
]


def _parse_pair_list_param(
    name: str,
    default: List[Tuple[int, int]],
) -> List[Tuple[int, int]]:
    """Parse bone pairs from "a-b,c-d" string or list of [a,b]."""
    raw_value = rospy.get_param(name, ",".join("%d-%d" % p for p in default))
    pairs: List[Tuple[int, int]] = []
    if isinstance(raw_value, str):
        items = [item.strip() for item in raw_value.split(",") if item.strip()]
        for item in items:
            parts = [p.strip() for p in item.replace(",", "-").split("-") if p.strip()]
            if len(parts) != 2:
                rospy.logwarn("Ignoring invalid bone pair '%s' in param %s", item, name)
                continue
            try:
                pairs.append((int(parts[0]), int(parts[1])))
            except (TypeError, ValueError):
                rospy.logwarn("Ignoring invalid bone pair '%s' in param %s", item, name)
    elif isinstance(raw_value, (list, tuple)):
        for item in raw_value:
            try:
                pairs.append((int(item[0]), int(item[1])))
            except (TypeError, ValueError, IndexError):
                rospy.logwarn("Ignoring invalid bone pair '%s' in param %s", item, name)
    return pairs if pairs else list(default)


def _parse_int_list_param(name: str, default: List[int]) -> List[int]:
    raw_value = rospy.get_param(name, ",".join(str(item) for item in default))
    if isinstance(raw_value, str):
        items = [item.strip() for item in raw_value.split(",") if item.strip()]
    elif isinstance(raw_value, (list, tuple)):
        items = list(raw_value)
    else:
        return list(default)

    result: List[int] = []
    for item in items:
        if isinstance(item, bool):
            continue
        try:
            result.append(int(item))
        except (TypeError, ValueError):
            rospy.logwarn("Ignoring invalid int value '%s' in param %s", item, name)

    return result if result else list(default)


def _parse_string_list_param(name: str, default: List[str]) -> List[str]:
    raw_value = rospy.get_param(name, ",".join(default))
    if isinstance(raw_value, str):
        items = [item.strip() for item in raw_value.split(",") if item.strip()]
    elif isinstance(raw_value, (list, tuple)):
        items = [str(item).strip() for item in raw_value if str(item).strip()]
    else:
        return list(default)
    return items if items else list(default)


def _ros_now() -> rospy.Time:
    try:
        return rospy.Time.now()
    except rospy.ROSInitException:
        return rospy.Time(0)


def speed_padding_meters(
    speed_mps: float,
    deadband_mps: float,
    gain: float,
    max_padding_m: float,
) -> float:
    """Extra obstacle clearance (metres) from robot speed.

    Zero below the deadband, linear in (speed - deadband) by ``gain``, capped at
    ``max_padding_m``. ``gain<=0`` or ``max_padding_m<=0`` disables it. Pure
    function so it can be unit-checked without ROS (see ``--selftest``)."""
    if gain <= 0.0 or max_padding_m <= 0.0:
        return 0.0
    over = speed_mps - deadband_mps
    if over <= 0.0:
        return 0.0
    return min(max_padding_m, gain * over)


def should_escalate_to_detour(
    hand_blocks: bool,
    goal_in_collision: Optional[bool],
) -> bool:
    """Decide whether a blocked waypoint should escalate to detour/hold.

    Escalate when the human hand blocks the waypoint OR the goal joint state is
    known to be in collision (OMPL would otherwise reject it in ~tens of ms and
    the node would busy-retry forever). Unknown validity (``None``, e.g. the
    ``check_state_validity`` service is unavailable) never escalates on its own,
    so behaviour is unchanged when the service is missing. Pure function so it
    can be unit-checked without ROS (see ``--selftest``)."""
    if hand_blocks:
        return True
    return goal_in_collision is True


class PlanLogger:
    """Append plan events (success / blocked / unsafe / not-executed) to a log
    folder, with the reason and world coordinates. Writes both a CSV (machine
    readable) and a .log (human readable) per run. Flushes every event so logs
    survive a crash."""

    CSV_FIELDS = [
        "time_iso", "t_rel_s", "event", "status", "reason",
        "start_x", "start_y", "start_z",
        "goal_x", "goal_y", "goal_z",
        "n_obstacles", "n_human_pts",
        "human_x", "human_y", "human_z",
        "min_dist_m", "astar_path_len", "planning_ms", "detail",
    ]

    def __init__(self, log_dir: str, enabled: bool = True) -> None:
        self.enabled = enabled
        self.csv_writer = None
        self._file = None
        self._txt = None
        self._lock = threading.Lock()
        self._t0 = time.time()
        self.csv_path = ""
        self.txt_path = ""
        if not enabled:
            return
        try:
            os.makedirs(log_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_path = os.path.join(log_dir, "plan_log_%s.csv" % stamp)
            self.txt_path = os.path.join(log_dir, "plan_log_%s.log" % stamp)
            self._file = open(self.csv_path, "a", newline="")
            self.csv_writer = csv.writer(self._file)
            self.csv_writer.writerow(self.CSV_FIELDS)
            self._file.flush()
            self._txt = open(self.txt_path, "a")
        except OSError as exc:
            rospy.logwarn("PlanLogger disabled, cannot open log dir %s: %s", log_dir, exc)
            self.enabled = False

    @staticmethod
    def _fmt(value) -> str:
        if value is None:
            return ""
        if isinstance(value, float):
            return "%.4f" % value
        return str(value)

    def _xyz(self, point) -> Tuple[str, str, str]:
        if point is None:
            return "", "", ""
        return self._fmt(point[0]), self._fmt(point[1]), self._fmt(point[2])

    def log(
        self,
        event: str,
        status: str,
        reason: str = "",
        start=None,
        goal=None,
        n_obstacles=None,
        n_human=None,
        human=None,
        min_dist=None,
        path_len=None,
        planning_ms=None,
        detail: str = "",
    ) -> None:
        if not self.enabled or self.csv_writer is None:
            return
        sx, sy, sz = self._xyz(start)
        gx, gy, gz = self._xyz(goal)
        hx, hy, hz = self._xyz(human)
        now_iso = datetime.now().isoformat(timespec="milliseconds")
        t_rel = time.time() - self._t0
        row = [
            now_iso, "%.3f" % t_rel, event, status, reason,
            sx, sy, sz, gx, gy, gz,
            self._fmt(n_obstacles), self._fmt(n_human),
            hx, hy, hz,
            self._fmt(min_dist), self._fmt(path_len), self._fmt(planning_ms),
            detail,
        ]
        with self._lock:
            try:
                self.csv_writer.writerow(row)
                self._file.flush()
                if self._txt is not None:
                    self._txt.write(
                        "[%s t=%.2fs] %s/%s reason=%s start=(%s,%s,%s) "
                        "goal=(%s,%s,%s) obs=%s human=%s nearest=(%s,%s,%s) "
                        "min_dist=%s path=%s plan_ms=%s %s\n"
                        % (
                            now_iso, t_rel, event, status, reason,
                            sx, sy, sz, gx, gy, gz,
                            self._fmt(n_obstacles), self._fmt(n_human),
                            hx, hy, hz,
                            self._fmt(min_dist), self._fmt(path_len),
                            self._fmt(planning_ms), detail,
                        )
                    )
                    self._txt.flush()
            except (OSError, ValueError) as exc:
                rospy.logwarn_throttle(5.0, "PlanLogger write failed: %s", exc)

    def close(self) -> None:
        with self._lock:
            for handle in (self._file, self._txt):
                try:
                    if handle is not None:
                        handle.close()
                except OSError:
                    pass


class BreadcrumbCache:
    """Memory of recently-traversed robot poses (joint config + TCP).

    Records a node while the robot moves (spacing >= revisit_threshold), trims
    the prefix on loop-closure (returning near an earlier node drops everything
    before it: A B C D E then back to B -> B C D E B), and caps the size.

    Used as a warm-start: when a waypoint is blocked, the planner can hop to the
    nearest cached pose that is STILL collision-free now (re-validated), so the
    full ARA*/MoveIt/trajectory safety gates still apply to the actual motion.
    """

    def __init__(self, revisit_threshold: float, max_nodes: int, enabled: bool = True) -> None:
        self.enabled = enabled
        self.revisit_threshold = max(1e-3, float(revisit_threshold))
        self.max_nodes = max(2, int(max_nodes))
        self._nodes: List[Dict[str, object]] = []
        self._lock = threading.Lock()

    @staticmethod
    def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def record(self, joints: JointMap, tcp: Tuple[float, float, float]) -> None:
        if not self.enabled:
            return
        with self._lock:
            if self._nodes:
                last_tcp = self._nodes[-1]["tcp"]  # type: ignore[index]
                if self._dist(tcp, last_tcp) < self.revisit_threshold:  # type: ignore[arg-type]
                    return  # not moved enough -> avoid duplicate while ~stationary
                # Loop-closure: returning near an earlier node drops the prefix.
                for i in range(len(self._nodes) - 1):
                    if self._dist(tcp, self._nodes[i]["tcp"]) <= self.revisit_threshold:  # type: ignore[arg-type]
                        del self._nodes[:i]
                        break
            self._nodes.append({"joints": dict(joints), "tcp": tuple(tcp), "stamp": time.time()})
            if len(self._nodes) > self.max_nodes:
                del self._nodes[: len(self._nodes) - self.max_nodes]

    def nearest_valid(self, goal_xyz, is_valid) -> Optional[JointMap]:
        """Joint config of the cached node nearest to goal whose TCP is still
        valid now (is_valid(tcp) -> bool). None if cache empty / all invalid."""
        with self._lock:
            nodes = list(self._nodes)
        best = None
        best_dist = float("inf")
        for node in nodes:
            tcp = node["tcp"]
            if not is_valid(tcp):
                continue
            dist = self._dist(tcp, goal_xyz)  # type: ignore[arg-type]
            if dist < best_dist:
                best_dist = dist
                best = node
        if best is None:
            return None
        return dict(best["joints"])  # type: ignore[arg-type]

    def size(self) -> int:
        with self._lock:
            return len(self._nodes)


class UR3FixedJointABNode:
    def __init__(self) -> None:
        rospy.init_node("ur3_fixed_joint_ab_node")

        self.group_name = rospy.get_param("~group_name", "manipulator")
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.auto_execute = rospy.get_param("~auto_execute", False)

        self.velocity_scale = rospy.get_param("~velocity_scale", 0.02)
        self.acceleration_scale = rospy.get_param("~acceleration_scale", 0.02)
        self.planning_time = rospy.get_param("~planning_time", 10.0)
        self.planning_attempts = rospy.get_param("~planning_attempts", 10)
        self.move_group_wait_timeout = rospy.get_param("~move_group_wait_timeout", 60.0)
        self.move_group_retry_period = rospy.get_param("~move_group_retry_period", 2.0)
        self.fk_wait_timeout = rospy.get_param("~fk_wait_timeout", 60.0)

        self.wait_after_waypoint = rospy.get_param("~wait_after_waypoint", 0.0)
        self.cycles = rospy.get_param("~cycles", 1)
        self.loop = rospy.get_param("~loop", False)

        self.enable_astar_guard = rospy.get_param("~enable_astar_guard", True)
        self.human_topic = rospy.get_param("~human_topic", "/human_hand_robot")
        self.human_timeout_sec = rospy.get_param("~human_timeout_sec", 1.0)
        self.use_skeleton_obstacles = rospy.get_param("~use_skeleton_obstacles", False)
        self.human_skeleton_topic = rospy.get_param("~human_skeleton_topic", "/human_skeleton_base")
        self.human_skeleton_timeout_sec = rospy.get_param("~human_skeleton_timeout_sec", 0.5)
        self.enable_moveit_scene_sync = rospy.get_param("~enable_moveit_scene_sync", True)
        self.moveit_scene_status_topic = rospy.get_param("~moveit_scene_status_topic", "/moveit_scene_status")
        self.require_moveit_scene_ready = rospy.get_param("~require_moveit_scene_ready", False)
        self.scene_status_timeout_sec = float(rospy.get_param("~scene_status_timeout_sec", 30.0))
        self.scene_ready_keywords = _parse_string_list_param(
            "~scene_ready_keywords",
            ["SCENE_OBJECT_APPLIED", "SCENE_READY", "SCENE_OBJECT_CONFIRMED"],
        )
        self.scene_removed_keywords = _parse_string_list_param(
            "~scene_removed_keywords",
            ["SCENE_OBJECT_REMOVED", "SCENE_OBJECT_CONFIRMED_REMOVED", "SCENE_EMPTY"],
        )
        self.use_legacy_human_point = rospy.get_param("~use_legacy_human_point", False)
        self.human_point_selection_mode = rospy.get_param(
            "~human_point_selection_mode",
            "collision_reduced",
        )
        self.max_human_points_for_astar = max(
            0,
            int(rospy.get_param("~max_human_points_for_astar", 20)),
        )
        self.prefer_hand_points = rospy.get_param("~prefer_hand_points", True)
        self.tracked_joint_ids = _parse_int_list_param(
            "~tracked_joint_ids",
            DEFAULT_TRACKED_JOINT_IDS,
        )
        self.body_joint_ids = list(BODY_JOINT_IDS)
        self.selected_body_joint_ids = _parse_int_list_param(
            "~selected_body_joint_ids",
            DEFAULT_SELECTED_BODY_JOINT_IDS,
        )
        self.selected_left_hand_joint_ids = _parse_int_list_param(
            "~selected_left_hand_joint_ids",
            DEFAULT_SELECTED_LEFT_HAND_JOINT_IDS,
        )
        self.selected_right_hand_joint_ids = _parse_int_list_param(
            "~selected_right_hand_joint_ids",
            DEFAULT_SELECTED_RIGHT_HAND_JOINT_IDS,
        )
        self.publish_split_planning_time = rospy.get_param("~publish_split_planning_time", True)
        self.replan_retry_period = rospy.get_param("~replan_retry_period", 0.5)
        self.max_replan_attempts = rospy.get_param("~max_replan_attempts", 0)
        self.min_cartesian_fraction = rospy.get_param("~min_cartesian_fraction", 0.90)

        self.voxel_size = rospy.get_param("~voxel_size", 0.10)
        self.map_x_min = rospy.get_param("~map_x_min", -0.8)
        self.map_x_max = rospy.get_param("~map_x_max", 0.8)
        self.map_y_min = rospy.get_param("~map_y_min", -1.0)
        self.map_y_max = rospy.get_param("~map_y_max", 1.0)
        self.map_z_min = rospy.get_param("~map_z_min", 0.0)
        self.map_z_max = rospy.get_param("~map_z_max", 1.6)
        self.human_inflate_radius = rospy.get_param("~human_inflate_radius", 2)
        # Obstacle stability cache: keep A* human obstacles steady against
        # skeleton jitter so they do not drop in/out across voxel boundaries.
        self.enable_obstacle_cache = rospy.get_param("~enable_obstacle_cache", True)
        # If a fresh human point stays within this distance of a cached anchor,
        # keep the cached position (same voxel) instead of moving the obstacle.
        self.obstacle_stability_threshold = float(
            rospy.get_param("~obstacle_stability_threshold", 0.04)
        )
        # Grace window: hold a cached anchor this long after it is no longer
        # seen, to bridge transient skeleton dropouts. After this, it drops.
        self.obstacle_cache_hold_sec = float(
            rospy.get_param("~obstacle_cache_hold_sec", 0.5)
        )
        # Bone obstacles: inflate voxels along the segments connecting joints, so
        # A* sees the limbs/torso, not only the joint spheres. Thin tube by
        # default (radius 1) to fill inter-joint gaps without exploding the voxel
        # count (keeps ARA* fast). Raise bone_inflate_radius for more clearance.
        # Sphere (vs solid cube) inflation: drops corner voxels beyond clearance
        # so obstacle voxel count drops ~3-4x. Clearance ball preserved.
        self.obstacle_inflate_sphere = rospy.get_param("~obstacle_inflate_sphere", True)
        self.enable_bone_obstacles = rospy.get_param("~enable_bone_obstacles", True)
        self.bone_inflate_radius = max(
            0, int(rospy.get_param("~bone_inflate_radius", 1))
        )
        self.bone_connection_pairs = _parse_pair_list_param(
            "~bone_connection_pairs",
            DEFAULT_BONE_PAIRS,
        )
        # Continuous obstacle build/publish: rebuild + publish A* human voxels on
        # a timer so obstacles persist (do not drop) between plan cycles and are
        # always built whenever a valid voxel is seen, regardless of robot
        # distance or planning state. <=0 disables (build only during planning).
        self.obstacle_publish_rate = float(
            rospy.get_param("~obstacle_publish_rate", 10.0)
        )
        # How long the last plan's path/start/goal markers stay shown after a
        # plan; obstacles are always drawn fresh each tick.
        self.obstacle_path_viz_hold = float(
            rospy.get_param("~obstacle_path_viz_hold", 1.0)
        )
        # Plan event logging: record success / obstacle-blocked / unsafe /
        # not-executed events with reason and world coordinates to a log folder.
        self.enable_plan_log = rospy.get_param("~enable_plan_log", True)
        self.plan_log_dir = rospy.get_param(
            "~plan_log_dir",
            os.path.join(os.path.dirname(SCRIPT_DIR), "logs"),
        )
        # Breadcrumb cache: remember traversed poses to warm-start when blocked.
        self.enable_breadcrumb = rospy.get_param("~enable_breadcrumb", True)
        self.breadcrumb_record_period = max(
            0.05, float(rospy.get_param("~breadcrumb_record_period", 0.5))
        )
        self.breadcrumb_revisit_threshold = float(
            rospy.get_param("~breadcrumb_revisit_threshold", 0.05)
        )
        self.breadcrumb_max_nodes = max(
            2, int(rospy.get_param("~breadcrumb_max_nodes", 200))
        )
        # Hop to a still-valid cached pose when a waypoint plan is blocked.
        self.enable_breadcrumb_fallback = rospy.get_param(
            "~enable_breadcrumb_fallback", True
        )
        # Region preference: A* prefers voxels inside the workspace box swept by
        # the A->B waypoints (cables limit the robot to that shoulder_pan zone).
        # Soft penalty (not a hard wall) so human-avoidance can still leave it.
        self.enable_region_preference = rospy.get_param("~enable_region_preference", True)
        self.region_penalty_weight = float(rospy.get_param("~region_penalty_weight", 4.0))
        self.region_margin = float(rospy.get_param("~region_margin", 0.10))
        # HARD limit: keep shoulder_pan within the A->B sweep range on EVERY
        # MoveIt plan (path constraint), so the executed trajectory never leaves
        # the shoulder_pan zone and cannot snag the pneumatic cable.
        self.enable_pan_limit = rospy.get_param("~enable_pan_limit", True)
        self.pan_limit_margin = float(rospy.get_param("~pan_limit_margin", 0.10))
        # Optional explicit override (rad); None -> auto from waypoints.
        self.shoulder_pan_min = rospy.get_param("~shoulder_pan_min", None)
        self.shoulder_pan_max = rospy.get_param("~shoulder_pan_max", None)
        self.human_safety_distance = rospy.get_param("~human_safety_distance", 0.25)
        self.robot_body_radius = rospy.get_param("~robot_body_radius", 0.12)
        self.human_body_radius = rospy.get_param("~human_body_radius", 0.25)
        self.speed_safety_gain = rospy.get_param("~speed_safety_gain", 0.35)
        self.max_speed_safety_margin = rospy.get_param("~max_speed_safety_margin", 0.30)
        self.execution_speed_safety_margin = rospy.get_param("~execution_speed_safety_margin", 0.10)
        self.trajectory_sample_step = max(1, int(rospy.get_param("~trajectory_sample_step", 3)))
        self.execution_monitor_rate = rospy.get_param("~execution_monitor_rate", 20.0)
        self.resampled_waypoint_count = max(2, int(rospy.get_param("~resampled_waypoint_count", 7)))
        self.hand_waypoint_collision_radius = rospy.get_param("~hand_waypoint_collision_radius", 0.15)
        self.detour_z_offset = rospy.get_param("~detour_z_offset", 0.07)
        self.max_detour_attempts = max(1, int(rospy.get_param("~max_detour_attempts", 5)))
        self.detour_retry_period = max(0.0, float(rospy.get_param("~detour_retry_period", 0.5)))
        self.detour_planning_time = max(0.01, float(rospy.get_param("~detour_planning_time", 0.75)))
        self.detour_hold_poll_period = max(
            0.01,
            float(rospy.get_param("~detour_hold_poll_period", 0.5)),
        )
        self.check_pause_sec = rospy.get_param("~check_pause_sec", 0.2)
        self.max_astar_marker_voxels = max(0, int(rospy.get_param("~max_astar_marker_voxels", 500)))
        self.ara_epsilon_start = float(rospy.get_param("~ara_epsilon_start", 3.0))
        self.ara_epsilon_final = float(rospy.get_param("~ara_epsilon_final", 1.0))
        self.ara_epsilon_decay = float(rospy.get_param("~ara_epsilon_decay", 0.5))
        self.ara_max_time_ms = float(rospy.get_param("~ara_max_time_ms", 50.0))
        self.ara_max_steps = max(1, int(rospy.get_param("~ara_max_steps", 50000)))

        # TCP voxel guard planner selection. "ara_star" (default) = AStarImproved3D,
        # unchanged. "lpa_star" = LPAStar3D incremental repair (drop-in API). LPA*
        # shares the ARA* time/step budget (ara_max_time_ms / ara_max_steps).
        self.guard_planner_type = str(rospy.get_param("~guard_planner_type", "ara_star"))
        self.lpa_epsilon = float(rospy.get_param("~lpa_epsilon", 1.0))
        self.lpa_start_reuse_radius_voxels = int(
            rospy.get_param("~lpa_start_reuse_radius_voxels", 1)
        )
        self.lpa_max_changed_obstacles_for_repair = int(
            rospy.get_param("~lpa_max_changed_obstacles_for_repair", 500)
        )

        # Speed-scaled A* obstacle padding: when the robot moves fast, inflate the
        # A* human voxels by extra clearance so the planner reroutes wider. Padding
        # is measured from live TCP speed during execution. Default gain=0 keeps
        # current behavior (padding always 0). Tune on hardware (calibration knob).
        self.astar_speed_padding_gain = max(
            0.0, float(rospy.get_param("~astar_speed_padding_gain", 0.0))
        )
        self.astar_speed_padding_deadband_mps = max(
            0.0, float(rospy.get_param("~astar_speed_padding_deadband_mps", 0.05))
        )
        self.astar_max_speed_padding_m = max(
            0.0, float(rospy.get_param("~astar_max_speed_padding_m", 0.05))
        )
        self.astar_speed_padding_smoothing_alpha = min(
            1.0, max(0.0, float(rospy.get_param("~astar_speed_padding_smoothing_alpha", 0.7)))
        )
        # In-motion A* re-check period. <=0 disables the re-check (padding still
        # inflates obstacles, just no mid-segment reroute trigger).
        self.astar_recheck_period_sec = max(
            0.0, float(rospy.get_param("~astar_recheck_period_sec", 0.2))
        )

        # False-positive filter for points detected on the robot itself or on
        # the planned robot path. This is separate from human safety distance.
        # If a camera point lies within this small band, treat it as robot
        # self-detection and ignore it before A*, trajectory safety, and
        # emergency-stop checks.
        self.ignore_human_points_near_robot_path = rospy.get_param(
            "~ignore_human_points_near_robot_path",
            True,
        )
        self.robot_path_false_positive_distance = float(
            rospy.get_param("~robot_path_false_positive_distance", 0.05)
        )
        self.robot_path_false_positive_filter_current_robot = rospy.get_param(
            "~robot_path_false_positive_filter_current_robot",
            True,
        )

        self.status_pub = rospy.Publisher(
            "/ur3_fixed_joint_path_status",
            String,
            queue_size=10,
        )
        self.astar_path_pub = rospy.Publisher(
            "/hrc_path_text",
            String,
            queue_size=10,
        )
        self.astar_marker_pub = rospy.Publisher(
            "/hrc_astar_voxel_markers",
            MarkerArray,
            queue_size=1,
        )
        self.planning_time_pub = rospy.Publisher(
            "/hrc_planning_time_ms",
            Float32,
            queue_size=10,
        )
        self.astar_planning_time_pub = rospy.Publisher(
            "/hrc_astar_planning_time_ms",
            Float32,
            queue_size=10,
        )
        self.moveit_planning_time_pub = rospy.Publisher(
            "/hrc_moveit_planning_time_ms",
            Float32,
            queue_size=10,
        )
        self.execution_time_pub = rospy.Publisher(
            "/hrc_execution_time_ms",
            Float32,
            queue_size=10,
        )
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = RobotCommander()
        self.group = self.create_move_group()

        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.set_max_velocity_scaling_factor(self.velocity_scale)
        self.group.set_max_acceleration_scaling_factor(self.acceleration_scale)
        self.group.set_goal_joint_tolerance(0.01)
        self.group.allow_replanning(True)

        self.active_joints = self.group.get_active_joints()
        self.end_effector_link = rospy.get_param(
            "~end_effector_link",
            self.group.get_end_effector_link(),
        )
        self.safety_link_names = self.get_safety_link_names()

        self.map_size_x = max(1, int(round((self.map_x_max - self.map_x_min) / self.voxel_size)) + 1)
        self.map_size_y = max(1, int(round((self.map_y_max - self.map_y_min) / self.voxel_size)) + 1)
        self.map_size_z = max(1, int(round((self.map_z_max - self.map_z_min) / self.voxel_size)) + 1)
        if self.guard_planner_type == "lpa_star":
            # Drop-in: same plan_with_info/replan_with_info/set_penalty_cells API.
            self.astar = LPAStar3D(
                size_x=self.map_size_x,
                size_y=self.map_size_y,
                size_z=self.map_size_z,
                diagonal=True,
                max_time_ms=self.ara_max_time_ms,
                max_steps=self.ara_max_steps,
                epsilon=self.lpa_epsilon,
                start_reuse_radius_voxels=self.lpa_start_reuse_radius_voxels,
                max_changed_obstacles_for_repair=self.lpa_max_changed_obstacles_for_repair,
            )
        else:
            self.astar = AStarImproved3D(
                size_x=self.map_size_x,
                size_y=self.map_size_y,
                size_z=self.map_size_z,
                diagonal=True,
                epsilon_start=self.ara_epsilon_start,
                epsilon_final=self.ara_epsilon_final,
                epsilon_decay=self.ara_epsilon_decay,
                max_time_ms=self.ara_max_time_ms,
                max_steps=self.ara_max_steps,
            )
        self._astar_last_goal: Optional[Voxel] = None
        self._first_plan_done: bool = False

        # Speed-scaled padding state. Live value tracked during execution; the
        # latch carries the in-motion padding into the next waypoint's plan
        # (robot is at rest there, so live speed would otherwise read ~0).
        self._current_tcp_speed: float = 0.0
        self._speed_pad_voxels_live: int = 0
        self._reroute_pad_voxels: int = 0
        self._prev_tcp_pos: Optional[Tuple[float, float, float]] = None
        self._prev_tcp_time: Optional[float] = None
        self._exec_goal_voxel: Optional[Voxel] = None
        self._last_exec_recheck: float = 0.0

        self.last_human_point: Optional[PointStamped] = None
        self.latest_skeleton_by_id: Dict[int, Point] = {}
        self.latest_selected_human_points: List[Point] = []
        self.latest_skeleton_points: List[Point] = []
        self.latest_skeleton_stamp: Optional[rospy.Time] = None
        # Cached human obstacle anchors: list of {"point": Point, "stamp": Time}.
        # Stabilizes A* obstacles against jitter / transient skeleton dropouts.
        self._obstacle_anchors: List[Dict[str, object]] = []
        # Lock guards obstacle cache + last-plan viz state, shared between the
        # planner thread and the obstacle-publish timer thread.
        self._obstacle_lock = threading.Lock()
        self._viz_start_voxel: Optional[Voxel] = None
        self._viz_goal_voxel: Optional[Voxel] = None
        self._viz_path: List[Voxel] = []
        self._viz_stamp: Optional[rospy.Time] = None
        self._obstacle_timer: Optional[rospy.Timer] = None
        self.latest_scene_status_text = ""
        self.latest_scene_status_stamp: Optional[rospy.Time] = None
        if self.use_legacy_human_point:
            rospy.Subscriber(
                self.human_topic,
                PointStamped,
                self.human_callback,
                queue_size=1,
            )
        if self.use_skeleton_obstacles:
            rospy.Subscriber(
                self.human_skeleton_topic,
                PoseArray,
                self.human_skeleton_callback,
                queue_size=1,
            )
        if self.enable_moveit_scene_sync:
            rospy.Subscriber(
                self.moveit_scene_status_topic,
                String,
                self.moveit_scene_status_callback,
                queue_size=10,
            )

        self.fk_service_name = rospy.get_param("~fk_service", "/compute_fk")
        self.fk_client: Optional[rospy.ServiceProxy] = None
        if self.enable_astar_guard:
            try:
                rospy.wait_for_service(self.fk_service_name, timeout=self.fk_wait_timeout)
                self.fk_client = rospy.ServiceProxy(self.fk_service_name, GetPositionFK)
            except rospy.ROSException as exc:
                rospy.logwarn(
                    f"FK service {self.fk_service_name} is not available, "
                    f"A* guard will rely on MoveIt collision replanning only: {exc}"
                )

        # State-validity service: lets the node detect a goal joint state that is
        # itself in collision (OMPL rejects it in ~tens of ms and the replan loop
        # would otherwise spin forever). Optional: if unavailable the node keeps
        # its previous behaviour (see should_escalate_to_detour / None handling).
        self.state_validity_service_name = rospy.get_param(
            "~state_validity_service", "/check_state_validity"
        )
        self.state_validity_client: Optional[rospy.ServiceProxy] = None
        try:
            rospy.wait_for_service(
                self.state_validity_service_name, timeout=self.fk_wait_timeout
            )
            self.state_validity_client = rospy.ServiceProxy(
                self.state_validity_service_name, GetStateValidity
            )
        except rospy.ROSException as exc:
            rospy.logwarn(
                f"State-validity service {self.state_validity_service_name} is not "
                f"available; goal-collision escalation disabled: {exc}"
            )

        rospy.loginfo(f"Planning group: {self.group_name}")
        rospy.loginfo(f"Active joints: {self.active_joints}")
        rospy.loginfo(
            "goal-collision escalation = %s (service %s)"
            % (
                "on" if self.state_validity_client is not None else "off",
                self.state_validity_service_name,
            )
        )
        rospy.loginfo(f"target_frame = {self.target_frame}")
        rospy.loginfo(f"auto_execute = {self.auto_execute}")
        rospy.loginfo(f"enable_astar_guard = {self.enable_astar_guard}")
        rospy.loginfo(f"guard_planner_type = {self.guard_planner_type}")
        rospy.loginfo(f"human_topic = {self.human_topic}")
        rospy.loginfo(f"use_skeleton_obstacles = {self.use_skeleton_obstacles}")
        rospy.loginfo(f"human_skeleton_topic = {self.human_skeleton_topic}")
        rospy.loginfo(f"enable_moveit_scene_sync = {self.enable_moveit_scene_sync}")
        rospy.loginfo(f"moveit_scene_status_topic = {self.moveit_scene_status_topic}")
        rospy.loginfo(f"require_moveit_scene_ready = {self.require_moveit_scene_ready}")
        rospy.loginfo(f"use_legacy_human_point = {self.use_legacy_human_point}")
        rospy.loginfo(f"human_point_selection_mode = {self.human_point_selection_mode}")
        rospy.loginfo(f"max_human_points_for_astar = {self.max_human_points_for_astar}")
        rospy.loginfo(f"ara_epsilon_start = {self.ara_epsilon_start:.2f}")
        rospy.loginfo(f"ara_epsilon_final = {self.ara_epsilon_final:.2f}")
        rospy.loginfo(f"ara_epsilon_decay = {self.ara_epsilon_decay:.2f}")
        rospy.loginfo(f"ara_max_time_ms = {self.ara_max_time_ms:.2f}")
        rospy.loginfo(f"ara_max_steps = {self.ara_max_steps}")
        rospy.loginfo(f"end_effector_link = {self.end_effector_link}")
        rospy.loginfo(f"human_safety_distance = {self.human_safety_distance:.3f} m")
        rospy.loginfo(f"robot_body_radius = {self.robot_body_radius:.3f} m")
        rospy.loginfo(f"human_body_radius = {self.human_body_radius:.3f} m")
        rospy.loginfo(f"safety_link_names = {self.safety_link_names}")
        rospy.loginfo(f"resampled_waypoint_count = {self.resampled_waypoint_count}")
        rospy.loginfo(f"wait_after_waypoint = {self.wait_after_waypoint:.3f} s")
        rospy.loginfo(f"hand_waypoint_collision_radius = {self.hand_waypoint_collision_radius:.3f} m")
        rospy.loginfo(f"detour_z_offset = {self.detour_z_offset:.3f} m")
        rospy.loginfo(f"max_detour_attempts = {self.max_detour_attempts}")
        rospy.loginfo(f"detour_retry_period = {self.detour_retry_period:.3f} s")
        rospy.loginfo(f"detour_planning_time = {self.detour_planning_time:.3f} s")
        rospy.loginfo(f"detour_hold_poll_period = {self.detour_hold_poll_period:.3f} s")
        rospy.loginfo(f"min_cartesian_fraction = {self.min_cartesian_fraction:.2f}")
        rospy.loginfo(
            f"ignore_human_points_near_robot_path = {self.ignore_human_points_near_robot_path}"
        )
        rospy.loginfo(
            f"robot_path_false_positive_distance = {self.robot_path_false_positive_distance:.3f} m"
        )
        rospy.loginfo(
            "robot_path_false_positive_filter_current_robot = "
            f"{self.robot_path_false_positive_filter_current_robot}"
        )

        raw_path_a_to_b = self.build_path_a_to_b()
        self.path_a_to_b = self.resample_joint_path(
            raw_path_a_to_b,
            self.resampled_waypoint_count,
        )

        rospy.loginfo(
            f"Loaded {len(raw_path_a_to_b)} raw waypoints, "
            f"resampled to {len(self.path_a_to_b)} joint waypoints."
        )

        if self.obstacle_publish_rate > 0.0:
            self._obstacle_timer = rospy.Timer(
                rospy.Duration(1.0 / self.obstacle_publish_rate),
                self._obstacle_timer_cb,
            )
            rospy.loginfo(
                "Continuous obstacle publisher ON at %.1f Hz",
                self.obstacle_publish_rate,
            )

        self.plan_logger = PlanLogger(self.plan_log_dir, self.enable_plan_log)
        if self.plan_logger.enabled:
            rospy.loginfo("Plan logging ON -> %s", self.plan_logger.csv_path)
        rospy.on_shutdown(self.plan_logger.close)

        self.breadcrumb = BreadcrumbCache(
            self.breadcrumb_revisit_threshold,
            self.breadcrumb_max_nodes,
            self.enable_breadcrumb,
        )
        self._breadcrumb_last_record = 0.0
        if self.enable_breadcrumb:
            rospy.loginfo(
                "Breadcrumb cache ON (period=%.2fs, revisit=%.3fm, max=%d, fallback=%s)",
                self.breadcrumb_record_period,
                self.breadcrumb_revisit_threshold,
                self.breadcrumb_max_nodes,
                self.enable_breadcrumb_fallback,
            )

        self._setup_region_preference()
        self._setup_pan_limit()

    def _setup_pan_limit(self) -> None:
        """Hard-constrain shoulder_pan to the A->B sweep range on every MoveIt
        plan, so the executed trajectory stays in the shoulder_pan zone (cable
        protection). Unlike the A* region penalty, this binds OMPL itself."""
        if not self.enable_pan_limit:
            return

        if self.shoulder_pan_min is not None and self.shoulder_pan_max is not None:
            pan_min = float(self.shoulder_pan_min)
            pan_max = float(self.shoulder_pan_max)
        else:
            pans = [jm["shoulder_pan_joint"] for jm in self.build_path_a_to_b()]
            if not pans:
                rospy.logwarn("Pan limit disabled: no waypoints to derive range.")
                return
            pan_min = min(pans)
            pan_max = max(pans)

        lo = pan_min - self.pan_limit_margin
        hi = pan_max + self.pan_limit_margin
        center = 0.5 * (lo + hi)

        constraint = Constraints()
        constraint.name = "shoulder_pan_region"
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "shoulder_pan_joint"
        joint_constraint.position = center
        joint_constraint.tolerance_above = hi - center
        joint_constraint.tolerance_below = center - lo
        joint_constraint.weight = 1.0
        constraint.joint_constraints.append(joint_constraint)
        self.group.set_path_constraints(constraint)
        rospy.loginfo(
            "Pan limit ON: shoulder_pan in [%.3f, %.3f] rad ([%.1f, %.1f] deg), "
            "enforced as MoveIt path constraint.",
            lo, hi, math.degrees(lo), math.degrees(hi),
        )

    def _setup_region_preference(self) -> None:
        """Build the soft A* penalty for voxels outside the workspace box swept
        by the A->B waypoints (the shoulder_pan working zone)."""
        if not self.enable_region_preference or self.region_penalty_weight <= 0.0:
            return

        tcps: List[Tuple[float, float, float]] = []
        for joint_map in self.build_path_a_to_b():
            pose = self._joint_map_goal_xyz(joint_map)
            if pose is not None:
                tcps.append(pose)
        if len(tcps) < 2:
            rospy.logwarn(
                "Region preference disabled: could not FK waypoints "
                "(fk_client=%s).", self.fk_client is not None,
            )
            return

        margin = self.region_margin
        x_min = min(p[0] for p in tcps) - margin
        x_max = max(p[0] for p in tcps) + margin
        y_min = min(p[1] for p in tcps) - margin
        y_max = max(p[1] for p in tcps) + margin
        z_min = min(p[2] for p in tcps) - margin
        z_max = max(p[2] for p in tcps) + margin

        penalty_cells: set[Voxel] = set()
        for ix in range(self.map_size_x):
            for iy in range(self.map_size_y):
                for iz in range(self.map_size_z):
                    wx, wy, wz = self.voxel_to_world((ix, iy, iz))
                    if not (
                        x_min <= wx <= x_max
                        and y_min <= wy <= y_max
                        and z_min <= wz <= z_max
                    ):
                        penalty_cells.add((ix, iy, iz))

        self.astar.set_penalty_cells(penalty_cells, self.region_penalty_weight)
        rospy.loginfo(
            "Region preference ON: box x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] "
            "margin=%.2f weight=%.1f penalised=%d/%d voxels",
            x_min, x_max, y_min, y_max, z_min, z_max,
            margin, self.region_penalty_weight,
            len(penalty_cells), self.map_size_x * self.map_size_y * self.map_size_z,
        )

    def create_move_group(self) -> MoveGroupCommander:
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            try:
                return MoveGroupCommander(
                    self.group_name,
                    wait_for_servers=self.move_group_retry_period,
                )
            except RuntimeError as exc:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed >= self.move_group_wait_timeout:
                    raise RuntimeError(
                        f"Unable to connect to move_group action server for group "
                        f"'{self.group_name}' after {elapsed:.1f}s. "
                        "Start move_group before planner_ab_replan_node.py."
                    ) from exc

                rospy.logwarn(
                    f"Waiting for move_group action server for group '{self.group_name}' "
                    f"({elapsed:.1f}/{self.move_group_wait_timeout:.1f}s): {exc}"
                )
                rospy.sleep(self.move_group_retry_period)

        raise rospy.ROSInterruptException()

    def get_safety_link_names(self) -> List[str]:
        configured = rospy.get_param("~safety_link_names", "")
        if configured:
            return [name.strip() for name in configured.split(",") if name.strip()]

        preferred = [
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "tool0",
        ]

        try:
            robot_links = set(self.robot.get_link_names(group=self.group_name))
            selected = [name for name in preferred if name in robot_links]
            if selected:
                return selected
        except Exception as exc:
            rospy.logwarn(f"Cannot query robot link names for safety monitor: {exc}")

        if self.end_effector_link:
            return [self.end_effector_link]
        return ["tool0"]

    def human_callback(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            rospy.logwarn_throttle(
                2.0,
                "planner_ab_replan_node expected human point in %s, got %s",
                self.target_frame,
                msg.header.frame_id,
            )
        self.last_human_point = msg

    def moveit_scene_status_callback(self, msg: String) -> None:
        self.latest_scene_status_text = msg.data
        self.latest_scene_status_stamp = _ros_now()

    def scene_status_is_fresh(self) -> bool:
        if not self.latest_scene_status_stamp:
            return False
        now = _ros_now()
        if now == rospy.Time(0):
            return False
        age = (now - self.latest_scene_status_stamp).to_sec()
        return age <= self.scene_status_timeout_sec

    def scene_status_age(self) -> Optional[float]:
        if not self.latest_scene_status_stamp:
            return None
        now = _ros_now()
        if now == rospy.Time(0):
            return None
        return (now - self.latest_scene_status_stamp).to_sec()

    def scene_status_matches(self, keywords: List[str]) -> bool:
        status = self.latest_scene_status_text
        if not status:
            return False
        return any(keyword and keyword in status for keyword in keywords)

    def moveit_scene_ready(self) -> bool:
        if not self.enable_moveit_scene_sync:
            return True
        status = self.latest_scene_status_text
        if "REMOVED" in status or "expected_absent" in status:
            return False
        if self.scene_status_matches(self.scene_removed_keywords):
            return False
        return self.scene_status_matches(self.scene_ready_keywords)

    def scene_ready_for_planning(self, human_points_active: bool) -> bool:
        if not self.enable_moveit_scene_sync:
            self.status_pub.publish("SCENE_SYNC_DISABLED")
            return True
        if not self.require_moveit_scene_ready:
            return True
        if not human_points_active:
            return True
        if not self.scene_status_is_fresh():
            return False
        return self.moveit_scene_ready()

    def is_valid_pose_point(self, pose) -> bool:
        return (
            math.isfinite(pose.position.x)
            and math.isfinite(pose.position.y)
            and math.isfinite(pose.position.z)
        )

    def decode_skeleton_pose_array(self, msg: PoseArray) -> Dict[int, Point]:
        skeleton_dict: Dict[int, Point] = {}
        decoded = pose_array_to_numeric_joint_dict(msg, self.tracked_joint_ids)
        for joint_id, coordinates in decoded.items():
            point = Point()
            point.x, point.y, point.z = coordinates
            skeleton_dict[joint_id] = point
        return skeleton_dict

    def is_hand_joint(self, joint_id: int) -> bool:
        return 100 <= joint_id <= 120 or 200 <= joint_id <= 220

    def is_body_joint(self, joint_id: int) -> bool:
        return joint_id in self.body_joint_ids

    def limit_selected_points(self, points: List[Point]) -> List[Point]:
        if self.max_human_points_for_astar <= 0:
            return list(points)
        return list(points[: self.max_human_points_for_astar])

    def points_for_joint_ids(
        self,
        skeleton_by_id: Dict[int, Point],
        joint_ids: List[int],
    ) -> List[Point]:
        return [skeleton_by_id[joint_id] for joint_id in joint_ids if joint_id in skeleton_by_id]

    def select_planner_human_points(self, skeleton_by_id: Dict[int, Point]) -> List[Point]:
        if not skeleton_by_id:
            return []

        mode = self.human_point_selection_mode.strip().lower()
        body_points = self.points_for_joint_ids(skeleton_by_id, self.selected_body_joint_ids)
        left_hand_points = self.points_for_joint_ids(skeleton_by_id, self.selected_left_hand_joint_ids)
        right_hand_points = self.points_for_joint_ids(skeleton_by_id, self.selected_right_hand_joint_ids)
        hand_points = left_hand_points + right_hand_points

        if mode == "all":
            ordered = [skeleton_by_id[joint_id] for joint_id in self.tracked_joint_ids if joint_id in skeleton_by_id]
            extras = [
                point
                for joint_id, point in skeleton_by_id.items()
                if joint_id not in set(self.tracked_joint_ids)
            ]
            return self.limit_selected_points(ordered + extras)

        if mode == "body_only":
            return self.limit_selected_points(body_points)

        if mode == "hands_only":
            return self.limit_selected_points(hand_points)

        if mode == "hands_priority":
            if self.prefer_hand_points:
                return self.limit_selected_points(hand_points + body_points)
            return self.limit_selected_points(body_points + hand_points)

        # collision_reduced: wrists, palm/fingertips and major arm/body anchors.
        if self.prefer_hand_points:
            return self.limit_selected_points(hand_points + body_points)
        return self.limit_selected_points(body_points + hand_points)

    def human_skeleton_callback(self, msg: PoseArray) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            rospy.logwarn_throttle(
                2.0,
                "planner_ab_replan_node expected human skeleton in %s, got %s",
                self.target_frame,
                msg.header.frame_id,
            )
            return

        skeleton_by_id = self.decode_skeleton_pose_array(msg)
        selected_points = self.select_planner_human_points(skeleton_by_id)

        self.latest_skeleton_by_id = skeleton_by_id
        self.latest_selected_human_points = selected_points
        self.latest_skeleton_points = selected_points
        self.latest_skeleton_stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        self.status_pub.publish(
            "SKELETON_ACTIVE points=%d selected=%d mode=%s"
            % (len(skeleton_by_id), len(selected_points), self.human_point_selection_mode)
        )

    def make_joint_map(
        self,
        elbow_joint: float,
        shoulder_lift_joint: float,
        shoulder_pan_joint: float,
        wrist_1_joint: float,
        wrist_2_joint: float,
        wrist_3_joint: float,
    ) -> JointMap:
        return {
            "shoulder_pan_joint": shoulder_pan_joint,
            "shoulder_lift_joint": shoulder_lift_joint,
            "elbow_joint": elbow_joint,
            "wrist_1_joint": wrist_1_joint,
            "wrist_2_joint": wrist_2_joint,
            "wrist_3_joint": wrist_3_joint,
        }

    def build_path_a_to_b(self) -> List[JointMap]:
        # Dữ liệu gốc bạn đưa ra có thứ tự:
        # elbow_joint, shoulder_lift_joint, shoulder_pan_joint,
        # wrist_1_joint, wrist_2_joint, wrist_3_joint

        return [
            self.make_joint_map(
                elbow_joint=-2.160663906727926,
                shoulder_lift_joint=-2.106152359639303,
                shoulder_pan_joint=0.0037121137138456106,
                wrist_1_joint=-0.4163420836078089,
                wrist_2_joint=1.678815484046936,
                wrist_3_joint=0.07622154802083969,
            ),
            self.make_joint_map(
                elbow_joint=-1.3727811018573206,
                shoulder_lift_joint=-1.6279404799090784,
                shoulder_pan_joint=0.023216815665364265,
                wrist_1_joint=-1.6459625403033655,
                wrist_2_joint=1.6788873672485352,
                wrist_3_joint=0.07784877717494965,
            ),
            self.make_joint_map(
                elbow_joint=-1.3802769819842737,
                shoulder_lift_joint=-1.628491226826803,
                shoulder_pan_joint=-0.734206501637594,
                wrist_1_joint=-1.6501067320453089,
                wrist_2_joint=1.6788992881774902,
                wrist_3_joint=0.07760946452617645,
            ),
            self.make_joint_map(
                elbow_joint=-1.1364339033709925,
                shoulder_lift_joint=-1.5298474470721644,
                shoulder_pan_joint=-1.3637602964984339,
                wrist_1_joint=-1.9822285811053675,
                wrist_2_joint=1.696462631225586,
                wrist_3_joint=0.0776214525103569,
            ),
            self.make_joint_map(
                elbow_joint=-1.7341268698321741,
                shoulder_lift_joint=-1.6721509138690394,
                shoulder_pan_joint=-2.069549862538473,
                wrist_1_joint=-1.2379863897906702,
                wrist_2_joint=1.6641021966934204,
                wrist_3_joint=0.07610207796096802,
            ),
            self.make_joint_map(
                elbow_joint=-2.1694443861590784,
                shoulder_lift_joint=-1.696669880543844,
                shoulder_pan_joint=-2.304720942174093,
                wrist_1_joint=-0.8804991880999964,
                wrist_2_joint=1.550010085105896,
                wrist_3_joint=0.845086395740509,
            ),
            self.make_joint_map(
                elbow_joint=-2.3058422247516077,
                shoulder_lift_joint=-1.9987319151507776,
                shoulder_pan_joint=-2.1886518637286585,
                wrist_1_joint=-0.3415635267840784,
                wrist_2_joint=1.618721604347229,
                wrist_3_joint=0.9668446183204651,
            ),
        ]

    def interpolate_joint_maps(
        self,
        start: JointMap,
        end: JointMap,
        ratio: float,
    ) -> JointMap:
        return {
            joint_name: start[joint_name] + (end[joint_name] - start[joint_name]) * ratio
            for joint_name in start
        }

    def joint_distance(self, a: JointMap, b: JointMap) -> float:
        return math.sqrt(sum((a[name] - b[name]) ** 2 for name in a))

    def resample_joint_path(self, path: List[JointMap], target_count: int) -> List[JointMap]:
        if len(path) <= 1 or target_count <= len(path):
            return list(path)

        segment_lengths = [
            self.joint_distance(path[index], path[index + 1])
            for index in range(len(path) - 1)
        ]
        total_length = sum(segment_lengths)
        if total_length <= 1e-9:
            return list(path)

        cumulative = [0.0]
        for length in segment_lengths:
            cumulative.append(cumulative[-1] + length)

        resampled: List[JointMap] = []
        for sample_index in range(target_count):
            target_distance = total_length * sample_index / (target_count - 1)

            if sample_index == target_count - 1:
                resampled.append(dict(path[-1]))
                continue

            segment_index = 0
            while (
                segment_index < len(segment_lengths) - 1
                and cumulative[segment_index + 1] < target_distance
            ):
                segment_index += 1

            segment_start = cumulative[segment_index]
            segment_length = segment_lengths[segment_index]
            ratio = 0.0
            if segment_length > 1e-9:
                ratio = (target_distance - segment_start) / segment_length

            resampled.append(
                self.interpolate_joint_maps(
                    path[segment_index],
                    path[segment_index + 1],
                    ratio,
                )
            )

        return resampled

    def joint_map_to_group_order(self, joint_map: JointMap) -> List[float]:
        values: List[float] = []

        for joint_name in self.active_joints:
            if joint_name not in joint_map:
                raise RuntimeError(f"Missing joint value for {joint_name}")

            values.append(joint_map[joint_name])

        return values

    def extract_plan_result(self, result) -> Tuple[bool, Optional[object]]:
        success = False
        plan = None

        if isinstance(result, tuple):
            if len(result) >= 2:
                success = bool(result[0])
                plan = result[1]
        else:
            plan = result
            success = plan is not None

        if plan is None:
            return False, None

        if not hasattr(plan, "joint_trajectory"):
            return False, None

        if not plan.joint_trajectory.points:
            return False, None

        return success, plan

    def world_to_voxel(self, x: float, y: float, z: float) -> Voxel:
        ix = int(round((x - self.map_x_min) / self.voxel_size))
        iy = int(round((y - self.map_y_min) / self.voxel_size))
        iz = int(round((z - self.map_z_min) / self.voxel_size))

        ix = max(0, min(self.map_size_x - 1, ix))
        iy = max(0, min(self.map_size_y - 1, iy))
        iz = max(0, min(self.map_size_z - 1, iz))
        return ix, iy, iz

    def voxel_to_world(self, voxel: Voxel) -> Tuple[float, float, float]:
        ix, iy, iz = voxel
        return (
            self.map_x_min + ix * self.voxel_size,
            self.map_y_min + iy * self.voxel_size,
            self.map_z_min + iz * self.voxel_size,
        )

    def voxel_to_text(self, path: List[Voxel]) -> str:
        return " -> ".join(f"({x},{y},{z})" for x, y, z in path)

    def make_voxel_marker(
        self,
        voxel: Voxel,
        marker_id: int,
        color: ColorRGBA,
        namespace: str,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = self.voxel_to_world(voxel)
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        marker.color = color
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def publish_astar_markers(
        self,
        start_voxel: Optional[Voxel],
        goal_voxel: Optional[Voxel],
        path: List[Voxel],
        obstacles: set[Voxel],
    ) -> None:
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = self.target_frame
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "hrc_astar_voxels"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 1

        def add_marker(voxel: Voxel, color: ColorRGBA, namespace: str) -> None:
            nonlocal marker_id
            marker_array.markers.append(
                self.make_voxel_marker(voxel, marker_id, color, namespace)
            )
            marker_id += 1

        for voxel in sorted(obstacles)[: self.max_astar_marker_voxels]:
            add_marker(
                voxel,
                ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.65),
                "hrc_astar_obstacles",
            )

        for voxel in path:
            if voxel == start_voxel or voxel == goal_voxel or voxel in obstacles:
                continue
            add_marker(
                voxel,
                ColorRGBA(r=0.0, g=0.8, b=0.2, a=0.45),
                "hrc_astar_path",
            )

        if start_voxel is not None:
            add_marker(start_voxel, ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0), "hrc_astar_start")
        if goal_voxel is not None:
            add_marker(goal_voxel, ColorRGBA(r=0.0, g=0.25, b=1.0, a=1.0), "hrc_astar_goal")
        self.astar_marker_pub.publish(marker_array)

    def store_plan_viz(
        self,
        start_voxel: Voxel,
        goal_voxel: Voxel,
        path: List[Voxel],
    ) -> None:
        """Record the latest plan's path/start/goal for the marker renderer."""
        with self._obstacle_lock:
            self._viz_start_voxel = start_voxel
            self._viz_goal_voxel = goal_voxel
            self._viz_path = list(path)
            self._viz_stamp = rospy.Time.now()

    def render_astar_markers(self) -> None:
        """Build obstacles fresh and publish them plus the recent plan overlay.

        Obstacles are always drawn (continuous build); path/start/goal only stay
        for ``obstacle_path_viz_hold`` seconds after the last plan.
        """
        obstacles = self.active_human_voxels()
        with self._obstacle_lock:
            start_voxel = self._viz_start_voxel
            goal_voxel = self._viz_goal_voxel
            path = list(self._viz_path)
            stamp = self._viz_stamp
        show_overlay = (
            stamp is not None
            and (rospy.Time.now() - stamp).to_sec() <= self.obstacle_path_viz_hold
        )
        if not show_overlay:
            start_voxel = None
            goal_voxel = None
            path = []
        self.publish_astar_markers(start_voxel, goal_voxel, path, obstacles)

    def _obstacle_timer_cb(self, _event: object) -> None:
        try:
            self.render_astar_markers()
        except Exception as exc:  # keep timer thread alive
            rospy.logwarn_throttle(5.0, "Obstacle marker timer error: %s", exc)

    def inflate_voxel(self, center: Voxel, radius: int) -> set[Voxel]:
        cx, cy, cz = center
        voxels: set[Voxel] = set()
        # Sphere inflation keeps only voxels within the clearance radius and
        # drops the cube corners (which sit beyond clearance), cutting the voxel
        # count ~3-4x without reducing the intended clearance ball.
        sphere = self.obstacle_inflate_sphere and radius > 0
        radius_sq = radius * radius

        for x in range(cx - radius, cx + radius + 1):
            for y in range(cy - radius, cy + radius + 1):
                for z in range(cz - radius, cz + radius + 1):
                    if sphere:
                        dx, dy, dz = x - cx, y - cy, z - cz
                        if dx * dx + dy * dy + dz * dz > radius_sq:
                            continue
                    if 0 <= x < self.map_size_x and 0 <= y < self.map_size_y and 0 <= z < self.map_size_z:
                        voxels.add((x, y, z))

        return voxels

    def active_human_voxel(self) -> Optional[Voxel]:
        if self.last_human_point is None:
            return None

        age = (rospy.Time.now() - self.last_human_point.header.stamp).to_sec()
        if self.last_human_point.header.stamp != rospy.Time(0) and age > self.human_timeout_sec:
            return None

        point = self.last_human_point.point
        return self.world_to_voxel(point.x, point.y, point.z)

    def active_skeleton_points(self) -> List[Point]:
        if not self.use_skeleton_obstacles or self.latest_skeleton_stamp is None:
            return []

        age = (rospy.Time.now() - self.latest_skeleton_stamp).to_sec()
        if age > self.human_skeleton_timeout_sec:
            self.status_pub.publish("SKELETON_STALE age=%.3f" % age)
            return []

        return list(self.latest_selected_human_points)

    def latest_human_points_raw(self) -> List[Point]:
        skeleton_points = self.active_skeleton_points()
        if skeleton_points:
            return skeleton_points

        if not self.use_legacy_human_point:
            return []

        human_msg = self.latest_human_point()
        if human_msg is None:
            return []
        return [human_msg.point]

    def can_plan_to_waypoint(self) -> bool:
        human_points = self.latest_human_points_raw()

        if not human_points:
            self.status_pub.publish("NO_SELECTED_HUMAN_POINTS")

        if not self.scene_ready_for_planning(True):
            age = self.scene_status_age()
            age_text = "none" if age is None else "%.3f" % age
            self.status_pub.publish(
                "WAITING_FOR_MOVEIT_SCENE status=%s age=%s"
                % (self.latest_scene_status_text or "none", age_text)
            )
            rospy.logwarn_throttle(
                2.0,
                "Waiting for MoveIt scene manager before planning: status='%s' age=%s.",
                self.latest_scene_status_text or "none",
                age_text,
            )
            return False

        if self.enable_moveit_scene_sync and bool(human_points) and self.moveit_scene_ready():
            age = self.scene_status_age()
            self.status_pub.publish(
                "MOVEIT_SCENE_READY status=%s age=%.3f"
                % (self.latest_scene_status_text, 0.0 if age is None else age)
            )

        return True

    def latest_human_points(self) -> List[Point]:
        return self.latest_human_points_raw()

    def _is_valid_human_point(self, point: Point) -> bool:
        """Reject NaN/inf or out-of-map points (invalid voxel)."""
        margin = self.voxel_size
        for value in (point.x, point.y, point.z):
            if value is None or math.isnan(value) or math.isinf(value):
                return False
        if not (self.map_x_min - margin <= point.x <= self.map_x_max + margin):
            return False
        if not (self.map_y_min - margin <= point.y <= self.map_y_max + margin):
            return False
        if not (self.map_z_min - margin <= point.z <= self.map_z_max + margin):
            return False
        return True

    @staticmethod
    def _point_distance(a: Point, b: Point) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def _nearest_human_xyz(self, gx: float, gy: float, gz: float):
        """Return (x, y, z) of the human point nearest to a goal, or None."""
        nearest = None
        best = float("inf")
        for point in self.latest_human_points():
            dist = math.sqrt((point.x - gx) ** 2 + (point.y - gy) ** 2 + (point.z - gz) ** 2)
            if dist < best:
                best = dist
                nearest = (point.x, point.y, point.z)
        return nearest

    def _joint_map_goal_xyz(self, joint_map: JointMap):
        """Best-effort TCP world coords for a joint target (via FK), or None."""
        try:
            target_pose = self.target_tcp_pose_for_joint_map(joint_map)
        except Exception:
            return None
        if target_pose is None:
            return None
        p = target_pose.pose.position
        return (p.x, p.y, p.z)

    def stable_human_points(self) -> List[Point]:
        """Return human obstacle points smoothed by the stability cache.

        - Points within ``obstacle_stability_threshold`` of a cached anchor keep
          the cached position (same voxel) so the obstacle does not flicker.
        - Anchors not seen this cycle are held for ``obstacle_cache_hold_sec``
          to bridge transient skeleton dropouts, then dropped.
        - Invalid points (NaN/inf/out-of-map) are discarded ("voxel not valid").
        """
        raw = [p for p in self.latest_human_points() if self._is_valid_human_point(p)]

        if not self.enable_obstacle_cache:
            return raw

        now = rospy.Time.now()
        threshold = self.obstacle_stability_threshold

        with self._obstacle_lock:
            used = [False] * len(self._obstacle_anchors)

            for point in raw:
                best_i = -1
                best_d = threshold
                for i, anchor in enumerate(self._obstacle_anchors):
                    if used[i]:
                        continue
                    dist = self._point_distance(point, anchor["point"])  # type: ignore[arg-type]
                    if dist <= best_d:
                        best_d = dist
                        best_i = i
                if best_i >= 0:
                    # Within threshold: keep cached position, refresh timestamp.
                    used[best_i] = True
                    self._obstacle_anchors[best_i]["stamp"] = now
                else:
                    # New or moved-beyond-threshold point: add a fresh anchor.
                    self._obstacle_anchors.append({"point": point, "stamp": now})
                    used.append(True)

            kept: List[Dict[str, object]] = []
            for i, anchor in enumerate(self._obstacle_anchors):
                if used[i]:
                    kept.append(anchor)
                elif (now - anchor["stamp"]).to_sec() <= self.obstacle_cache_hold_sec:  # type: ignore[operator]
                    kept.append(anchor)  # grace hold: avoid transient drop
                # else: not seen past hold window -> drop
            self._obstacle_anchors = kept

            return [anchor["point"] for anchor in self._obstacle_anchors]  # type: ignore[misc]

    def _bone_obstacle_voxels(self) -> set[Voxel]:
        """Inflate voxels along the segments connecting joints (limbs/torso).

        Samples each bone at half-voxel steps, then inflates the unique line
        voxels by ``bone_inflate_radius`` so A* sees the connections, not only
        the joint spheres. Uses the latest decoded skeleton (needs joint ids).
        """
        if not self.enable_bone_obstacles:
            return set()

        skeleton = self.latest_skeleton_by_id
        if not skeleton:
            return set()

        step = max(self.voxel_size * 0.5, 1e-3)
        line_voxels: set[Voxel] = set()

        for joint_a, joint_b in self.bone_connection_pairs:
            point_a = skeleton.get(joint_a)
            point_b = skeleton.get(joint_b)
            if point_a is None or point_b is None:
                continue
            if not (
                self._is_valid_human_point(point_a)
                and self._is_valid_human_point(point_b)
            ):
                continue

            length = self._point_distance(point_a, point_b)
            samples = max(1, int(math.ceil(length / step)))
            for index in range(samples + 1):
                ratio = index / samples
                line_voxels.add(
                    self.world_to_voxel(
                        point_a.x + (point_b.x - point_a.x) * ratio,
                        point_a.y + (point_b.y - point_a.y) * ratio,
                        point_a.z + (point_b.z - point_a.z) * ratio,
                    )
                )

        voxels: set[Voxel] = set()
        for voxel in line_voxels:
            voxels.update(self.inflate_voxel(voxel, self.bone_inflate_radius))
        return voxels

    def active_human_voxels(self) -> set[Voxel]:
        # Speed term: live padding (during motion) or the latched in-motion value
        # carried into the next waypoint's rest-replan, whichever is larger.
        speed_pad = max(self._speed_pad_voxels_live, self._reroute_pad_voxels)
        radius = max(
            self.human_inflate_radius,
            int(math.ceil(self.base_clearance() / self.voxel_size)),
        ) + speed_pad
        voxels: set[Voxel] = set()

        for point in self.stable_human_points():
            voxels.update(
                self.inflate_voxel(
                    self.world_to_voxel(point.x, point.y, point.z),
                    radius,
                )
            )

        voxels |= self._bone_obstacle_voxels()

        return voxels

    def current_tcp_voxel(self) -> Voxel:
        pose = self.group.get_current_pose(self.end_effector_link).pose
        return self.world_to_voxel(
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )

    def _speed_padding_voxels(self, speed_mps: float) -> int:
        pad_m = speed_padding_meters(
            speed_mps,
            self.astar_speed_padding_deadband_mps,
            self.astar_speed_padding_gain,
            self.astar_max_speed_padding_m,
        )
        if pad_m <= 0.0:
            return 0
        return int(math.ceil(pad_m / self.voxel_size))

    def _update_tcp_speed(self, now_t: float) -> None:
        """Sample TCP speed from successive end-effector position deltas, EMA
        smooth it, and refresh the live speed-padding voxel count. Cheap; called
        each execution-monitor tick. Produces 0 padding when the feature is off."""
        try:
            pos = self.group.get_current_pose(self.end_effector_link).pose.position
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "TCP speed sample failed: %s", exc)
            return
        cur = (pos.x, pos.y, pos.z)
        if self._prev_tcp_pos is not None and self._prev_tcp_time is not None:
            dt = now_t - self._prev_tcp_time
            if dt > 1e-6:
                raw = math.sqrt(
                    (cur[0] - self._prev_tcp_pos[0]) ** 2
                    + (cur[1] - self._prev_tcp_pos[1]) ** 2
                    + (cur[2] - self._prev_tcp_pos[2]) ** 2
                ) / dt
                alpha = self.astar_speed_padding_smoothing_alpha
                self._current_tcp_speed = (
                    alpha * self._current_tcp_speed + (1.0 - alpha) * raw
                )
        self._prev_tcp_pos = cur
        self._prev_tcp_time = now_t
        self._speed_pad_voxels_live = self._speed_padding_voxels(self._current_tcp_speed)

    def _exec_route_blocked(self) -> bool:
        """In-motion A* re-check: is the route from the current TCP to the active
        segment goal blocked by the (speed-inflated) human obstacles? Uses a full
        fresh plan and resets the guard's incremental state so the next plan-time
        guard call re-plans cleanly."""
        if self._exec_goal_voxel is None:
            return False
        start_v = self.current_tcp_voxel()
        goal_v = self._exec_goal_voxel
        if start_v == goal_v:
            return False
        obstacles = self.active_human_voxels()
        obstacles.discard(start_v)
        obstacles.discard(goal_v)
        result = self.astar.plan_with_info(
            start_v, goal_v, obstacles, max_steps=self.ara_max_steps
        )
        # Re-check disturbs astar incremental state; force a fresh guard plan next.
        self._astar_last_goal = None
        self._first_plan_done = False
        return not result.path

    def _record_breadcrumb_if_due(self) -> None:
        """Sample current joint config + TCP into the breadcrumb cache, at most
        once per breadcrumb_record_period. Called from the execution monitor."""
        if not self.enable_breadcrumb:
            return
        now = time.time()
        if now - self._breadcrumb_last_record < self.breadcrumb_record_period:
            return
        self._breadcrumb_last_record = now
        try:
            values = self.group.get_current_joint_values()
            pose = self.group.get_current_pose(self.end_effector_link).pose
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Breadcrumb sample failed: %s", exc)
            return
        joints = {name: float(val) for name, val in zip(self.active_joints, values)}
        self.breadcrumb.record(
            joints,
            (pose.position.x, pose.position.y, pose.position.z),
        )

    def _try_breadcrumb_hop(self, target_joint_map: JointMap) -> bool:
        """When a waypoint is blocked, plan+execute a hop to the nearest cached
        pose that is still collision-free now, to warm-start toward the goal.
        All safety gates (ARA*, trajectory_is_safe, emergency stop) still run."""
        if not (self.enable_breadcrumb and self.enable_breadcrumb_fallback):
            return False
        goal_xyz = self._joint_map_goal_xyz(target_joint_map)
        if goal_xyz is None:
            return False
        obstacles = self.active_human_voxels()

        def _is_valid(tcp) -> bool:
            return self.world_to_voxel(tcp[0], tcp[1], tcp[2]) not in obstacles

        hop_joint_map = self.breadcrumb.nearest_valid(goal_xyz, _is_valid)
        if hop_joint_map is None:
            return False

        rospy.loginfo("Breadcrumb fallback: hopping to a cached free pose.")
        hop_plan = self.plan_to_joint_map(hop_joint_map)
        if hop_plan is None:
            self.plan_logger.log(
                "BREADCRUMB", "HOP_PLAN_FAILED",
                goal=goal_xyz,
                n_human=len(self.latest_human_points()),
                detail="cache hop target could not be planned",
            )
            return False
        ok = self.execute_plan(hop_plan)
        self.plan_logger.log(
            "BREADCRUMB", "HOP" if ok else "HOP_EXEC_FAILED",
            goal=goal_xyz,
            n_human=len(self.latest_human_points()),
            detail="cache size=%d" % self.breadcrumb.size(),
        )
        return ok

    def target_tcp_pose_for_joint_map(self, joint_map: JointMap) -> Optional[PoseStamped]:
        if self.fk_client is None:
            return None

        current_state = self.robot.get_current_state()
        joint_values = self.joint_map_to_group_order(joint_map)
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = list(current_state.joint_state.name)
        joint_state.position = list(current_state.joint_state.position)

        position_by_name = dict(zip(joint_state.name, joint_state.position))
        for joint_name, joint_value in zip(self.active_joints, joint_values):
            position_by_name[joint_name] = joint_value

        joint_state.position = [position_by_name[name] for name in joint_state.name]
        current_state.joint_state = joint_state

        request = GetPositionFKRequest()
        request.header.frame_id = self.target_frame
        request.fk_link_names = [self.end_effector_link]
        request.robot_state = current_state

        try:
            response = self.fk_client(request)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, f"compute_fk failed: {exc}")
            return None

        if response.error_code.val != response.error_code.SUCCESS or not response.pose_stamped:
            rospy.logwarn_throttle(
                2.0,
                f"compute_fk returned error code {response.error_code.val}",
            )
            return None

        return response.pose_stamped[0]

    def hand_distance_to_pose(self, pose: PoseStamped) -> Optional[float]:
        human_points = self.latest_human_points()
        if not human_points:
            return None

        target = pose.pose.position
        return min(
            math.sqrt(
                (target.x - human.x) ** 2
                + (target.y - human.y) ** 2
                + (target.z - human.z) ** 2
            )
            for human in human_points
        )

    def hand_blocks_target_pose(self, pose: PoseStamped) -> bool:
        distance = self.hand_distance_to_pose(pose)
        if distance is None:
            return False

        if distance < self.hand_waypoint_collision_radius:
            rospy.logwarn(
                f"Human is near next waypoint target: {distance:.3f} m "
                f"< {self.hand_waypoint_collision_radius:.3f} m."
            )
            self.status_pub.publish(
                f"HUMAN_BLOCKS_WAYPOINT distance={distance:.3f}"
            )
            return True

        return False

    def goal_state_in_collision(self, joint_map: JointMap) -> Optional[bool]:
        """True if the goal joint state is in collision per MoveIt's
        check_state_validity, False if valid, None if the check is unavailable.

        OMPL rejects an in-collision goal almost instantly, so without this the
        replan loop would keep calling plan() forever on a guaranteed-invalid
        target while the A* guard still reports a free TCP path."""
        if self.state_validity_client is None:
            return None

        robot_state = self.robot.get_current_state()
        joint_values = self.joint_map_to_group_order(joint_map)

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = list(robot_state.joint_state.name)
        joint_state.position = list(robot_state.joint_state.position)

        position_by_name = dict(zip(joint_state.name, joint_state.position))
        for joint_name, joint_value in zip(self.active_joints, joint_values):
            position_by_name[joint_name] = joint_value
        joint_state.position = [position_by_name[name] for name in joint_state.name]
        robot_state.joint_state = joint_state

        request = GetStateValidityRequest()
        request.robot_state = robot_state
        request.group_name = self.group_name

        try:
            response = self.state_validity_client(request)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, f"check_state_validity failed: {exc}")
            return None

        return not response.valid

    def waypoint_obstructed(
        self,
        joint_map: JointMap,
        target_pose: Optional[PoseStamped],
    ) -> bool:
        """Combined detour gate: hand near the waypoint OR goal state in
        collision. Agrees with what OMPL actually rejects (see B+C fix)."""
        hand_blocks = (
            target_pose is not None and self.hand_blocks_target_pose(target_pose)
        )
        return should_escalate_to_detour(
            hand_blocks, self.goal_state_in_collision(joint_map)
        )

    def make_upward_detour_pose(self, target_pose: PoseStamped) -> PoseStamped:
        detour = PoseStamped()
        detour.header.frame_id = target_pose.header.frame_id or self.target_frame
        detour.header.stamp = rospy.Time.now()
        detour.pose = copy.deepcopy(target_pose.pose)
        detour.pose.position.z += self.detour_z_offset
        return detour

    def fk_pose_for_joint_positions(
        self,
        joint_names: List[str],
        joint_positions: Tuple[float, ...],
    ) -> Optional[PoseStamped]:
        if self.fk_client is None:
            return None

        current_state = self.robot.get_current_state()
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = list(current_state.joint_state.name)
        joint_state.position = list(current_state.joint_state.position)

        position_by_name = dict(zip(joint_state.name, joint_state.position))
        for joint_name, joint_value in zip(joint_names, joint_positions):
            position_by_name[joint_name] = joint_value

        joint_state.position = [position_by_name[name] for name in joint_state.name]
        current_state.joint_state = joint_state

        request = GetPositionFKRequest()
        request.header.frame_id = self.target_frame
        request.fk_link_names = [self.end_effector_link]
        request.robot_state = current_state

        try:
            response = self.fk_client(request)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, f"compute_fk trajectory sample failed: {exc}")
            return None

        if response.error_code.val != response.error_code.SUCCESS or not response.pose_stamped:
            rospy.logwarn_throttle(
                2.0,
                f"compute_fk trajectory sample returned error code {response.error_code.val}",
            )
            return None

        return response.pose_stamped[0]

    def fk_poses_for_joint_positions(
        self,
        joint_names: List[str],
        joint_positions: Tuple[float, ...],
        link_names: List[str],
    ) -> List[PoseStamped]:
        if self.fk_client is None or not link_names:
            return []

        current_state = self.robot.get_current_state()
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = list(current_state.joint_state.name)
        joint_state.position = list(current_state.joint_state.position)

        position_by_name = dict(zip(joint_state.name, joint_state.position))
        for joint_name, joint_value in zip(joint_names, joint_positions):
            position_by_name[joint_name] = joint_value

        joint_state.position = [position_by_name[name] for name in joint_state.name]
        current_state.joint_state = joint_state
        return self.fk_poses_for_robot_state(current_state, link_names)

    def fk_poses_for_robot_state(self, robot_state, link_names: List[str]) -> List[PoseStamped]:
        if self.fk_client is None or not link_names:
            return []

        request = GetPositionFKRequest()
        request.header.frame_id = self.target_frame
        request.fk_link_names = link_names
        request.robot_state = robot_state

        try:
            response = self.fk_client(request)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, f"compute_fk safety links failed: {exc}")
            return []

        if response.error_code.val != response.error_code.SUCCESS:
            rospy.logwarn_throttle(
                2.0,
                f"compute_fk safety links returned error code {response.error_code.val}",
            )
            return []

        return list(response.pose_stamped)

    def latest_human_point(self) -> Optional[PointStamped]:
        if self.last_human_point is None:
            return None

        if self.last_human_point.header.stamp != rospy.Time(0):
            age = (rospy.Time.now() - self.last_human_point.header.stamp).to_sec()
            if age > self.human_timeout_sec:
                return None

        return self.last_human_point

    def base_clearance(self) -> float:
        return max(
            self.human_safety_distance,
            self.robot_body_radius + self.human_body_radius,
        )

    @staticmethod
    def point_distance(a: Point, b: Point) -> float:
        return math.sqrt(
            (a.x - b.x) ** 2
            + (a.y - b.y) ** 2
            + (a.z - b.z) ** 2
        )

    @staticmethod
    def point_to_segment_distance(point: Point, a: Point, b: Point) -> float:
        abx = b.x - a.x
        aby = b.y - a.y
        abz = b.z - a.z
        apx = point.x - a.x
        apy = point.y - a.y
        apz = point.z - a.z

        ab_len_sq = abx * abx + aby * aby + abz * abz
        if ab_len_sq <= 1e-12:
            return UR3FixedJointABNode.point_distance(point, a)

        t = (apx * abx + apy * aby + apz * abz) / ab_len_sq
        t = max(0.0, min(1.0, t))

        closest = Point()
        closest.x = a.x + t * abx
        closest.y = a.y + t * aby
        closest.z = a.z + t * abz
        return UR3FixedJointABNode.point_distance(point, closest)

    def min_distance_to_link_polyline(
        self,
        human_point: Point,
        link_poses: List[PoseStamped],
    ) -> Tuple[float, str]:
        min_distance = float("inf")
        closest_name = ""

        link_points = [pose.pose.position for pose in link_poses]
        for link_name, link_point in zip(self.safety_link_names, link_points):
            distance = self.point_distance(human_point, link_point)
            if distance < min_distance:
                min_distance = distance
                closest_name = link_name

        for index in range(len(link_points) - 1):
            distance = self.point_to_segment_distance(
                human_point,
                link_points[index],
                link_points[index + 1],
            )
            if distance < min_distance:
                min_distance = distance
                closest_name = "%s-%s" % (
                    self.safety_link_names[index],
                    self.safety_link_names[index + 1],
                )

        return min_distance, closest_name

    def filter_human_points_near_link_poses(
        self,
        human_points: List[Point],
        link_poses: List[PoseStamped],
        context: str,
        distance_threshold: float,
    ) -> List[Point]:
        if (
            not self.ignore_human_points_near_robot_path
            or not human_points
            or not link_poses
            or distance_threshold <= 0.0
        ):
            return human_points

        kept: List[Point] = []
        ignored = 0
        min_distance = float("inf")
        closest_part = ""

        for human_point in human_points:
            distance, robot_part = self.min_distance_to_link_polyline(
                human_point,
                link_poses,
            )
            if distance < min_distance:
                min_distance = distance
                closest_part = robot_part

            if distance <= distance_threshold:
                ignored += 1
                continue

            kept.append(human_point)

        if ignored > 0:
            self.status_pub.publish(
                "IGNORED_HUMAN_POINTS_NEAR_ROBOT context=%s ignored=%d kept=%d "
                "threshold=%.3f min_dist=%.3f part=%s"
                % (
                    context,
                    ignored,
                    len(kept),
                    distance_threshold,
                    min_distance,
                    closest_part,
                )
            )
            rospy.logwarn_throttle(
                1.0,
                "Ignored %d human point(s) near robot path/context=%s "
                "(threshold=%.3f m, min_dist=%.3f m, part=%s).",
                ignored,
                context,
                distance_threshold,
                min_distance,
                closest_part,
            )

        return kept

    def filter_human_points_near_plan_path(
        self,
        human_points: List[Point],
        plan,
        sample_indexes: List[int],
    ) -> List[Point]:
        if (
            not self.ignore_human_points_near_robot_path
            or not human_points
            or self.fk_client is None
        ):
            return human_points

        trajectory = plan.joint_trajectory
        kept: List[Point] = []
        ignored = 0
        min_distance = float("inf")
        closest_part = ""

        for human_point in human_points:
            point_min_distance = float("inf")
            point_closest_part = ""

            for index in sample_indexes:
                trajectory_point = trajectory.points[index]
                link_poses = self.fk_poses_for_joint_positions(
                    list(trajectory.joint_names),
                    tuple(trajectory_point.positions),
                    self.safety_link_names,
                )
                if not link_poses:
                    continue

                distance, robot_part = self.min_distance_to_link_polyline(
                    human_point,
                    link_poses,
                )
                if distance < point_min_distance:
                    point_min_distance = distance
                    point_closest_part = robot_part

            if point_min_distance < min_distance:
                min_distance = point_min_distance
                closest_part = point_closest_part

            if point_min_distance <= self.robot_path_false_positive_distance:
                ignored += 1
                continue

            kept.append(human_point)

        if ignored > 0:
            self.status_pub.publish(
                "IGNORED_HUMAN_POINTS_NEAR_PLANNED_PATH ignored=%d kept=%d "
                "threshold=%.3f min_dist=%.3f part=%s"
                % (
                    ignored,
                    len(kept),
                    self.robot_path_false_positive_distance,
                    min_distance,
                    closest_part,
                )
            )
            rospy.logwarn(
                "Ignored %d human point(s) because they overlap planned robot path "
                "(threshold=%.3f m, min_dist=%.3f m, part=%s).",
                ignored,
                self.robot_path_false_positive_distance,
                min_distance,
                closest_part,
            )

        return kept

    def trajectory_speed_margin(
        self,
        previous_poses: Optional[List[PoseStamped]],
        current_poses: List[PoseStamped],
        dt: float,
    ) -> float:
        if previous_poses is None or dt <= 1e-6:
            return 0.0

        max_speed = 0.0
        for previous, current in zip(previous_poses, current_poses):
            p0 = previous.pose.position
            p1 = current.pose.position
            distance = math.sqrt(
                (p1.x - p0.x) ** 2
                + (p1.y - p0.y) ** 2
                + (p1.z - p0.z) ** 2
            )
            max_speed = max(max_speed, distance / dt)

        return min(self.max_speed_safety_margin, max_speed * self.speed_safety_gain)

    def trajectory_is_safe(self, plan) -> bool:
        human_points = self.latest_human_points()
        if not human_points:
            self.status_pub.publish("NO_SELECTED_HUMAN_POINTS trajectory_check=skipped")
            return True

        self.status_pub.publish("TRAJECTORY_CHECK_SELECTED_POINTS count=%d" % len(human_points))

        if self.fk_client is None:
            rospy.logwarn_throttle(
                2.0,
                "Cannot validate planned trajectory against human points because FK is unavailable.",
            )
            return True

        trajectory = plan.joint_trajectory
        if not trajectory.points:
            return False

        sample_indexes = list(range(0, len(trajectory.points), self.trajectory_sample_step))
        if sample_indexes[-1] != len(trajectory.points) - 1:
            sample_indexes.append(len(trajectory.points) - 1)

        min_distance = float("inf")
        previous_poses: Optional[List[PoseStamped]] = None
        previous_time: Optional[float] = None
        base_clearance = self.base_clearance()

        for index in sample_indexes:
            point = trajectory.points[index]
            link_poses = self.fk_poses_for_joint_positions(
                list(trajectory.joint_names),
                tuple(point.positions),
                self.safety_link_names,
            )
            if not link_poses:
                continue

            current_time = point.time_from_start.to_sec()
            dt = 0.0 if previous_time is None else current_time - previous_time
            speed_margin = self.trajectory_speed_margin(
                previous_poses,
                link_poses,
                dt,
            )
            required_clearance = base_clearance + speed_margin

            for link_name, link_pose in zip(self.safety_link_names, link_poses):
                link = link_pose.pose.position
                for human in human_points:
                    distance = math.sqrt(
                        (link.x - human.x) ** 2
                        + (link.y - human.y) ** 2
                        + (link.z - human.z) ** 2
                    )
                    min_distance = min(min_distance, distance)

                    if distance < required_clearance:
                        self.status_pub.publish(
                            f"TRAJECTORY_TOO_CLOSE_TO_HUMAN link={link_name} "
                            f"distance={distance:.3f} required={required_clearance:.3f}"
                        )
                        rospy.logwarn(
                            f"Planned trajectory sample {index}, link {link_name}, "
                            f"is too close to human: {distance:.3f} m "
                            f"< {required_clearance:.3f} m "
                            f"(base={base_clearance:.3f}, speed_margin={speed_margin:.3f}). "
                            "Waiting and replanning."
                        )
                        self.plan_logger.log(
                            "TRAJECTORY", "TOO_CLOSE",
                            reason="link=%s dist=%.3f required=%.3f" % (
                                link_name, distance, required_clearance,
                            ),
                            start=(link.x, link.y, link.z),
                            human=(human.x, human.y, human.z),
                            n_human=len(human_points),
                            min_dist=distance,
                            detail="sample=%d base=%.3f speed_margin=%.3f" % (
                                index, base_clearance, speed_margin,
                            ),
                        )
                        return False

            previous_poses = link_poses
            previous_time = current_time

        rospy.loginfo(
            f"Planned trajectory minimum robot-link human distance: {min_distance:.3f} m"
        )
        self.status_pub.publish(
            "TRAJECTORY_SAFE selected=%d min_distance=%.3f"
            % (len(human_points), min_distance)
        )
        return True

    def current_robot_hand_min_distance(self) -> Optional[Tuple[float, str]]:
        human_points = self.latest_human_points()
        if not human_points:
            return None

        link_poses = self.fk_poses_for_robot_state(
            self.robot.get_current_state(),
            self.safety_link_names,
        )
        if not link_poses:
            return None

        min_distance = float("inf")
        min_link = ""
        for link_name, link_pose in zip(self.safety_link_names, link_poses):
            link = link_pose.pose.position
            for human in human_points:
                distance = math.sqrt(
                    (link.x - human.x) ** 2
                    + (link.y - human.y) ** 2
                    + (link.z - human.z) ** 2
                )
                if distance < min_distance:
                    min_distance = distance
                    min_link = link_name

        return min_distance, min_link

    def astar_path_is_available_to_pose(self, target_pose: PoseStamped) -> bool:
        if not self.enable_astar_guard:
            return True

        start_voxel = self.current_tcp_voxel()
        target = target_pose.pose.position
        goal_voxel = self.world_to_voxel(target.x, target.y, target.z)

        obstacles = self.active_human_voxels()
        obstacles.discard(start_voxel)
        obstacles.discard(goal_voxel)

        started = time.time()
        astar_result: PlanResult
        if not self._first_plan_done or goal_voxel != self._astar_last_goal:
            astar_result = self.astar.plan_with_info(
                start_voxel,
                goal_voxel,
                obstacles,
                max_steps=self.ara_max_steps,
            )
            self._first_plan_done = True
            self._astar_last_goal = goal_voxel
        else:
            astar_result = self.astar.replan_with_info(
                start_voxel,
                obstacles,
                max_steps=self.ara_max_steps,
            )
        path = astar_result.path
        elapsed_ms = (time.time() - started) * 1000.0
        self.planning_time_pub.publish(Float32(data=elapsed_ms))
        if self.publish_split_planning_time:
            self.astar_planning_time_pub.publish(Float32(data=elapsed_ms))
        self.store_plan_viz(start_voxel, goal_voxel, path)
        if self.obstacle_publish_rate <= 0.0:
            # No timer: publish markers inline (legacy plan-time-only behavior).
            self.publish_astar_markers(start_voxel, goal_voxel, path, obstacles)

        if not path:
            reason = astar_result.reason
            self.astar_path_pub.publish("NO_ASTAR_PATH reason=%s" % reason)
            self.status_pub.publish("ASTAR_PATH_BLOCKED reason=%s obstacles=%d" % (reason, len(obstacles)))
            rospy.logwarn(
                f"No ARA* path from {start_voxel} to {goal_voxel}; "
                "waiting for obstacle to move or workspace params to be adjusted."
            )
            self.plan_logger.log(
                "ASTAR", "BLOCKED",
                reason="no_path:%s" % reason,
                start=self.voxel_to_world(start_voxel),
                goal=(target.x, target.y, target.z),
                n_obstacles=len(obstacles),
                n_human=len(self.latest_human_points()),
                human=self._nearest_human_xyz(target.x, target.y, target.z),
                path_len=0,
                planning_ms=elapsed_ms,
                detail="ARA* blocked by obstacle",
            )
            return False

        self.astar_path_pub.publish(self.voxel_to_text(path))
        self.status_pub.publish(
            "ASTAR_OK path=%d obstacles=%d expanded=%s epsilon=%s time_ms=%.2f"
            % (
                len(path),
                len(obstacles),
                astar_result.metrics.get("expanded_steps", 0),
                astar_result.metrics.get("epsilon_satisfied", "n/a"),
                elapsed_ms,
            )
        )
        rospy.loginfo(
            "ARA* path available: start=%s goal=%s obstacles=%d path=%d planning_time=%.2fms",
            start_voxel,
            goal_voxel,
            len(obstacles),
            len(path),
            elapsed_ms,
        )
        self.plan_logger.log(
            "ASTAR", "OK",
            start=self.voxel_to_world(start_voxel),
            goal=(target.x, target.y, target.z),
            n_obstacles=len(obstacles),
            n_human=len(self.latest_human_points()),
            path_len=len(path),
            planning_ms=elapsed_ms,
        )
        return True

    def astar_path_is_available(self, joint_map: JointMap) -> bool:
        target_pose = self.target_tcp_pose_for_joint_map(joint_map)
        if target_pose is None:
            return True
        return self.astar_path_is_available_to_pose(target_pose)

    def retime_plan(self, plan):
        try:
            return self.group.retime_trajectory(
                self.robot.get_current_state(),
                plan,
                velocity_scaling_factor=self.velocity_scale,
                acceleration_scaling_factor=self.acceleration_scale,
            )
        except Exception as exc:
            rospy.logwarn(f"retime_trajectory failed, using original plan: {exc}")
            return plan

    def plan_to_joint_map(self, joint_map: JointMap):
        if not self.can_plan_to_waypoint():
            return None

        if not self.astar_path_is_available(joint_map):
            return None

        joint_values = self.joint_map_to_group_order(joint_map)

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(joint_values)

        started = time.time()
        result = self.group.plan()
        elapsed_ms = (time.time() - started) * 1000.0
        self.planning_time_pub.publish(Float32(data=elapsed_ms))
        if self.publish_split_planning_time:
            self.moveit_planning_time_pub.publish(Float32(data=elapsed_ms))
        success, plan = self.extract_plan_result(result)

        if not success or plan is None:
            rospy.logwarn("Joint target planning failed.")
            self.status_pub.publish("JOINT_PLAN_FAILED")
            self.plan_logger.log(
                "MOVEIT", "FAILED",
                reason="moveit_no_plan",
                goal=self._joint_map_goal_xyz(joint_map),
                n_human=len(self.latest_human_points()),
                planning_ms=elapsed_ms,
                detail="OMPL found no plan (A* path existed)",
            )
            return None

        if not self.trajectory_is_safe(plan):
            self.plan_logger.log(
                "PLAN", "NOT_EXECUTED",
                reason="trajectory_unsafe",
                goal=self._joint_map_goal_xyz(joint_map),
                n_human=len(self.latest_human_points()),
                planning_ms=elapsed_ms,
                detail="planned ok but trajectory too close to human",
            )
            return None

        self.status_pub.publish("MOVEIT_PLAN_OK time=%.2f" % elapsed_ms)
        rospy.loginfo("Joint target planning success in %.2f ms.", elapsed_ms)
        return self.retime_plan(plan)

    def plan_to_pose(
        self,
        pose_goal: PoseStamped,
        planning_time: Optional[float] = None,
    ):
        if not self.can_plan_to_waypoint():
            return None

        if not self.astar_path_is_available_to_pose(pose_goal):
            return None

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose_goal, self.end_effector_link)

        requested_planning_time = self.planning_time if planning_time is None else planning_time
        self.group.set_planning_time(requested_planning_time)
        started = time.time()
        try:
            result = self.group.plan()
        finally:
            self.group.set_planning_time(self.planning_time)
        elapsed_ms = (time.time() - started) * 1000.0
        self.planning_time_pub.publish(Float32(data=elapsed_ms))
        if self.publish_split_planning_time:
            self.moveit_planning_time_pub.publish(Float32(data=elapsed_ms))
        success, plan = self.extract_plan_result(result)

        if not success or plan is None:
            rospy.logwarn("Pose target planning failed.")
            self.status_pub.publish("POSE_PLAN_FAILED")
            self.group.clear_pose_targets()
            return None

        if not self.trajectory_is_safe(plan):
            self.group.clear_pose_targets()
            return None

        self.group.clear_pose_targets()
        self.status_pub.publish("MOVEIT_PLAN_OK time=%.2f" % elapsed_ms)
        rospy.loginfo("Pose target planning success in %.2f ms.", elapsed_ms)
        return self.retime_plan(plan)

    def plan_and_execute_detour_with_retry(self, pose_goal: PoseStamped, label: str) -> bool:
        for attempt in range(1, self.max_detour_attempts + 1):
            if rospy.is_shutdown():
                return False

            plan = self.plan_to_pose(
                pose_goal,
                planning_time=self.detour_planning_time,
            )
            if plan is not None:
                return self.execute_plan(plan)

            if attempt >= self.max_detour_attempts:
                break

            rospy.logwarn(
                f"Replanning {label}, attempt {attempt + 1}/{self.max_detour_attempts}..."
            )
            rospy.sleep(self.detour_retry_period)

        rospy.logwarn(
            f"Failed to plan {label} after {self.max_detour_attempts} attempts."
        )
        return False

    def hold_until_waypoint_clear(
        self,
        joint_map: JointMap,
        target_pose: PoseStamped,
        index: int,
    ) -> bool:
        self.status_pub.publish(
            f"DETOUR_HOLD waypoint={index} attempts={self.max_detour_attempts}"
        )
        rospy.logwarn(
            f"Detour attempts exhausted at waypoint {index}. "
            "Holding until waypoint is clear."
        )

        while not rospy.is_shutdown():
            if not self.waypoint_obstructed(joint_map, target_pose):
                self.status_pub.publish(f"DETOUR_HOLD_CLEAR waypoint={index}")
                rospy.loginfo(f"Waypoint {index} clear. Resuming normal waypoint planning.")
                return True
            rospy.sleep(self.detour_hold_poll_period)

        return False

    def run_detour_if_hand_blocks_waypoint(self, joint_map: JointMap, index: int) -> bool:
        target_pose = self.target_tcp_pose_for_joint_map(joint_map)
        if target_pose is None:
            return True

        rospy.sleep(self.check_pause_sec)
        if not self.waypoint_obstructed(joint_map, target_pose):
            return True

        detour_pose = self.make_upward_detour_pose(target_pose)
        self.status_pub.publish(
            f"RUN_UPWARD_DETOUR waypoint={index} dz={self.detour_z_offset:.3f}"
        )
        rospy.logwarn(
            f"Running upward detour before waypoint {index}: "
            f"+Z {self.detour_z_offset:.3f} m."
        )
        if self.plan_and_execute_detour_with_retry(
            detour_pose,
            f"UPWARD_DETOUR_{index}",
        ):
            return True

        return self.hold_until_waypoint_clear(joint_map, target_pose, index)

    def execute_plan(self, plan, goal_voxel: Optional[Voxel] = None) -> bool:
        if not self.auto_execute:
            rospy.loginfo("auto_execute is False, only planning.")
            self.status_pub.publish("PLAN_ONLY_SUCCESS")
            self.plan_logger.log(
                "PLAN", "NOT_EXECUTED",
                reason="auto_execute_false",
                n_human=len(self.latest_human_points()),
                detail="plan ok, execution disabled by config",
            )
            return True

        # New segment motion starts: live speed governs again and any reroute
        # latch from the previous segment has already been consumed by the plan
        # that produced this trajectory, so clear it.
        self._exec_goal_voxel = goal_voxel
        self._reroute_pad_voxels = 0
        self._speed_pad_voxels_live = 0
        self._current_tcp_speed = 0.0
        self._prev_tcp_pos = None
        self._prev_tcp_time = None
        self._last_exec_recheck = 0.0

        started = time.time()
        ok = self.group.execute(plan, wait=False)
        if not ok:
            self.execution_time_pub.publish(Float32(data=(time.time() - started) * 1000.0))
            self.group.stop()
            self.group.clear_pose_targets()
            self.status_pub.publish("EXECUTE_FAILED")
            self.plan_logger.log(
                "EXECUTE", "FAILED",
                reason="moveit_execute_returned_false",
                n_human=len(self.latest_human_points()),
            )
            self._speed_pad_voxels_live = 0
            self._exec_goal_voxel = None
            return False

        trajectory = plan.joint_trajectory
        duration = 0.0
        if trajectory.points:
            duration = trajectory.points[-1].time_from_start.to_sec()

        deadline = rospy.Time.now() + rospy.Duration(duration + 1.0)
        rate = rospy.Rate(self.execution_monitor_rate)
        min_distance = float("inf")

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            now_t = time.time()
            self._update_tcp_speed(now_t)

            distance_result = self.current_robot_hand_min_distance()
            if distance_result is not None:
                distance, link_name = distance_result
                min_distance = min(min_distance, distance)
                required_clearance = self.base_clearance() + self.execution_speed_safety_margin
                if distance < required_clearance:
                    rospy.logwarn(
                        f"Emergency stop: {link_name}-human distance {distance:.3f} m "
                        f"< {required_clearance:.3f} m during execution."
                    )
                    self.status_pub.publish(
                        f"EXECUTION_STOPPED_HUMAN_TOO_CLOSE link={link_name} "
                        f"distance={distance:.3f} required={required_clearance:.3f}"
                    )
                    self.execution_time_pub.publish(Float32(data=(time.time() - started) * 1000.0))
                    self.group.stop()
                    self.group.clear_pose_targets()
                    self.plan_logger.log(
                        "EXECUTE", "STOPPED_HUMAN_TOO_CLOSE",
                        reason="link=%s dist=%.3f required=%.3f" % (
                            link_name, distance, required_clearance,
                        ),
                        n_human=len(self.latest_human_points()),
                        min_dist=distance,
                        detail="emergency stop during execution",
                    )
                    self._speed_pad_voxels_live = 0
                    self._exec_goal_voxel = None
                    return False

            # Tier 1: in-motion A* re-check. Only when speed padding is active and
            # no reroute is pending yet. Flags a reroute for the NEXT leg's plan;
            # does not stop (option A: swap at next waypoint). Tier 2 above stays
            # the in-segment safety net while the robot finishes this short hop.
            if (
                self._speed_pad_voxels_live > 0
                and self._reroute_pad_voxels == 0
                and self._exec_goal_voxel is not None
                and self.astar_recheck_period_sec >= 0.0
                and (now_t - self._last_exec_recheck) >= self.astar_recheck_period_sec
            ):
                self._last_exec_recheck = now_t
                if self._exec_route_blocked():
                    self._reroute_pad_voxels = self._speed_pad_voxels_live
                    self.status_pub.publish(
                        "ASTAR_EXEC_REPLAN pad_voxels=%d speed=%.3f goal=%s"
                        % (
                            self._reroute_pad_voxels,
                            self._current_tcp_speed,
                            self._exec_goal_voxel,
                        )
                    )
                    rospy.logwarn(
                        "In-motion A* re-check: route to %s blocked at %.3f m/s; "
                        "latching pad=%d voxels, will reroute at next waypoint.",
                        self._exec_goal_voxel,
                        self._current_tcp_speed,
                        self._reroute_pad_voxels,
                    )

            self._record_breadcrumb_if_due()
            rate.sleep()

        self.group.stop()
        self.group.clear_pose_targets()
        # Segment finished: live padding resets (robot stopping); any reroute
        # latch is kept so the next waypoint's plan sees the inflated obstacles.
        self._speed_pad_voxels_live = 0
        self._current_tcp_speed = 0.0
        self._exec_goal_voxel = None

        elapsed_ms = (time.time() - started) * 1000.0
        self.execution_time_pub.publish(Float32(data=elapsed_ms))
        rospy.loginfo(
            f"Execute monitor done. Minimum robot-human distance: {min_distance:.3f} m. "
            f"Execution time: {elapsed_ms:.2f} ms"
        )

        self.status_pub.publish("EXECUTE_SUCCESS")
        self.plan_logger.log(
            "EXECUTE", "SUCCESS",
            n_human=len(self.latest_human_points()),
            min_dist=None if min_distance == float("inf") else min_distance,
            planning_ms=elapsed_ms,
        )

        return True

    def run_waypoint(self, joint_map: JointMap, index: int, total: int, direction: str) -> bool:
        rospy.loginfo(f"{direction}, waypoint {index}/{total}")

        for joint_name in self.active_joints:
            rospy.loginfo(f"  {joint_name}: {joint_map[joint_name]:.6f} rad")

        if not self.run_detour_if_hand_blocks_waypoint(joint_map, index):
            return False

        attempt = 0
        plan = None
        breadcrumb_hop_tried = False

        while not rospy.is_shutdown():
            attempt += 1
            plan = self.plan_to_joint_map(joint_map)
            if plan is not None:
                break

            # First block: try one warm-start hop via the breadcrumb cache.
            if not breadcrumb_hop_tried:
                breadcrumb_hop_tried = True
                if self._try_breadcrumb_hop(joint_map):
                    continue  # retry the real target from the cached free pose

            # B+C: if the goal joint state is itself in collision, OMPL will keep
            # failing in ~tens of ms forever. Escalate to detour/hold (which only
            # releases once the goal is valid again) instead of busy-retrying a
            # guaranteed-invalid target. None (service unavailable) -> old path.
            if should_escalate_to_detour(False, self.goal_state_in_collision(joint_map)):
                rospy.logwarn_throttle(
                    2.0,
                    f"Goal state for waypoint {index}/{total} is in collision; "
                    "escalating to detour/hold instead of replanning in place.",
                )
                self.plan_logger.log(
                    "DETOUR", "ESCALATE",
                    reason="goal_state_in_collision",
                    goal=self._joint_map_goal_xyz(joint_map),
                    n_human=len(self.latest_human_points()),
                    detail="goal joint state in collision; OMPL would loop",
                )
                if not self.run_detour_if_hand_blocks_waypoint(joint_map, index):
                    return False
                breadcrumb_hop_tried = False  # allow a fresh hop after clearing
                continue

            if self.max_replan_attempts > 0 and attempt >= self.max_replan_attempts:
                rospy.logwarn(
                    f"Failed to find a valid plan for waypoint {index}/{total} "
                    f"after {attempt} attempts."
                )
                return False

            rospy.logwarn(
                f"Replanning waypoint {index}/{total}, attempt {attempt + 1}..."
            )
            rospy.sleep(self.replan_retry_period)

        # Segment goal voxel for the in-motion A* re-check (None -> re-check off).
        goal_voxel = None
        target_pose = self.target_tcp_pose_for_joint_map(joint_map)
        if target_pose is not None:
            tp = target_pose.pose.position
            goal_voxel = self.world_to_voxel(tp.x, tp.y, tp.z)

        ok = self.execute_plan(plan, goal_voxel=goal_voxel)
        if not ok:
            return False

        if self.wait_after_waypoint > 0.0:
            rospy.sleep(self.wait_after_waypoint)
        return True

    def run_path(self, path: List[JointMap], direction: str) -> bool:
        self._first_plan_done = False
        self._astar_last_goal = None
        total = len(path)

        for idx, joint_map in enumerate(path, start=1):
            ok = self.run_waypoint(joint_map, idx, total, direction)
            if not ok:
                rospy.logwarn(f"Stopped at {direction}, waypoint {idx}/{total}")
                return False

            if rospy.is_shutdown():
                return False

        return True

    def spin(self) -> None:
        cycle_index = 0

        while not rospy.is_shutdown():
            cycle_index += 1

            rospy.loginfo(f"Starting cycle {cycle_index}")
            self.status_pub.publish(f"CYCLE_{cycle_index}_START")

            rospy.loginfo("Running A to B")
            ok_forward = self.run_path(self.path_a_to_b, "A_TO_B")
            if not ok_forward:
                self.status_pub.publish("A_TO_B_FAILED_RETRYING")
                rospy.logwarn("A_TO_B failed. Waiting for obstacle to clear, then retrying from A.")
                rospy.sleep(2.0)
                continue

            rospy.loginfo("Running B to A")
            path_b_to_a = list(reversed(self.path_a_to_b[:-1]))
            ok_backward = self.run_path(path_b_to_a, "B_TO_A")
            if not ok_backward:
                self.status_pub.publish("B_TO_A_FAILED_RETRYING")
                rospy.logwarn("B_TO_A failed. Waiting for obstacle to clear, then retrying from A.")
                rospy.sleep(2.0)
                continue

            self.status_pub.publish(f"CYCLE_{cycle_index}_DONE")
            rospy.loginfo(f"Cycle {cycle_index} done")

            if not self.loop and cycle_index >= self.cycles:
                break

        rospy.loginfo("ur3_fixed_joint_ab_node finished.")


def _selftest() -> None:
    """Assert-based check for the pure speed->padding math. Run with --selftest
    (no ROS needed): python3 planner_ab_replan_node.py --selftest"""
    f = speed_padding_meters
    assert f(0.0, 0.05, 1.0, 0.05) == 0.0            # below deadband -> 0
    assert f(0.04, 0.05, 1.0, 0.05) == 0.0           # below deadband -> 0
    assert abs(f(0.10, 0.05, 0.5, 0.05) - 0.025) < 1e-9  # linear region
    assert f(0.20, 0.05, 1.0, 0.05) == 0.05          # capped at max
    assert f(5.0, 0.05, 1.0, 0.05) == 0.05           # cap holds at high speed
    assert f(5.0, 0.05, 0.0, 0.05) == 0.0            # gain 0 -> disabled
    assert f(5.0, 0.05, 1.0, 0.0) == 0.0             # max 0 -> disabled
    print("planner speed_padding_meters self-check OK")

    g = should_escalate_to_detour
    assert g(True, None) is True       # hand blocks -> escalate regardless
    assert g(True, False) is True      # hand blocks even if goal valid
    assert g(False, True) is True      # goal in collision -> escalate
    assert g(False, False) is False    # clear + valid goal -> no escalate
    assert g(False, None) is False     # validity unknown -> preserve old path
    print("planner should_escalate_to_detour self-check OK")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
        sys.exit(0)
    try:
        node = UR3FixedJointABNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
