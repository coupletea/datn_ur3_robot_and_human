#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import sys
import threading
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from data_skeleton import pose_array_to_numeric_joint_dict


Point3D = Tuple[float, float, float]
Skeleton = Dict[int, Point3D]

INVALID_COORD = float("nan")
DEFAULT_TRACKED_JOINT_IDS = [
    0, 11, 12, 13, 14, 15, 16, 23, 24,
]
BODY_FUSION_JOINT_IDS = [0, 11, 12, 13, 14, 15, 16, 23, 24]
BODY_CORE_FUSION_JOINT_IDS = [0, 11, 12, 23, 24]
ARM_FUSION_JOINT_IDS = [13, 14, 15, 16]


def _int_list_param(name: str, default: Iterable[int]) -> List[int]:
    value = rospy.get_param(name, list(default))
    if isinstance(value, str):
        return [int(item.strip()) for item in value.split(",") if item.strip()]
    return [int(item) for item in value]


def _bool_param(name: str, default: bool) -> bool:
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on")
    return bool(value)


def _is_finite_point(point: Sequence[float]) -> bool:
    return len(point) == 3 and all(math.isfinite(float(value)) for value in point)


def _point_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


def _weighted_average_point(
    p_front: Sequence[float],
    p_back: Sequence[float],
    w_front: float,
    w_back: float,
) -> Point3D:
    total = float(w_front) + float(w_back)
    if total <= 1e-9 or not math.isfinite(total):
        w_front = 0.5
        w_back = 0.5
        total = 1.0
    return (
        (float(p_front[0]) * w_front + float(p_back[0]) * w_back) / total,
        (float(p_front[1]) * w_front + float(p_back[1]) * w_back) / total,
        (float(p_front[2]) * w_front + float(p_back[2]) * w_back) / total,
    )


def _np_to_point(point: np.ndarray) -> Point3D:
    return (float(point[0]), float(point[1]), float(point[2]))


class MultiKinectSkeletonController:
    MODE_FUSION_OK = "FUSION_OK"
    MODE_FALLBACK_FRONT = "FALLBACK_FRONT"
    MODE_FALLBACK_BACK = "FALLBACK_BACK"
    MODE_NO_INPUT = "NO_INPUT"

    def __init__(self) -> None:
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.front_skeleton_base_topic = rospy.get_param(
            "~front_skeleton_base_topic", "/kinect_front/human_skeleton_base"
        )
        self.back_skeleton_base_topic = rospy.get_param(
            "~back_skeleton_base_topic", "/kinect_back/human_skeleton_base"
        )
        self.output_skeleton_base_topic = rospy.get_param(
            "~output_skeleton_base_topic", "/human_skeleton_base"
        )
        self.fusion_status_topic = rospy.get_param(
            "~fusion_status_topic", "/human_skeleton_fusion_status"
        )
        self.fusion_debug_marker_topic = rospy.get_param(
            "~fusion_debug_marker_topic", "/human_skeleton_fusion_markers"
        )

        self.tracked_joint_ids = _int_list_param("~tracked_joint_ids", DEFAULT_TRACKED_JOINT_IDS)
        self.fusion_rate_hz = float(rospy.get_param("~fusion_rate_hz", 30.0))
        self.max_input_age_sec = float(rospy.get_param("~max_input_age_sec", 0.35))
        self.sync_max_stamp_diff_sec = float(rospy.get_param("~sync_max_stamp_diff_sec", 0.25))
        self.allow_single_camera_fallback = _bool_param("~allow_single_camera_fallback", True)
        self.empty_publish_on_no_input = _bool_param("~empty_publish_on_no_input", True)

        self.body_core_joint_ids = set(_int_list_param("~body_core_joint_ids", BODY_CORE_FUSION_JOINT_IDS))
        self.body_joint_ids = set(_int_list_param("~body_joint_ids", [0, 11, 12, 13, 14, 15, 16, 23, 24]))
        self.arm_joint_ids = set(_int_list_param("~arm_joint_ids", [13, 14, 15, 16]))
        self.fusion_joint_ids = [
            int(joint_id)
            for joint_id in self.tracked_joint_ids
            if int(joint_id) in set(BODY_FUSION_JOINT_IDS)
        ]
        if not self.fusion_joint_ids:
            self.fusion_joint_ids = list(BODY_FUSION_JOINT_IDS)
        self.left_hand_joint_ids = set(_int_list_param("~left_hand_joint_ids", range(100, 121)))
        self.right_hand_joint_ids = set(_int_list_param("~right_hand_joint_ids", range(200, 221)))
        self.hand_joint_ids = set(
            _int_list_param("~hand_joint_ids", list(range(100, 121)) + list(range(200, 221)))
        )

        self.body_merge_distance_m = float(rospy.get_param("~body_merge_distance_m", 0.15))
        self.arm_merge_distance_m = float(rospy.get_param("~arm_merge_distance_m", 0.12))
        self.hand_merge_distance_m = float(rospy.get_param("~hand_merge_distance_m", 0.08))
        self.default_merge_distance_m = float(rospy.get_param("~default_merge_distance_m", 0.12))
        self.body_conflict_distance_m = float(rospy.get_param("~body_conflict_distance_m", 0.20))
        self.arm_conflict_distance_m = float(rospy.get_param("~arm_conflict_distance_m", 0.16))
        self.hand_conflict_distance_m = float(rospy.get_param("~hand_conflict_distance_m", 0.10))
        self.max_conflict_ratio = float(rospy.get_param("~max_conflict_ratio", 0.35))
        self.agree_thresh = {
            "body_core": float(rospy.get_param("~agree_thresh_body_core_m", 0.035)),
            "arm": float(rospy.get_param("~agree_thresh_arm_m", 0.040)),
        }
        self.soft_thresh_multiplier = float(rospy.get_param("~soft_thresh_multiplier", 3.0))

        self.body_valid_weight = float(rospy.get_param("~body_valid_weight", 2.0))
        self.arm_valid_weight = float(rospy.get_param("~arm_valid_weight", 1.5))
        self.hand_valid_weight = float(rospy.get_param("~hand_valid_weight", 1.0))
        self.freshness_penalty_weight = float(rospy.get_param("~freshness_penalty_weight", 5.0))
        self.jump_penalty_weight = float(rospy.get_param("~jump_penalty_weight", 2.0))
        self.conflict_penalty_weight = float(rospy.get_param("~conflict_penalty_weight", 1.0))
        self.stale_score_penalty = float(rospy.get_param("~stale_score_penalty", 100.0))
        self.score_tie_epsilon = float(rospy.get_param("~score_tie_epsilon", 0.5))
        self.prefer_camera_when_tie = rospy.get_param("~prefer_camera_when_tie", "front")
        self.prefer_newer_frame_when_tie = _bool_param("~prefer_newer_frame_when_tie", True)

        self.enable_side_swap_detection = _bool_param("~enable_side_swap_detection", True)
        self.side_swap_left_joint_ids = _int_list_param("~side_swap_left_joint_ids", [11, 13, 15])
        self.side_swap_right_joint_ids = _int_list_param("~side_swap_right_joint_ids", [12, 14, 16])
        self.side_swap_margin_m = float(rospy.get_param("~side_swap_margin_m", 0.05))
        self.side_swap_min_pairs = int(rospy.get_param("~side_swap_min_pairs", 3))
        self.apply_side_swap_to_hands = _bool_param("~apply_side_swap_to_hands", True)

        self.enable_geometry_validation = _bool_param("~enable_geometry_validation", True)
        self.min_body_core_points = int(rospy.get_param("~min_body_core_points", 3))
        self.shoulder_width_min_m = float(rospy.get_param("~shoulder_width_min_m", 0.20))
        self.shoulder_width_max_m = float(rospy.get_param("~shoulder_width_max_m", 0.75))
        self.torso_length_min_m = float(rospy.get_param("~torso_length_min_m", 0.25))
        self.torso_length_max_m = float(rospy.get_param("~torso_length_max_m", 1.00))
        self.max_arm_bone_length_m = float(rospy.get_param("~max_arm_bone_length_m", 0.75))

        self.enable_fusion_temporal_filter = _bool_param("~enable_fusion_temporal_filter", True)
        self.fusion_smoothing_alpha = float(rospy.get_param("~fusion_smoothing_alpha", 0.45))
        self.fusion_max_jump_m = float(rospy.get_param("~fusion_max_jump_m", 0.25))
        self.fusion_lost_frames = int(rospy.get_param("~fusion_lost_frames", 3))
        self.hold_last_valid_joint_sec = float(rospy.get_param("~hold_last_valid_joint_sec", 0.15))
        self.kf_Q_pos = float(rospy.get_param("~kf_process_noise_pos", 0.01))
        self.kf_Q_vel = float(rospy.get_param("~kf_process_noise_vel", 0.10))
        self.kf_R_body_core = float(rospy.get_param("~kf_meas_noise_body_core", 0.015))
        self.kf_R_arm = float(rospy.get_param("~kf_meas_noise_arm", 0.025))
        self.kf_mahal_body_core = float(rospy.get_param("~kf_mahal_thresh_body_core", 9.0))
        self.kf_mahal_arm = float(rospy.get_param("~kf_mahal_thresh_arm", 14.0))
        self.kf_cold_start_n = int(rospy.get_param("~kf_cold_start_frames", 3))
        self.min_alive_joints = int(rospy.get_param("~min_alive_joints", 4))
        self.freeze_max_frames = int(rospy.get_param("~freeze_max_frames", 10))

        self.min_valid_joints_to_publish = int(rospy.get_param("~min_valid_joints_to_publish", 5))
        self.min_body_joints_to_publish = int(rospy.get_param("~min_body_joints_to_publish", 3))
        self.min_arm_or_hand_joints_to_publish = int(rospy.get_param("~min_arm_or_hand_joints_to_publish", 2))
        self.publish_empty_when_invalid = _bool_param("~publish_empty_when_invalid", True)

        self.status_interval_sec = float(rospy.get_param("~status_interval_sec", 1.0))
        self.publish_conflict_detail = _bool_param("~publish_conflict_detail", True)
        self.max_conflict_log_per_frame = int(rospy.get_param("~max_conflict_log_per_frame", 5))
        self.publish_source_markers = _bool_param("~publish_source_markers", False)
        self.debug_print_scores = _bool_param("~debug_print_scores", False)

        self._lock = threading.Lock()
        self.lock = self._lock
        self.front_msg: Optional[PoseArray] = None
        self.back_msg: Optional[PoseArray] = None
        self.front_msg_time: Optional[rospy.Time] = None
        self.back_msg_time: Optional[rospy.Time] = None
        self.prev_front_skeleton: Skeleton = {}
        self.prev_back_skeleton: Skeleton = {}
        self.prev_front_stamp: Optional[rospy.Time] = None
        self.prev_back_stamp: Optional[rospy.Time] = None
        self.prev_fused_skeleton: Skeleton = {}
        self.prev_fused_stamp: Optional[rospy.Time] = None
        self.prev_joint_seen_time: Dict[int, rospy.Time] = {}
        self.prev_missing_counts: Dict[int, int] = {}
        self.kf_state: Dict[int, np.ndarray] = {}
        self.kf_P: Dict[int, np.ndarray] = {}
        self.kf_stamp: Dict[int, rospy.Time] = {}
        self.kf_initialized: Dict[int, bool] = {}
        self.kf_cold_counts: Dict[int, int] = {}
        self._prev_pos_front: Dict[int, np.ndarray] = {}
        self._prev_pos_back: Dict[int, np.ndarray] = {}
        self._freeze_count_front = 0
        self._freeze_count_back = 0
        self.last_status_text = ""
        self.last_status_time = rospy.Time(0)
        self.last_source_map: Dict[int, str] = {}
        self._front_dirty = False
        self._back_dirty = False
        self._process_lock = threading.Lock()
        self._processing = False
        self._back_swapped = False
        self._swap_hysteresis_margin = float(rospy.get_param("~swap_hysteresis_margin", 0.32))
        self._fusion_mode = self.MODE_NO_INPUT
        self._last_published_mode: Optional[str] = None

        self.output_pub = rospy.Publisher(self.output_skeleton_base_topic, PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher(self.fusion_status_topic, String, queue_size=10, latch=True)
        self.pub_fusion_mode = rospy.Publisher(
            "/human_skeleton_fusion_mode",
            String,
            queue_size=1,
            latch=True,
        )
        self.marker_pub = None
        if self.publish_source_markers:
            self.marker_pub = rospy.Publisher(self.fusion_debug_marker_topic, MarkerArray, queue_size=1)

        rospy.Subscriber(self.front_skeleton_base_topic, PoseArray, self.front_callback, queue_size=1)
        rospy.Subscriber(self.back_skeleton_base_topic, PoseArray, self.back_callback, queue_size=1)
        rospy.loginfo(
            "multi_kinect_skeleton_controller front=%s back=%s output=%s joints=%d",
            self.front_skeleton_base_topic,
            self.back_skeleton_base_topic,
            self.output_skeleton_base_topic,
            len(self.tracked_joint_ids),
        )

    def decode_pose_array(self, msg: PoseArray) -> Skeleton:
        decoded = pose_array_to_numeric_joint_dict(msg, self.tracked_joint_ids)
        return {
            int(joint_id): decoded[int(joint_id)]
            for joint_id in self.tracked_joint_ids
            if int(joint_id) in decoded
        }

    def encode_pose_array(self, skeleton: Skeleton, stamp) -> PoseArray:
        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = stamp
        for joint_id in self.tracked_joint_ids:
            pose_msg = Pose()
            pose_msg.orientation.w = 1.0
            if joint_id in skeleton and _is_finite_point(skeleton[joint_id]):
                pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = skeleton[joint_id]
            else:
                pose_msg.position.x = INVALID_COORD
                pose_msg.position.y = INVALID_COORD
                pose_msg.position.z = INVALID_COORD
            pose_array.poses.append(pose_msg)
        return pose_array

    def sanitize_skeleton(self, skeleton: Skeleton) -> Skeleton:
        return {int(joint_id): point for joint_id, point in skeleton.items() if _is_finite_point(point)}

    def count_valid_groups(self, skeleton: Skeleton) -> Dict[str, int]:
        return {
            "body_core": sum(1 for joint_id in self.body_core_joint_ids if joint_id in skeleton),
            "body": sum(1 for joint_id in self.body_joint_ids if joint_id in skeleton),
            "arm": sum(1 for joint_id in self.arm_joint_ids if joint_id in skeleton),
            "left_hand": sum(1 for joint_id in self.left_hand_joint_ids if joint_id in skeleton),
            "right_hand": sum(1 for joint_id in self.right_hand_joint_ids if joint_id in skeleton),
            "hand": sum(1 for joint_id in self.hand_joint_ids if joint_id in skeleton),
            "total": len(skeleton),
        }

    def is_input_fresh(self, stamp, now) -> Tuple[bool, float]:
        if stamp is None or stamp == rospy.Time(0):
            return False, float("inf")
        age_sec = max(0.0, (now - stamp).to_sec())
        return age_sec <= self.max_input_age_sec, age_sec

    def is_camera_alive(self, source: str, skeleton: Skeleton, stamp, now) -> Tuple[bool, str, float]:
        fresh, age_sec = self.is_input_fresh(stamp, now)
        if not fresh:
            return False, "stale age=%.3fs" % age_sec, age_sec
        if len(skeleton) < self.min_alive_joints:
            return False, "too_few_valid_joints=%d" % len(skeleton), age_sec

        prev_key = "_prev_pos_%s" % source
        count_key = "_freeze_count_%s" % source
        prev = getattr(self, prev_key, {})
        common = set(prev.keys()).intersection(skeleton.keys())
        if len(common) >= 3:
            max_delta = max(
                float(np.linalg.norm(np.asarray(skeleton[joint_id], dtype=float) - prev[joint_id]))
                for joint_id in common
            )
            if max_delta < 1e-4:
                freeze_count = int(getattr(self, count_key, 0)) + 1
                setattr(self, count_key, freeze_count)
                if freeze_count > self.freeze_max_frames:
                    return False, "freeze_detected count=%d" % freeze_count, age_sec
            else:
                setattr(self, count_key, 0)

        setattr(
            self,
            prev_key,
            {joint_id: np.asarray(point, dtype=float) for joint_id, point in skeleton.items()},
        )
        return True, "ok", age_sec

    def validate_geometry(self, skeleton: Skeleton) -> Tuple[bool, str]:
        if not self.enable_geometry_validation:
            return True, "GEOMETRY_DISABLED"
        counts = self.count_valid_groups(skeleton)
        if counts["body_core"] < self.min_body_core_points:
            return False, "GEOMETRY_FEW_CORE count=%d" % counts["body_core"]
        if 11 in skeleton and 12 in skeleton:
            shoulder_width = _point_distance(skeleton[11], skeleton[12])
            if shoulder_width < self.shoulder_width_min_m or shoulder_width > self.shoulder_width_max_m:
                return False, "GEOMETRY_BAD_SHOULDER width=%.3f" % shoulder_width
        if all(joint_id in skeleton for joint_id in (11, 12, 23, 24)):
            shoulder_center = tuple((skeleton[11][axis] + skeleton[12][axis]) / 2.0 for axis in range(3))
            hip_center = tuple((skeleton[23][axis] + skeleton[24][axis]) / 2.0 for axis in range(3))
            torso_length = _point_distance(shoulder_center, hip_center)
            if torso_length < self.torso_length_min_m or torso_length > self.torso_length_max_m:
                return False, "GEOMETRY_BAD_TORSO length=%.3f" % torso_length
        for a, b in ((11, 13), (13, 15), (12, 14), (14, 16)):
            if a in skeleton and b in skeleton and _point_distance(skeleton[a], skeleton[b]) > self.max_arm_bone_length_m:
                return False, "GEOMETRY_BAD_ARM %d-%d" % (a, b)
        return True, "GEOMETRY_OK"

    def compute_jump_count(self, source_name: str, skeleton: Skeleton, stamp) -> int:
        previous = self.prev_front_skeleton if source_name == "front" else self.prev_back_skeleton
        jump_count = 0
        for joint_id, point in skeleton.items():
            if joint_id in previous and _point_distance(point, previous[joint_id]) > self.fusion_max_jump_m:
                jump_count += 1
        if source_name == "front":
            self.prev_front_skeleton = dict(skeleton)
            self.prev_front_stamp = stamp
        else:
            self.prev_back_skeleton = dict(skeleton)
            self.prev_back_stamp = stamp
        return jump_count

    def compute_conflict_map(self, front_skeleton: Skeleton, back_skeleton: Skeleton) -> Dict[int, Dict[str, float]]:
        conflicts: Dict[int, Dict[str, float]] = {}
        for joint_id in set(self.fusion_joint_ids).intersection(front_skeleton).intersection(back_skeleton):
            distance = _point_distance(front_skeleton[joint_id], back_skeleton[joint_id])
            threshold = self.conflict_distance_for_joint(joint_id)
            if distance > threshold:
                conflicts[joint_id] = {
                    "distance": distance,
                    "threshold": threshold,
                    "type": self.joint_type(joint_id),
                }
        return conflicts

    def compute_quality_score(
        self,
        source_name: str,
        skeleton: Skeleton,
        age_sec: float,
        jump_count: int,
        conflict_count: int,
        geometry_ok: bool,
    ) -> float:
        counts = self.count_valid_groups(skeleton)
        score = (
            counts["body"] * self.body_valid_weight
            + counts["arm"] * self.arm_valid_weight
            + counts["hand"] * self.hand_valid_weight
            - age_sec * self.freshness_penalty_weight
            - jump_count * self.jump_penalty_weight
            - conflict_count * self.conflict_penalty_weight
        )
        if not geometry_ok:
            score -= self.stale_score_penalty * 0.5
        if not math.isfinite(age_sec):
            score -= self.stale_score_penalty
        if self.debug_print_scores:
            rospy.loginfo_throttle(1.0, "fusion_score %s score=%.2f counts=%s", source_name, score, counts)
        return score

    def detect_side_swap(self, reference_skeleton: Skeleton, candidate_skeleton: Skeleton) -> Tuple[bool, float, float, int]:
        direct_error = 0.0
        swap_error = 0.0
        pair_count = 0
        for left_id, right_id in zip(self.side_swap_left_joint_ids, self.side_swap_right_joint_ids):
            if left_id in reference_skeleton and right_id in reference_skeleton:
                if left_id in candidate_skeleton and right_id in candidate_skeleton:
                    direct_error += _point_distance(reference_skeleton[left_id], candidate_skeleton[left_id])
                    direct_error += _point_distance(reference_skeleton[right_id], candidate_skeleton[right_id])
                    swap_error += _point_distance(reference_skeleton[left_id], candidate_skeleton[right_id])
                    swap_error += _point_distance(reference_skeleton[right_id], candidate_skeleton[left_id])
                    pair_count += 1
        if pair_count > 0:
            direct_error /= float(pair_count * 2)
            swap_error /= float(pair_count * 2)
        should_swap = (
            pair_count >= self.side_swap_min_pairs
            and swap_error + self.side_swap_margin_m < direct_error
        )
        return should_swap, direct_error, swap_error, pair_count

    def swap_left_right_skeleton(self, skeleton: Skeleton) -> Skeleton:
        swapped = dict(skeleton)
        pairs = [(11, 12), (13, 14), (15, 16), (23, 24)]
        if self.apply_side_swap_to_hands:
            pairs.extend((100 + index, 200 + index) for index in range(21))
        for left_id, right_id in pairs:
            left = swapped.pop(left_id, None)
            right = swapped.pop(right_id, None)
            if left is not None:
                swapped[right_id] = left
            if right is not None:
                swapped[left_id] = right
        return swapped

    def joint_type(self, joint_id: int) -> str:
        if joint_id in self.body_core_joint_ids:
            return "body_core"
        if joint_id in self.arm_joint_ids:
            return "arm"
        if joint_id in self.body_joint_ids:
            return "body"
        if joint_id in self.left_hand_joint_ids:
            return "left_hand"
        if joint_id in self.right_hand_joint_ids:
            return "right_hand"
        if joint_id in self.hand_joint_ids:
            return "hand"
        return "unknown"

    def merge_distance_for_joint(self, joint_id: int) -> float:
        kind = self.joint_type(joint_id)
        if kind in ("left_hand", "right_hand", "hand"):
            return self.hand_merge_distance_m
        if kind == "arm":
            return self.arm_merge_distance_m
        if kind in ("body", "body_core"):
            return self.body_merge_distance_m
        return self.default_merge_distance_m

    def conflict_distance_for_joint(self, joint_id: int) -> float:
        kind = self.joint_type(joint_id)
        if kind in ("left_hand", "right_hand", "hand"):
            return self.hand_conflict_distance_m
        agreement_type = "arm" if kind == "arm" else "body_core"
        if agreement_type in self.agree_thresh:
            return self.agree_thresh[agreement_type] * self.soft_thresh_multiplier
        if kind == "arm":
            return self.arm_conflict_distance_m
        if kind in ("body", "body_core"):
            return self.body_conflict_distance_m
        return self.default_merge_distance_m

    def select_point_for_conflict(
        self,
        joint_id: int,
        p_front: Point3D,
        p_back: Point3D,
        front_score: float,
        back_score: float,
        front_counts: Dict[str, int],
        back_counts: Dict[str, int],
        front_age: float,
        back_age: float,
    ) -> Tuple[Point3D, str, str]:
        kind = self.joint_type(joint_id)
        if kind in ("left_hand", "right_hand", "hand"):
            if front_counts["hand"] != back_counts["hand"]:
                source = "front" if front_counts["hand"] > back_counts["hand"] else "back"
                return (p_front if source == "front" else p_back), source, "conflict_hand_count"
        elif kind == "arm":
            if front_counts["arm"] != back_counts["arm"]:
                source = "front" if front_counts["arm"] > back_counts["arm"] else "back"
                return (p_front if source == "front" else p_back), source, "conflict_arm_count"
        elif front_counts["body"] != back_counts["body"]:
            source = "front" if front_counts["body"] > back_counts["body"] else "back"
            return (p_front if source == "front" else p_back), source, "conflict_body_count"

        if abs(front_score - back_score) > self.score_tie_epsilon:
            source = "front" if front_score > back_score else "back"
            return (p_front if source == "front" else p_back), source, "conflict_score"
        if self.prefer_newer_frame_when_tie and abs(front_age - back_age) > 1e-6:
            source = "front" if front_age < back_age else "back"
            return (p_front if source == "front" else p_back), source, "conflict_newer"
        source = "front" if self.prefer_camera_when_tie == "front" else "back"
        return (p_front if source == "front" else p_back), source, "conflict_tie"

    def fuse_joint(
        self,
        joint_id: int,
        front_skeleton: Skeleton,
        back_skeleton: Skeleton,
        front_score: float,
        back_score: float,
    ) -> Tuple[Optional[Point3D], str, str]:
        has_front = joint_id in front_skeleton
        has_back = joint_id in back_skeleton
        if has_front and not has_back:
            return front_skeleton[joint_id], "front", "front_only_joint"
        if has_back and not has_front:
            return back_skeleton[joint_id], "back", "back_only_joint"
        if not has_front and not has_back:
            return None, "missing", "missing_joint"

        p_front = front_skeleton[joint_id]
        p_back = back_skeleton[joint_id]
        distance = _point_distance(p_front, p_back)
        joint_kind = self.joint_type(joint_id)
        agreement_type = "arm" if joint_kind == "arm" else "body_core"
        agree_threshold = self.agree_thresh.get(agreement_type, self.agree_thresh["body_core"])
        soft_threshold = agree_threshold * self.soft_thresh_multiplier
        w_front = max(float(front_score), 0.01)
        w_back = max(float(back_score), 0.01)

        if distance <= agree_threshold:
            return (
                _weighted_average_point(p_front, p_back, w_front, w_back),
                "blend_agree",
                "agree distance=%.3f threshold=%.3f" % (distance, agree_threshold),
            )
        if distance <= soft_threshold:
            return (
                _weighted_average_point(p_front, p_back, w_front ** 2, w_back ** 2),
                "blend_soft",
                "soft_conflict distance=%.3f threshold=%.3f" % (distance, soft_threshold),
            )
        if front_score >= back_score:
            return p_front, "front_hard", "hard_conflict distance=%.3f" % distance
        return p_back, "back_hard", "hard_conflict distance=%.3f" % distance

    def fuse_skeletons(
        self,
        front_skeleton: Skeleton,
        back_skeleton: Skeleton,
        front_score: float,
        back_score: float,
    ) -> Tuple[Skeleton, Dict[int, str], Dict[int, str]]:
        fused: Skeleton = {}
        source_map: Dict[int, str] = {}
        fusion_reasons: Dict[int, str] = {}
        for joint_id in self.fusion_joint_ids:
            point, source, reason = self.fuse_joint(
                joint_id,
                front_skeleton,
                back_skeleton,
                front_score,
                back_score,
            )
            if point is not None and _is_finite_point(point):
                fused[joint_id] = point
                source_map[joint_id] = source
                fusion_reasons[joint_id] = reason
        return fused, source_map, fusion_reasons

    def _kf_predict(self, joint_id: int, stamp) -> None:
        if joint_id not in self.kf_state:
            return
        dt = max(0.001, (stamp - self.kf_stamp[joint_id]).to_sec())
        dt = float(np.clip(dt, 0.001, 0.5))
        transition = np.eye(6)
        transition[0, 3] = dt
        transition[1, 4] = dt
        transition[2, 5] = dt
        process_noise = np.diag([self.kf_Q_pos] * 3 + [self.kf_Q_vel] * 3) * dt
        self.kf_state[joint_id] = transition @ self.kf_state[joint_id]
        self.kf_P[joint_id] = transition @ self.kf_P[joint_id] @ transition.T + process_noise
        self.kf_stamp[joint_id] = stamp

    def _kf_update(self, joint_id: int, measurement: np.ndarray, stamp) -> Tuple[np.ndarray, bool]:
        joint_kind = self.joint_type(joint_id)
        measurement_type = "arm" if joint_kind == "arm" else "body_core"
        if joint_id not in self.kf_state:
            self.kf_state[joint_id] = np.array(
                [measurement[0], measurement[1], measurement[2], 0.0, 0.0, 0.0],
                dtype=float,
            )
            self.kf_P[joint_id] = np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0])
            self.kf_stamp[joint_id] = stamp
            self.kf_initialized[joint_id] = False
            self.kf_cold_counts[joint_id] = 1
            if self.kf_cold_counts[joint_id] >= self.kf_cold_start_n:
                self.kf_initialized[joint_id] = True
            return measurement, True

        self._kf_predict(joint_id, stamp)
        state = self.kf_state[joint_id]
        covariance = self.kf_P[joint_id]
        observation = np.zeros((3, 6))
        observation[0, 0] = 1.0
        observation[1, 1] = 1.0
        observation[2, 2] = 1.0
        measurement_noise = self.kf_R_arm if measurement_type == "arm" else self.kf_R_body_core
        R = np.eye(3) * (measurement_noise ** 2)
        innovation = measurement - observation @ state
        innovation_covariance = observation @ covariance @ observation.T + R

        try:
            innovation_covariance_inv = np.linalg.inv(innovation_covariance)
        except np.linalg.LinAlgError:
            innovation_covariance_inv = np.linalg.pinv(innovation_covariance)

        mahal_sq = float(innovation @ innovation_covariance_inv @ innovation)
        mahal_threshold = self.kf_mahal_arm if measurement_type == "arm" else self.kf_mahal_body_core
        if mahal_sq > mahal_threshold:
            return self.kf_state[joint_id][:3].copy(), False

        kalman_gain = covariance @ observation.T @ innovation_covariance_inv
        self.kf_state[joint_id] = state + kalman_gain @ innovation
        self.kf_P[joint_id] = (np.eye(6) - kalman_gain @ observation) @ covariance
        self.kf_stamp[joint_id] = stamp
        self.kf_cold_counts[joint_id] = self.kf_cold_counts.get(joint_id, 0) + 1
        if self.kf_cold_counts[joint_id] >= self.kf_cold_start_n:
            self.kf_initialized[joint_id] = True
        return self.kf_state[joint_id][:3].copy(), True

    def apply_fusion_kalman_filter(self, fused_skeleton: Skeleton, stamp) -> Tuple[Skeleton, bool, str]:
        if not self.enable_fusion_temporal_filter:
            return dict(fused_skeleton), True, "kf_disabled"

        filtered: Skeleton = {}
        cold_joints: List[int] = []
        rejected_joints: List[int] = []
        for joint_id in self.fusion_joint_ids:
            point = fused_skeleton.get(joint_id)
            if point is not None and _is_finite_point(point):
                smoothed, accepted = self._kf_update(joint_id, np.asarray(point, dtype=float), stamp)
                self.prev_missing_counts[joint_id] = 0
                self.prev_joint_seen_time[joint_id] = stamp
                if not accepted:
                    rejected_joints.append(joint_id)
                if not self.kf_initialized.get(joint_id, False):
                    cold_joints.append(joint_id)
                    continue
                filtered[joint_id] = _np_to_point(smoothed)
            elif self.kf_initialized.get(joint_id, False) and joint_id in self.kf_state:
                missing = self.prev_missing_counts.get(joint_id, 0) + 1
                self.prev_missing_counts[joint_id] = missing
                last_seen = self.prev_joint_seen_time.get(joint_id, stamp)
                hold_age = max(0.0, (stamp - last_seen).to_sec())
                if missing <= self.fusion_lost_frames and hold_age <= self.hold_last_valid_joint_sec:
                    self._kf_predict(joint_id, stamp)
                    filtered[joint_id] = _np_to_point(self.kf_state[joint_id][:3])

        self.prev_fused_skeleton = dict(filtered)
        self.prev_fused_stamp = stamp
        ready = not cold_joints
        reason = "ready"
        if cold_joints:
            reason = "kf_cold_start joints=%s" % ",".join(str(joint_id) for joint_id in cold_joints)
        elif rejected_joints:
            reason = "kf_rejected joints=%s" % ",".join(str(joint_id) for joint_id in rejected_joints)
        return filtered, ready, reason

    def is_output_valid(self, fused_skeleton: Skeleton) -> Tuple[bool, str]:
        counts = self.count_valid_groups(fused_skeleton)
        arm_hand = counts["arm"] + counts["hand"]
        if counts["total"] >= self.min_valid_joints_to_publish:
            return True, "valid_total=%d" % counts["total"]
        if counts["body"] >= self.min_body_joints_to_publish:
            return True, "valid_body=%d" % counts["body"]
        if arm_hand >= self.min_arm_or_hand_joints_to_publish:
            return True, "valid_arm_hand=%d" % arm_hand
        return False, "too_few total=%d body=%d arm_hand=%d" % (counts["total"], counts["body"], arm_hand)

    def publish_empty_skeleton(self, stamp, reason: str) -> None:
        self.output_pub.publish(self.encode_pose_array({}, stamp))
        self.publish_status(reason)

    def publish_status(self, text: str, force: bool = False) -> None:
        now = rospy.Time.now()
        age = (now - self.last_status_time).to_sec() if self.last_status_time != rospy.Time(0) else float("inf")
        if force or text != self.last_status_text or age >= self.status_interval_sec:
            self.status_pub.publish(text)
            self.last_status_text = text
            self.last_status_time = now

    def _publish_fusion_mode(self, mode: str) -> None:
        if mode == self._last_published_mode:
            return

        try:
            self.pub_fusion_mode.publish(String(data=mode))
            self._last_published_mode = mode
            if mode == self.MODE_FUSION_OK:
                rospy.loginfo("[fusion] Mode: FUSION_OK, both cameras active")
            elif mode == self.MODE_FALLBACK_FRONT:
                rospy.logwarn("[fusion] Mode: FALLBACK_FRONT, front camera only")
            elif mode == self.MODE_FALLBACK_BACK:
                rospy.logwarn("[fusion] Mode: FALLBACK_BACK, back camera only")
            elif mode == self.MODE_NO_INPUT:
                rospy.logwarn("[fusion] Mode: NO_INPUT, no active camera")
        except Exception as exc:
            rospy.logdebug("[fusion] _publish_fusion_mode error: %s", exc)

    def _reset_kf(self, reason: str = "manual") -> None:
        self.kf_state.clear()
        self.kf_P.clear()
        self.kf_stamp.clear()
        self.kf_initialized.clear()
        self.kf_cold_counts.clear()
        self.prev_fused_skeleton.clear()
        self.prev_joint_seen_time.clear()
        self.prev_missing_counts.clear()
        rospy.logwarn("[fusion] KF reset: %s", reason)

    def _set_fusion_mode(self, mode: str) -> None:
        previous = self._fusion_mode
        if mode == self.MODE_NO_INPUT and previous != self.MODE_NO_INPUT:
            self._reset_kf("%s->%s" % (previous, mode))
        elif previous == self.MODE_NO_INPUT and mode in (
            self.MODE_FALLBACK_FRONT,
            self.MODE_FALLBACK_BACK,
            self.MODE_FUSION_OK,
        ):
            self._reset_kf("%s->%s" % (previous, mode))
        elif previous in (self.MODE_FALLBACK_FRONT, self.MODE_FALLBACK_BACK) and mode == self.MODE_FUSION_OK:
            self._reset_kf("%s->%s" % (previous, mode))
        self._fusion_mode = mode
        self._publish_fusion_mode(mode)

    def front_callback(self, msg: PoseArray) -> None:
        with self._lock:
            self.front_msg = msg
            self.front_msg_time = rospy.Time.now()
            self._front_dirty = True
        self._try_process()

    def back_callback(self, msg: PoseArray) -> None:
        with self._lock:
            self.back_msg = msg
            self.back_msg_time = rospy.Time.now()
            self._back_dirty = True
        self._try_process()

    def _msg_stamp(self, msg: Optional[PoseArray], receive_time: Optional[rospy.Time]) -> Optional[rospy.Time]:
        if msg is None:
            return None
        if msg.header.stamp != rospy.Time(0):
            return msg.header.stamp
        return receive_time

    def _try_process(self) -> None:
        with self._lock:
            has_new_data = self._front_dirty or self._back_dirty

        if not has_new_data:
            return

        if not self._process_lock.acquire(blocking=False):
            return

        self._processing = True
        try:
            with self._lock:
                self._front_dirty = False
                self._back_dirty = False
            self.process_once()
        finally:
            self._processing = False
            self._process_lock.release()

    def process_once(self, event=None) -> None:
        now = rospy.Time.now()
        with self._lock:
            front_msg = self.front_msg
            back_msg = self.back_msg
            front_receive_time = self.front_msg_time
            back_receive_time = self.back_msg_time

        front_stamp = self._msg_stamp(front_msg, front_receive_time)
        back_stamp = self._msg_stamp(back_msg, back_receive_time)
        front_skeleton_raw = self.sanitize_skeleton(self.decode_pose_array(front_msg)) if front_msg else {}
        back_skeleton_raw = self.sanitize_skeleton(self.decode_pose_array(back_msg)) if back_msg else {}
        front_skeleton = {
            joint_id: point for joint_id, point in front_skeleton_raw.items() if joint_id in self.fusion_joint_ids
        }
        back_skeleton = {
            joint_id: point for joint_id, point in back_skeleton_raw.items() if joint_id in self.fusion_joint_ids
        }

        front_alive, front_alive_reason, front_age = self.is_camera_alive("front", front_skeleton, front_stamp, now)
        back_alive, back_alive_reason, back_age = self.is_camera_alive("back", back_skeleton, back_stamp, now)

        if front_alive and back_alive and front_stamp and back_stamp:
            stamp_diff = abs((front_stamp - back_stamp).to_sec())
            if stamp_diff > self.sync_max_stamp_diff_sec:
                if front_age <= back_age:
                    back_skeleton = {}
                    back_alive = False
                    back_alive_reason = "desynced stamp_diff=%.3fs" % stamp_diff
                else:
                    front_skeleton = {}
                    front_alive = False
                    front_alive_reason = "desynced stamp_diff=%.3fs" % stamp_diff

        if front_alive and back_alive:
            new_mode = self.MODE_FUSION_OK
        elif front_alive:
            new_mode = self.MODE_FALLBACK_FRONT
        elif back_alive:
            new_mode = self.MODE_FALLBACK_BACK
        else:
            new_mode = self.MODE_NO_INPUT
        self._set_fusion_mode(new_mode)
        if not front_alive:
            front_skeleton = {}
        if not back_alive:
            back_skeleton = {}

        if not self.allow_single_camera_fallback:
            if not (front_alive and back_alive):
                if self.empty_publish_on_no_input:
                    self.publish_empty_skeleton(
                        now,
                        "FUSION_NO_VALID_INPUT require_both=true front=%s back=%s"
                        % (front_alive_reason, back_alive_reason),
                    )
                return

        if not front_alive and not back_alive:
            self._back_swapped = False
            rospy.logdebug("[fusion] Side swap state reset, no input")
            if self.empty_publish_on_no_input:
                self.publish_empty_skeleton(
                    now,
                    "FUSION_NO_VALID_INPUT front=%s back=%s"
                    % (front_alive_reason, back_alive_reason),
                )
            return

        if self.enable_side_swap_detection and front_alive and back_alive:
            should_swap, direct_error, swap_error, pair_count = self.detect_side_swap(front_skeleton, back_skeleton)
            if pair_count >= self.side_swap_min_pairs:
                margin = max(0.0, min(1.0, self._swap_hysteresis_margin))
                if not self._back_swapped:
                    if swap_error < direct_error * (1.0 - margin):
                        self._back_swapped = True
                        rospy.logdebug(
                            "[fusion] Side swap: ACTIVATE (direct=%.3f swap=%.3f margin=%.0f%%)",
                            direct_error,
                            swap_error,
                            margin * 100.0,
                        )
                elif direct_error < swap_error * (1.0 - margin):
                    self._back_swapped = False
                    rospy.logdebug(
                        "[fusion] Side swap: DEACTIVATE (direct=%.3f swap=%.3f margin=%.0f%%)",
                        direct_error,
                        swap_error,
                        margin * 100.0,
                    )

            if self._back_swapped:
                back_skeleton = self.swap_left_right_skeleton(back_skeleton)
                if should_swap:
                    self.publish_status(
                        "FUSION_SIDE_SWAP_DETECTED camera=back direct=%.3f swap=%.3f pairs=%d"
                        % (direct_error, swap_error, pair_count),
                        force=True,
                    )

        conflict_map = self.compute_conflict_map(front_skeleton, back_skeleton)
        front_conflicts = len(conflict_map) if front_alive and back_alive else 0
        back_conflicts = len(conflict_map) if front_alive and back_alive else 0
        front_geometry_ok, front_geometry_reason = self.validate_geometry(front_skeleton) if front_alive else (False, "NO_FRONT")
        back_geometry_ok, back_geometry_reason = self.validate_geometry(back_skeleton) if back_alive else (False, "NO_BACK")
        front_score = self.compute_quality_score("front", front_skeleton, front_age, 0, front_conflicts, front_geometry_ok)
        back_score = self.compute_quality_score("back", back_skeleton, back_age, 0, back_conflicts, back_geometry_ok)
        front_counts = self.count_valid_groups(front_skeleton)
        back_counts = self.count_valid_groups(back_skeleton)

        if front_alive and back_alive:
            fused, source_map, fusion_reasons = self.fuse_skeletons(
                front_skeleton,
                back_skeleton,
                front_score,
                back_score,
            )
        elif front_alive:
            fused = dict(front_skeleton)
            source_map = {joint_id: "front" for joint_id in fused}
            fusion_reasons = {joint_id: "front_fallback" for joint_id in fused}
        else:
            fused = dict(back_skeleton)
            source_map = {joint_id: "back" for joint_id in fused}
            fusion_reasons = {joint_id: "back_fallback" for joint_id in fused}

        active_stamps = []
        if front_alive and front_stamp is not None:
            active_stamps.append(front_stamp)
        if back_alive and back_stamp is not None:
            active_stamps.append(back_stamp)
        stamp = max(active_stamps) if active_stamps else now
        fused, kf_ready, kf_reason = self.apply_fusion_kalman_filter(fused, stamp)
        if not kf_ready:
            if self.publish_empty_when_invalid:
                self.publish_empty_skeleton(stamp, "FUSION_KF_NOT_READY %s" % kf_reason)
            return

        valid, valid_reason = self.is_output_valid(fused)
        if not valid:
            if self.publish_empty_when_invalid:
                self.publish_empty_skeleton(stamp, "FUSION_TOO_FEW_JOINTS %s" % valid_reason)
            return

        self.output_pub.publish(self.encode_pose_array(fused, stamp))
        self.last_source_map = source_map
        if self.publish_source_markers and self.marker_pub is not None:
            self.marker_pub.publish(self.build_marker_array(front_skeleton, back_skeleton, fused, stamp))

        if front_alive and back_alive:
            status_prefix = "FUSION_OK"
            if conflict_map:
                status_prefix = "FUSION_CONFLICT"
        elif front_alive:
            status_prefix = "FUSION_FRONT_ONLY"
        else:
            status_prefix = "FUSION_BACK_ONLY"

        detail = ""
        if conflict_map and self.publish_conflict_detail:
            shown = list(conflict_map.items())[: self.max_conflict_log_per_frame]
            detail = " conflicts=" + ";".join(
                "%s:%.3f>%0.3f" % (joint_id, item["distance"], item["threshold"])
                for joint_id, item in shown
            )
        conflict_ratio = float(len(conflict_map)) / float(max(1, len(set(front_skeleton).intersection(back_skeleton))))
        self.publish_status(
            "%s fused=%d front=%d back=%d front_score=%.2f back_score=%.2f conflict_ratio=%.2f %s %s %s kf=%s health_front=%s health_back=%s%s"
            % (
                status_prefix,
                len(fused),
                front_counts["total"],
                back_counts["total"],
                front_score,
                back_score,
                conflict_ratio,
                front_geometry_reason,
                back_geometry_reason,
                valid_reason,
                kf_reason,
                front_alive_reason,
                back_alive_reason,
                detail,
            )
        )

    def build_marker_array(self, front_skeleton: Skeleton, back_skeleton: Skeleton, fused: Skeleton, stamp) -> MarkerArray:
        marker_array = MarkerArray()
        configs = [
            ("front", front_skeleton, (0.0, 0.4, 1.0, 0.55), 0),
            ("back", back_skeleton, (1.0, 0.4, 0.0, 0.55), 1000),
            ("fused", fused, (0.0, 1.0, 0.2, 0.9), 2000),
        ]
        for ns, skeleton, color, offset in configs:
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = stamp
            marker.ns = ns
            marker.id = offset
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.035
            marker.scale.y = 0.035
            marker.scale.z = 0.035
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in skeleton.values()]
            marker.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(marker)
        return marker_array

    def spin(self) -> None:
        rospy.loginfo(
            "Event-driven fusion enabled; fusion_rate_hz=%.2f is kept for launch compatibility.",
            self.fusion_rate_hz,
        )
        rospy.spin()


def main() -> None:
    rospy.init_node("multi_kinect_skeleton_controller")
    controller = MultiKinectSkeletonController()
    controller.spin()


if __name__ == "__main__":
    main()
