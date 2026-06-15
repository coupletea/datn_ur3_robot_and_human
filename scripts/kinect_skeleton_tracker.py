#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import queue
import statistics
import sys
import threading
import time
from collections import defaultdict, deque
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401 - required by tf2 Buffer.transform(PointStamped)
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseArray, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from tf.transformations import quaternion_from_euler

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from data_skeleton import (
    BODY_LANDMARK_ORDER,
    LANDMARK_ORDER,
    KinectFrameTimeout,
    attach_hands_to_pose_wrists,
    dilate_mask,
    draw_filtered_skeleton,
    extract_hand_landmarks_3d,
    extract_pose_landmarks_3d,
    initialize_camera,
    is_body_landmark,
    is_hand_landmark,
    load_mediapipe_holistic,
    load_yolo_segment_model,
    merge_pose_and_hands,
    read_color_frame_only,
    read_registered_frames,
    run_holistic,
    run_yolo_person_segmentation,
    select_best_person_mask,
    skeleton_dict_to_pose_array,
    validate_body_geometry,
    validate_hand_geometry,
)


Point3D = Tuple[float, float, float]
SkeletonDict = Dict[str, Point3D]
SkeletonPixels = Dict[str, Tuple[int, int]]
ARM_BONES = [
    ("pose_11", "pose_13"),
    ("pose_13", "pose_15"),
    ("pose_12", "pose_14"),
    ("pose_14", "pose_16"),
]


def clamp_rate(value, rate_min: float, rate_max: float) -> float:
    """Clamp a requested loop rate (Hz) into [rate_min, rate_max]."""
    return max(float(rate_min), min(float(rate_max), float(value)))


def _int_list_param(name: str, default: Iterable[int]) -> List[int]:
    value = rospy.get_param(name, list(default))
    if isinstance(value, str):
        return [int(item.strip()) for item in value.split(",") if item.strip()]
    return [int(item) for item in value]


def _float_tuple_param(name: str, default: Sequence[float]) -> Tuple[float, ...]:
    value = rospy.get_param(name, list(default))
    if isinstance(value, str):
        return tuple(float(item.strip()) for item in value.split(",") if item.strip())
    return tuple(float(item) for item in value)


def _float_list_param(name: str, default: Iterable[float]) -> List[float]:
    value = rospy.get_param(name, list(default))
    if isinstance(value, str):
        return [float(item.strip()) for item in value.split(",") if item.strip()]
    return [float(item) for item in value]


def _string_list_param(name: str, default: Iterable[str]) -> List[str]:
    value = rospy.get_param(name, list(default))
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item).strip() for item in value if str(item).strip()]


def _bool_param(name: str, default: bool) -> bool:
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("true", "1", "yes", "on"):
            return True
        if text in ("false", "0", "no", "off", ""):
            return False
    return bool(value)


def _is_finite_point(point: Sequence[float]) -> bool:
    return len(point) == 3 and all(math.isfinite(float(value)) for value in point)


def _point_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


def _flip_image(image, flip_horizontal: bool = False, flip_vertical: bool = False):
    if flip_horizontal and flip_vertical:
        return cv2.flip(image, -1)
    if flip_horizontal:
        return cv2.flip(image, 1)
    if flip_vertical:
        return cv2.flip(image, 0)
    return image


def publish_static_transform(
    parent_frame: str,
    child_frame: str,
    translation: List[float],
    rotation_rpy: List[float],
) -> tf2_ros.StaticTransformBroadcaster:
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]

    quat = quaternion_from_euler(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]
    broadcaster.sendTransform(transform)
    return broadcaster


class SpatialArmFilter:
    """Filter skeleton in camera frame before TF transformation."""

    def __init__(
        self,
        enabled: bool = True,
        median_window: int = 2,
        max_jump_m: float = 0.5,
        enable_arm_length_lock: bool = True,
        arm_calib_frames: int = 50,
        enable_hip_clamp: bool = False,
        hip_clamp_offset_m: float = 0.30,
        landmark_order: Iterable[str] = LANDMARK_ORDER,
    ) -> None:
        self.enabled = bool(enabled)
        self.median_window = max(1, int(median_window))
        self.max_jump_m = max(0.0, float(max_jump_m))
        self.enable_arm_length_lock = bool(enable_arm_length_lock)
        self.arm_calib_frames = max(1, int(arm_calib_frames))
        self.enable_hip_clamp = bool(enable_hip_clamp)
        self.hip_clamp_offset_m = float(hip_clamp_offset_m)
        self.landmark_order = list(landmark_order)
        self.reset()

    def reset(self) -> None:
        self.joint_history: Dict[str, Deque[Point3D]] = defaultdict(
            lambda: deque(maxlen=self.median_window)
        )
        self.bone_samples: Dict[Tuple[str, str], List[float]] = defaultdict(list)
        self.bone_lengths: Dict[Tuple[str, str], float] = {}
        self.calibration_frames = 0
        self.calibrated = False

    @staticmethod
    def _median_point(history: Sequence[Point3D]) -> Point3D:
        return (
            float(statistics.median(point[0] for point in history)),
            float(statistics.median(point[1] for point in history)),
            float(statistics.median(point[2] for point in history)),
        )

    def _filter_one_joint(self, name: str, point: Point3D) -> Point3D:
        history = self.joint_history[name]
        if history and self.max_jump_m > 0.0 and _point_distance(point, history[-1]) > self.max_jump_m:
            return history[-1]
        history.append(point)
        return self._median_point(history)

    def _calibrate_arm_lengths(self, skeleton: SkeletonDict) -> None:
        if self.calibrated or not self.enable_arm_length_lock:
            return
        if not all(a in skeleton and b in skeleton for a, b in ARM_BONES):
            return

        for bone in ARM_BONES:
            length = _point_distance(skeleton[bone[0]], skeleton[bone[1]])
            if 0.1 < length < 1.0:
                self.bone_samples[bone].append(length)

        self.calibration_frames += 1
        if self.calibration_frames < self.arm_calib_frames:
            return

        for bone in ARM_BONES:
            samples = self.bone_samples.get(bone, [])
            if samples:
                self.bone_lengths[bone] = float(statistics.median(samples))
        self.calibrated = bool(self.bone_lengths)
        if self.calibrated:
            rospy.loginfo(
                "SpatialArmFilter bone calibration locked, used %d frames",
                self.calibration_frames,
            )

    def _fix_bone_lengths(self, skeleton: SkeletonDict) -> SkeletonDict:
        fixed = dict(skeleton)
        for (joint_a, joint_b), length in self.bone_lengths.items():
            if joint_a not in fixed or joint_b not in fixed:
                continue
            point_a = fixed[joint_a]
            point_b = fixed[joint_b]
            direction = (
                point_b[0] - point_a[0],
                point_b[1] - point_a[1],
                point_b[2] - point_a[2],
            )
            norm = math.sqrt(sum(value * value for value in direction))
            if norm < 1e-6 or length <= 0.0:
                continue
            midpoint = tuple((point_a[index] + point_b[index]) * 0.5 for index in range(3))
            unit = tuple(value / norm for value in direction)
            fixed[joint_a] = tuple(
                midpoint[index] - unit[index] * length * 0.5 for index in range(3)
            )
            fixed[joint_b] = tuple(
                midpoint[index] + unit[index] * length * 0.5 for index in range(3)
            )
        return fixed

    def _clamp_hips(self, skeleton: SkeletonDict) -> SkeletonDict:
        clamped = dict(skeleton)
        for shoulder, hip in (("pose_11", "pose_23"), ("pose_12", "pose_24")):
            if shoulder not in clamped or hip not in clamped:
                continue
            hip_point = clamped[hip]
            clamped[hip] = (
                hip_point[0],
                clamped[shoulder][1] + self.hip_clamp_offset_m,
                hip_point[2],
            )
        return clamped

    def update(self, skeleton: SkeletonDict) -> SkeletonDict:
        if not self.enabled:
            return dict(skeleton)

        filtered: SkeletonDict = {}
        for name in self.landmark_order:
            point = skeleton.get(name)
            if point is not None and _is_finite_point(point):
                filtered[name] = self._filter_one_joint(name, point)

        self._calibrate_arm_lengths(filtered)
        if self.calibrated:
            filtered = self._fix_bone_lengths(filtered)
        if self.enable_hip_clamp:
            filtered = self._clamp_hips(filtered)
        return filtered


class TemporalSkeletonFilter:
    """Loc temporal cho skeleton da qua spatial filter."""

    def __init__(
        self,
        confirm_frames: int,
        lost_frames: int,
        max_jump_m: float,
        smoothing_alpha: float,
        landmark_order: Iterable[str],
    ) -> None:
        self.confirm_frames = max(1, int(confirm_frames))
        self.lost_frames = max(0, int(lost_frames))
        self.max_jump_m = max(0.0, float(max_jump_m))
        self.smoothing_alpha = max(0.0, min(1.0, float(smoothing_alpha)))
        self.landmark_order = list(landmark_order)
        self.valid_count = 0
        self.lost_count = 0
        self.last_skeleton: SkeletonDict = {}

    def reset(self) -> None:
        self.valid_count = 0
        self.lost_count = 0
        self.last_skeleton = {}

    def update(self, skeleton: SkeletonDict, stamp) -> Tuple[bool, SkeletonDict, str]:
        """Cap nhat filter va tra ve skeleton duoc publish neu da confirm."""
        if not skeleton:
            self.lost_count += 1
            if self.lost_count > self.lost_frames:
                self.reset()
            return False, {}, "TEMPORAL_NO_SKELETON lost=%d" % self.lost_count

        filtered: SkeletonDict = {}
        rejected_jump = 0
        for name in self.landmark_order:
            if name not in skeleton:
                continue
            point = skeleton[name]
            if not _is_finite_point(point):
                continue

            if name in self.last_skeleton:
                previous = self.last_skeleton[name]
                jump = _point_distance(point, previous)
                if self.max_jump_m > 0.0 and jump > self.max_jump_m:
                    rejected_jump += 1
                    continue
                alpha = self.smoothing_alpha
                filtered[name] = (
                    previous[0] * (1.0 - alpha) + point[0] * alpha,
                    previous[1] * (1.0 - alpha) + point[1] * alpha,
                    previous[2] * (1.0 - alpha) + point[2] * alpha,
                )
            else:
                filtered[name] = point

        if not filtered:
            self.lost_count += 1
            return False, {}, "TEMPORAL_ALL_JUMP_REJECTED rejected=%d" % rejected_jump

        self.last_skeleton = filtered
        self.valid_count += 1
        self.lost_count = 0
        if self.valid_count < self.confirm_frames:
            return (
                False,
                filtered,
                "TEMPORAL_CONFIRMING valid=%d/%d rejected_jump=%d"
                % (self.valid_count, self.confirm_frames, rejected_jump),
            )

        return (
            True,
            filtered,
            "TEMPORAL_OK valid=%d rejected_jump=%d"
            % (self.valid_count, rejected_jump),
        )


class KinectSkeletonTracker:
    def __init__(self) -> None:
        rospy.init_node("kinect_skeleton_tracker")

        self.camera_frame = rospy.get_param("~camera_frame", "kinect2_ir_optical_frame")
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.camera_name = rospy.get_param("~camera_name", "kinect")
        self.camera_device_index = int(rospy.get_param("~camera_device_index", 0))
        self.camera_serial = str(rospy.get_param("~camera_serial", "")).strip()
        self.camera_packet_pipeline = str(rospy.get_param("~camera_packet_pipeline", "default")).strip()
        self.startup_delay_sec = max(0.0, float(rospy.get_param("~startup_delay_sec", 0.0)))
        self.camera_init_delay_s = max(0.0, float(rospy.get_param("~camera_init_delay_s", 0.0)))
        self._camera_init_delay_done = False
        self.human_skeleton_camera_topic = rospy.get_param("~human_skeleton_camera_topic", "/human_skeleton_camera")
        self.human_skeleton_base_topic = rospy.get_param("~human_skeleton_base_topic", "/human_skeleton_base")
        self.status_topic = rospy.get_param("~status_topic", "/human_skeleton_status")
        self.debug_image_topic = rospy.get_param("~debug_image_topic", "/kinect_skeleton/image_raw")
        self.raw_image_topic = str(rospy.get_param("~raw_image_topic", "")).strip()
        self.publish_static_tf = rospy.get_param("~publish_static_tf", False)
        self.static_tf_xyz = _float_list_param("~static_tf_xyz", [0.0, -0.70, 1.55])
        self.static_tf_rpy = _float_list_param("~static_tf_rpy", [-1.5708, 0.95, -1.5708])
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.rate_cmd_topic = str(rospy.get_param("~rate_cmd_topic", "")).strip()
        self.rate_min = float(rospy.get_param("~rate_min", 1.0))
        self.rate_max = float(rospy.get_param("~rate_max", 60.0))
        self._rate_lock = threading.Lock()
        self._target_rate_hz = clamp_rate(self.rate_hz, self.rate_min, self.rate_max)
        if self._target_rate_hz < self.rate_hz:
            rospy.logwarn(
                "[%s] configured rate_hz=%.1f clamped to %.1f by rate_max=%.1f",
                self.camera_name,
                self.rate_hz,
                self._target_rate_hz,
                self.rate_max,
            )
        self.frame_timeout_ms = int(rospy.get_param("~frame_timeout_ms", 1000))
        self.rgb_only_mode = _bool_param("~rgb_only_mode", False)
        self.enable_depth = _bool_param("~enable_depth", True)
        self._sensor_mode = self._resolve_sensor_mode()
        self.reopen_on_frame_timeout = rospy.get_param("~reopen_on_frame_timeout", True)
        self.restart_process_on_frame_timeout = rospy.get_param("~restart_process_on_frame_timeout", True)
        self.frame_timeout_reopen_threshold = max(
            1,
            int(rospy.get_param("~frame_timeout_reopen_threshold", 3)),
        )
        self.camera_reopen_backoff_sec = max(
            0.0,
            float(rospy.get_param("~camera_reopen_backoff_sec", 1.0)),
        )
        self.camera_reopen_timeout_s = max(
            0.0,
            float(rospy.get_param("~camera_reopen_timeout_s", 0.0)),
        )
        self.status_interval_sec = float(rospy.get_param("~status_interval_sec", 1.0))
        self.tf_timeout_sec = float(rospy.get_param("~tf_timeout_sec", 0.2))
        self.debug_image_flip_horizontal = rospy.get_param("~debug_image_flip_horizontal", False)
        self.debug_image_flip_vertical = rospy.get_param("~debug_image_flip_vertical", False)

        self.enable_yolo_person_segmentation = rospy.get_param("~enable_yolo_person_segmentation", True)
        self.yolo_model_path = rospy.get_param("~yolo_model_path", "yolov8n-seg.pt")
        self.yolo_device = rospy.get_param("~yolo_device", "cpu")
        self.yolo_gpu_memory_fraction = float(rospy.get_param("~yolo_gpu_memory_fraction", 0.45))
        self.yolo_conf_threshold = float(rospy.get_param("~yolo_conf_threshold", 0.45))
        self.yolo_iou_threshold = float(rospy.get_param("~yolo_iou_threshold", 0.50))
        self.person_class_id = int(rospy.get_param("~person_class_id", 0))
        self.person_mask_min_area_ratio = float(rospy.get_param("~person_mask_min_area_ratio", 0.005))
        self.person_mask_dilate_px = int(rospy.get_param("~person_mask_dilate_px", 5))
        self.person_mask_prefer_center = rospy.get_param("~person_mask_prefer_center", True)
        self.person_select_mode = str(rospy.get_param("~person_select_mode", "legacy")).strip().lower()
        self.nearest_min_depth_px = max(1, int(rospy.get_param("~nearest_min_depth_px", 50)))
        self.require_person_mask_for_pose = rospy.get_param("~require_person_mask_for_pose", True)
        self.require_person_mask_for_hands = rospy.get_param("~require_person_mask_for_hands", True)

        self.enable_holistic = rospy.get_param("~enable_holistic", True)
        self.holistic_model_complexity = int(rospy.get_param("~holistic_model_complexity", 0))
        self.holistic_min_detection_confidence = float(rospy.get_param("~holistic_min_detection_confidence", 0.5))
        self.holistic_min_tracking_confidence = float(rospy.get_param("~holistic_min_tracking_confidence", 0.5))
        self.holistic_enable_segmentation = rospy.get_param("~holistic_enable_segmentation", False)
        self.min_visibility = float(rospy.get_param("~min_visibility", 0.6))
        self.pose_landmark_ids = _int_list_param("~pose_landmark_ids", [0, 11, 12, 13, 14, 15, 16, 23, 24])
        self.body_min_core_points = int(rospy.get_param("~body_min_core_points", 5))
        self.shoulder_width_range = _float_tuple_param("~shoulder_width_range", (0.20, 0.75))
        self.torso_length_range = _float_tuple_param("~torso_length_range", (0.25, 1.00))

        self.enable_hand_detection = rospy.get_param("~enable_hand_detection", True)
        self.min_hand_valid_points = int(rospy.get_param("~min_hand_valid_points", 8))
        self.max_finger_span = float(rospy.get_param("~max_finger_span", 0.30))
        self.min_palm_size = float(rospy.get_param("~min_palm_size", 0.025))
        self.max_palm_size = float(rospy.get_param("~max_palm_size", 0.18))
        self.hand_wrist_attach_max_distance_m = float(rospy.get_param("~hand_wrist_attach_max_distance_m", 0.15))
        self.hand_wrist_attach_max_distance_px = float(rospy.get_param("~hand_wrist_attach_max_distance_px", 80.0))
        self.allow_hand_side_swap = rospy.get_param("~allow_hand_side_swap", True)

        self.temporal_filter = TemporalSkeletonFilter(
            confirm_frames=rospy.get_param("~temporal_confirm_frames", 1),
            lost_frames=rospy.get_param("~temporal_lost_frames", 99),
            max_jump_m=rospy.get_param("~temporal_max_jump_m", 999.0),
            smoothing_alpha=rospy.get_param("~temporal_smoothing_alpha", 0.0),
            landmark_order=LANDMARK_ORDER,
        )
        self.spatial_filter = SpatialArmFilter(
            enabled=_bool_param("~enable_arm_spatial_filter", True),
            median_window=rospy.get_param("~arm_median_window", 2),
            max_jump_m=rospy.get_param("~arm_max_jump_m", 0.5),
            enable_arm_length_lock=_bool_param("~enable_arm_length_lock", True),
            arm_calib_frames=rospy.get_param("~arm_calib_frames", 50),
            enable_hip_clamp=_bool_param("~enable_hip_clamp", False),
            hip_clamp_offset_m=rospy.get_param("~hip_clamp_offset_m", 0.30),
            landmark_order=LANDMARK_ORDER,
        )

        self.draw_person_mask = rospy.get_param("~draw_person_mask", True)
        self.draw_rejected_raw = rospy.get_param("~draw_rejected_raw", False)
        self.camera_pub = rospy.Publisher(self.human_skeleton_camera_topic, PoseArray, queue_size=1)
        self.base_pub = rospy.Publisher(self.human_skeleton_base_topic, PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.image_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        self.raw_image_pub = (
            rospy.Publisher(self.raw_image_topic, Image, queue_size=1)
            if self.raw_image_topic
            else None
        )

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_broadcaster = None
        if self.publish_static_tf:
            self.static_broadcaster = publish_static_transform(
                self.target_frame,
                self.camera_frame,
                self.static_tf_xyz,
                self.static_tf_rpy,
            )

        self.yolo_model = None
        self.yolo_error = ""
        if self.enable_yolo_person_segmentation:
            try:
                self.yolo_model = load_yolo_segment_model(
                    self.yolo_model_path,
                    self.yolo_device,
                    gpu_memory_fraction=self.yolo_gpu_memory_fraction,
                )
            except Exception as exc:
                self.yolo_error = str(exc)
                rospy.logerr("YOLO person segmentation unavailable: %s", exc)
        self._yolo_async = (
            self.enable_yolo_person_segmentation
            and rospy.get_param("~yolo_async_thread", False)
        )
        if self._yolo_async and self.person_select_mode == "nearest" and self._sensor_mode == "rgb_depth":
            self._yolo_async = False
            rospy.loginfo(
                "[%s] YOLO async disabled: nearest person selection requires current depth frame",
                self.camera_name,
            )
        self._yolo_mask_cache = None
        self._yolo_cache_lock = threading.Lock()
        if self._yolo_async and self.yolo_model is not None:
            self._yolo_input_q = queue.Queue(maxsize=1)
            self._yolo_thread = threading.Thread(
                target=self._yolo_worker,
                name="yolo_worker_%s" % self.camera_name,
                daemon=True,
            )
            self._yolo_thread.start()
            rospy.loginfo("[%s] YOLO async thread started", self.camera_name)

        self.holistic = None
        self.holistic_error = ""
        if self.enable_holistic:
            try:
                self.holistic = load_mediapipe_holistic(
                    model_complexity=self.holistic_model_complexity,
                    min_detection_confidence=self.holistic_min_detection_confidence,
                    min_tracking_confidence=self.holistic_min_tracking_confidence,
                    enable_segmentation=self.holistic_enable_segmentation,
                )
            except Exception as exc:
                self.holistic_error = str(exc)
                rospy.logerr("MediaPipe Holistic unavailable: %s", exc)

        self.freenect = None
        self.device = None
        self.listener = None
        self.registration = None
        self.camera_error = ""
        self.last_status_text = ""
        self.last_status_time = rospy.Time(0)
        self.last_frame_time = None
        self.last_fps = 0.0
        self.consecutive_frame_timeouts = 0
        self.camera_reopen_count = 0

        rospy.loginfo(
            "TRACKER_INIT camera=%s serial=%s index=%d pipeline=%s startup_delay=%.1fs frame=%s target=%s",
            self.camera_name,
            self.camera_serial or "<by-index>",
            self.camera_device_index,
            self.camera_packet_pipeline,
            self.startup_delay_sec,
            self.camera_frame,
            self.target_frame,
        )
        rospy.loginfo("kinect_skeleton_tracker schema landmarks=%d", len(LANDMARK_ORDER))
        rospy.loginfo(
            "Sensor mode camera=%s mode=%s rgb_only_mode=%s enable_depth=%s",
            self.camera_name,
            self._sensor_mode,
            self.rgb_only_mode,
            self.enable_depth,
        )
        rospy.loginfo("YOLO enabled=%s model=%s device=%s", self.enable_yolo_person_segmentation, self.yolo_model_path, self.yolo_device)
        rospy.loginfo("Holistic enabled=%s complexity=%d", self.enable_holistic, self.holistic_model_complexity)
        rospy.loginfo("Hand detection enabled=%s min_points=%d", self.enable_hand_detection, self.min_hand_valid_points)
        rospy.loginfo(
            "Frame timeout recovery enabled=%s restart_process=%s threshold=%d backoff=%.1fs",
            self.reopen_on_frame_timeout,
            self.restart_process_on_frame_timeout,
            self.frame_timeout_reopen_threshold,
            self.camera_reopen_backoff_sec,
        )
        if self.rate_cmd_topic:
            rospy.Subscriber(self.rate_cmd_topic, Float32, self._on_rate_cmd, queue_size=1)
            rospy.loginfo(
                "[%s] runtime rate control on %s (clamp %.1f-%.1f Hz)",
                self.camera_name,
                self.rate_cmd_topic,
                self.rate_min,
                self.rate_max,
            )

    def _resolve_sensor_mode(self) -> str:
        if self.rgb_only_mode:
            return "rgb_only"
        if self.enable_depth:
            return "rgb_depth"
        rospy.logwarn(
            "[%s] enable_depth=false with rgb_only_mode=false; falling back to rgb_only",
            self.camera_name,
        )
        return "rgb_only"

    def publish_status(self, text: str, force: bool = False) -> None:
        text = "camera=%s %s" % (self.camera_name, text)
        now = rospy.Time.now()
        age = (now - self.last_status_time).to_sec() if self.last_status_time != rospy.Time(0) else float("inf")
        if force or text != self.last_status_text or age >= self.status_interval_sec:
            self.status_pub.publish(text)
            self.last_status_text = text
            self.last_status_time = now

    def publish_empty_skeleton(self, stamp, reason: str) -> None:
        self.camera_pub.publish(skeleton_dict_to_pose_array({}, self.camera_frame, stamp, LANDMARK_ORDER))
        self.base_pub.publish(skeleton_dict_to_pose_array({}, self.target_frame, stamp, LANDMARK_ORDER))
        self.publish_status(reason)

    def open_camera(self) -> bool:
        if self.startup_delay_sec > 0.0:
            rospy.loginfo(
                "kinect_skeleton_tracker camera=%s waiting %.1fs before opening Kinect",
                self.camera_name,
                self.startup_delay_sec,
            )
            rospy.sleep(self.startup_delay_sec)
            if rospy.is_shutdown():
                return False

        try:
            self.freenect, self.device, self.listener, self.registration = initialize_camera(
                device_index=self.camera_device_index,
                serial=self.camera_serial,
                packet_pipeline=self.camera_packet_pipeline,
                color_only=(self._sensor_mode == "rgb_only"),
            )
        except Exception as exc:
            self.camera_error = str(exc)
            self.publish_status("CAMERA_ERROR %s" % self.camera_error, force=True)
            rospy.logerr("kinect_skeleton_tracker camera=%s cannot open Kinect: %s", self.camera_name, exc)
            return False

        rospy.loginfo(
            "kinect_skeleton_tracker camera=%s initialized serial=%s index=%d pipeline=%s mode=%s",
            self.camera_name,
            self.camera_serial or "<by-index>",
            self.camera_device_index,
            self.camera_packet_pipeline,
            self._sensor_mode,
        )
        self.publish_status(
            "CAMERA_OK serial=%s index=%d pipeline=%s mode=%s"
            % (
                self.camera_serial or "<by-index>",
                self.camera_device_index,
                self.camera_packet_pipeline,
                self._sensor_mode,
            ),
            force=True,
        )
        return True

    def _yolo_worker(self) -> None:
        rospy.loginfo("[%s] YOLO worker thread running on %s", self.camera_name, self.yolo_device)
        while not rospy.is_shutdown():
            try:
                frame_rgb = self._yolo_input_q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                detections = run_yolo_person_segmentation(
                    self.yolo_model,
                    frame_rgb,
                    self.yolo_conf_threshold,
                    self.yolo_iou_threshold,
                    self.person_class_id,
                )
                person_mask, _best = select_best_person_mask(
                    detections,
                    frame_rgb.shape,
                    self.person_mask_min_area_ratio,
                    self.person_mask_prefer_center,
                )
                if person_mask is not None:
                    person_mask = dilate_mask(person_mask, self.person_mask_dilate_px)
                with self._yolo_cache_lock:
                    self._yolo_mask_cache = person_mask
            except Exception as exc:
                rospy.logwarn_throttle(
                    10.0,
                    "[%s] YOLO worker error: %s",
                    self.camera_name,
                    exc,
                )

    def close_camera(self) -> None:
        device = self.device
        self.device = None
        self.listener = None
        self.registration = None
        self.freenect = None

        if device is None:
            return
        try:
            device.stop()
        except Exception as exc:
            rospy.logwarn("Error while stopping Kinect device: %s", exc)
        try:
            device.close()
        except Exception as exc:
            rospy.logwarn("Error while closing Kinect device: %s", exc)

    def maybe_reopen_camera_after_timeout(self, reason: str) -> None:
        if not self.reopen_on_frame_timeout:
            return
        if self.consecutive_frame_timeouts < self.frame_timeout_reopen_threshold:
            return
        if self.camera_reopen_timeout_s > 0.0 and self.last_frame_time is not None:
            frame_age = time.time() - self.last_frame_time
            if frame_age < self.camera_reopen_timeout_s:
                return

        self.camera_reopen_count += 1
        rospy.logwarn(
            "kinect_skeleton_tracker camera=%s recovering after %d consecutive frame timeouts: %s",
            self.camera_name,
            self.consecutive_frame_timeouts,
            reason,
        )
        self.publish_status(
            "CAMERA_RECOVERY frame_timeouts=%d reopen_count=%d reason=%s"
            % (self.consecutive_frame_timeouts, self.camera_reopen_count, reason),
            force=True,
        )

        if self.restart_process_on_frame_timeout:
            rospy.logwarn(
                "kinect_skeleton_tracker camera=%s exiting for roslaunch respawn",
                self.camera_name,
            )
            if self.camera_reopen_backoff_sec > 0.0 and not rospy.is_shutdown():
                rospy.sleep(min(0.5, self.camera_reopen_backoff_sec))
            os._exit(75)

        self.close_camera()
        self.consecutive_frame_timeouts = 0
        if self.camera_reopen_backoff_sec > 0.0 and not rospy.is_shutdown():
            rospy.sleep(self.camera_reopen_backoff_sec)

    def transform_skeleton_to_target(self, skeleton_camera: SkeletonDict, stamp) -> SkeletonDict:
        skeleton_base: SkeletonDict = {}
        for name, point_3d in skeleton_camera.items():
            if not _is_finite_point(point_3d):
                continue

            point = PointStamped()
            point.header.frame_id = self.camera_frame
            point.header.stamp = stamp
            point.point.x = point_3d[0]
            point.point.y = point_3d[1]
            point.point.z = point_3d[2]

            transformed = self.tf_buffer.transform(
                point,
                self.target_frame,
                rospy.Duration(self.tf_timeout_sec),
            )
            skeleton_base[name] = (
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            )
        return skeleton_base

    def _update_fps(self) -> float:
        now = time.time()
        if self.last_frame_time is not None:
            dt = now - self.last_frame_time
            if dt > 1e-6:
                self.last_fps = 1.0 / dt
        self.last_frame_time = now
        return self.last_fps

    def _make_debug_image(self, image_bgr, skeleton_pixels, person_mask, status_text, fps):
        debug_image = draw_filtered_skeleton(
            image_bgr,
            skeleton_pixels,
            person_mask=person_mask,
            draw_person_mask=self.draw_person_mask,
            draw_body=True,
            draw_hands=True,
            draw_reject_regions=False,
            status_text=status_text,
            fps=fps,
        )
        debug_image = _flip_image(
            debug_image,
            flip_horizontal=self.debug_image_flip_horizontal,
            flip_vertical=self.debug_image_flip_vertical,
        )
        return debug_image

    def publish_raw_image(self, image_bgr, stamp) -> None:
        if self.raw_image_pub is None:
            return
        image_msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.camera_frame
        self.raw_image_pub.publish(image_msg)

    def _extract_rgb_only_pixels(self, results, width: int, height: int) -> SkeletonPixels:
        pixels: SkeletonPixels = {}

        pose_landmarks = getattr(results, "pose_landmarks", None)
        if pose_landmarks is not None:
            landmarks = getattr(pose_landmarks, "landmark", [])
            for landmark_id in self.pose_landmark_ids:
                if 0 <= landmark_id < len(landmarks):
                    landmark = landmarks[landmark_id]
                    pixels["pose_%d" % landmark_id] = (
                        int(landmark.x * width),
                        int(landmark.y * height),
                    )

        if self.enable_hand_detection:
            for side, attr in (
                ("left_hand", "left_hand_landmarks"),
                ("right_hand", "right_hand_landmarks"),
            ):
                hand_landmarks = getattr(results, attr, None)
                if hand_landmarks is None:
                    continue
                for index, landmark in enumerate(getattr(hand_landmarks, "landmark", [])):
                    pixels["%s_%d" % (side, index)] = (
                        int(landmark.x * width),
                        int(landmark.y * height),
                    )

        return pixels

    def process_rgb_only_frame(self) -> Dict[str, object]:
        stamp = rospy.Time.now()
        color_bgr, color_rgb = read_color_frame_only(
            self.listener,
            self.frame_timeout_ms,
        )
        if self.person_select_mode == "nearest":
            rospy.logwarn_throttle(
                10.0,
                "[%s] nearest person selection unavailable in RGB-only mode; using legacy selector",
                self.camera_name,
            )
        self.publish_raw_image(color_bgr, stamp)
        fps = self._update_fps()

        status_parts = ["RGB_ONLY", "no_depth"]
        person_mask = None
        if self.enable_yolo_person_segmentation:
            if self.yolo_model is None:
                status_parts.append("YOLO_ERROR_%s" % (self.yolo_error or "model_not_loaded"))
            elif self._yolo_async:
                try:
                    self._yolo_input_q.put_nowait(color_rgb.copy())
                except queue.Full:
                    pass
                with self._yolo_cache_lock:
                    person_mask = self._yolo_mask_cache
                if person_mask is None:
                    status_parts.append("YOLO_ASYNC_PENDING")
            else:
                try:
                    detections = run_yolo_person_segmentation(
                        self.yolo_model,
                        color_rgb,
                        self.yolo_conf_threshold,
                        self.yolo_iou_threshold,
                        self.person_class_id,
                    )
                    person_mask, _best = select_best_person_mask(
                        detections,
                        color_rgb.shape,
                        self.person_mask_min_area_ratio,
                        self.person_mask_prefer_center,
                        select_mode="legacy",
                    )
                    if person_mask is None:
                        status_parts.append("NO_PERSON_MASK detections=%d" % len(detections))
                    else:
                        person_mask = dilate_mask(person_mask, self.person_mask_dilate_px)
                        status_parts.append("YOLO_OK")
                except Exception as exc:
                    status_parts.append("YOLO_ERROR_%s" % exc)
        else:
            status_parts.append("YOLO_DISABLED")

        skeleton_pixels: SkeletonPixels = {}
        if self.enable_holistic and self.holistic is not None:
            try:
                results = run_holistic(self.holistic, color_rgb)
                height, width = color_rgb.shape[:2]
                skeleton_pixels = self._extract_rgb_only_pixels(results, width, height)
                status_parts.append("HOLISTIC_2D points=%d" % len(skeleton_pixels))
            except Exception as exc:
                status_parts.append("HOLISTIC_ERROR_%s" % exc)
        elif self.enable_holistic:
            status_parts.append("HOLISTIC_ERROR_%s" % (self.holistic_error or "holistic_not_loaded"))
        else:
            status_parts.append("HOLISTIC_DISABLED")

        status = " ".join(status_parts)
        debug_image = self._make_debug_image(
            color_bgr,
            skeleton_pixels,
            person_mask,
            status,
            fps,
        )
        return {
            "ok": False,
            "reason": status,
            "skeleton_camera": {},
            "skeleton_base": {},
            "skeleton_pixels": {},
            "debug_image_bgr": debug_image,
            "stamp": stamp,
        }

    def process_one_frame(self) -> Dict[str, object]:
        stamp = rospy.Time.now()
        registered_bgr, registered_rgb, undistorted, _registered = read_registered_frames(
            self.listener,
            self.registration,
            self.frame_timeout_ms,
        )
        self.publish_raw_image(registered_bgr, stamp)
        fps = self._update_fps()

        def reject(reason: str, mask=None):
            self.spatial_filter.reset()
            debug_image = self._make_debug_image(registered_bgr, {}, mask, reason, fps)
            return {
                "ok": False,
                "reason": reason,
                "skeleton_camera": {},
                "skeleton_base": {},
                "skeleton_pixels": {},
                "debug_image_bgr": debug_image,
                "stamp": stamp,
            }

        person_mask = None
        if self.enable_yolo_person_segmentation:
            if self.yolo_model is None:
                return reject("YOLO_ERROR %s" % (self.yolo_error or "model_not_loaded"))
            if self._yolo_async:
                try:
                    self._yolo_input_q.put_nowait(registered_rgb.copy())
                except queue.Full:
                    pass
                with self._yolo_cache_lock:
                    person_mask = self._yolo_mask_cache
            else:
                try:
                    detections = run_yolo_person_segmentation(
                        self.yolo_model,
                        registered_rgb,
                        self.yolo_conf_threshold,
                        self.yolo_iou_threshold,
                        self.person_class_id,
                    )
                except Exception as exc:
                    return reject("YOLO_ERROR %s" % exc)

                person_mask, _best = select_best_person_mask(
                    detections,
                    registered_rgb.shape,
                    self.person_mask_min_area_ratio,
                    self.person_mask_prefer_center,
                    select_mode=self.person_select_mode,
                    depth_map=undistorted.asarray(dtype=np.float32),
                    min_depth_px=self.nearest_min_depth_px,
                )
                if person_mask is None:
                    return reject("NO_PERSON_MASK detections=%d" % len(detections))
                person_mask = dilate_mask(person_mask, self.person_mask_dilate_px)
        elif self.require_person_mask_for_pose or self.require_person_mask_for_hands:
            return reject("PERSON_MASK_REQUIRED_BUT_YOLO_DISABLED")

        if not self.enable_holistic or self.holistic is None:
            return reject("HOLISTIC_ERROR %s" % (self.holistic_error or "holistic_not_loaded"), person_mask)

        holistic_rgb = registered_rgb
        if self.person_select_mode == "nearest" and person_mask is not None:
            holistic_rgb = registered_rgb.copy()
            holistic_rgb[~person_mask.astype(bool)] = 0

        try:
            results = run_holistic(self.holistic, holistic_rgb)
        except Exception as exc:
            return reject("HOLISTIC_ERROR %s" % exc, person_mask)

        height, width = registered_rgb.shape[:2]
        pose_mask = person_mask if self.require_person_mask_for_pose else None
        hand_mask = person_mask if self.require_person_mask_for_hands else None

        pose_points, pose_pixels = extract_pose_landmarks_3d(
            results,
            self.registration,
            undistorted,
            pose_mask,
            width,
            height,
            self.min_visibility,
            self.pose_landmark_ids,
        )
        body_ok, body_reason = validate_body_geometry(
            pose_points,
            min_core_points=self.body_min_core_points,
            shoulder_width_range=self.shoulder_width_range,
            torso_length_range=self.torso_length_range,
        )
        if not body_ok:
            return reject(body_reason, person_mask)

        hand_points_by_side: Dict[str, SkeletonDict] = {"left": {}, "right": {}}
        hand_pixels_by_side: Dict[str, SkeletonPixels] = {"left": {}, "right": {}}
        hand_status: List[str] = []
        if self.enable_hand_detection:
            raw_left_points, raw_left_pixels = extract_hand_landmarks_3d(
                getattr(results, "left_hand_landmarks", None),
                "left_hand",
                self.registration,
                undistorted,
                hand_mask,
                width,
                height,
                self.min_hand_valid_points,
            )
            raw_right_points, raw_right_pixels = extract_hand_landmarks_3d(
                getattr(results, "right_hand_landmarks", None),
                "right_hand",
                self.registration,
                undistorted,
                hand_mask,
                width,
                height,
                self.min_hand_valid_points,
            )

            for side, points, pixels in (
                ("left", raw_left_points, raw_left_pixels),
                ("right", raw_right_points, raw_right_pixels),
            ):
                if not points:
                    continue
                hand_ok, reason = validate_hand_geometry(
                    points,
                    min_valid_points=self.min_hand_valid_points,
                    max_finger_span=self.max_finger_span,
                    min_palm_size=self.min_palm_size,
                    max_palm_size=self.max_palm_size,
                )
                if hand_ok:
                    hand_points_by_side[side] = points
                    hand_pixels_by_side[side] = pixels
                hand_status.append("%s_%s" % (side, reason))

        attached_hands, attached_pixels, attach_status = attach_hands_to_pose_wrists(
            pose_points,
            hand_points_by_side,
            hand_pixels_by_side,
            pose_pixels=pose_pixels,
            max_attach_distance_m=self.hand_wrist_attach_max_distance_m,
            max_attach_distance_px=self.hand_wrist_attach_max_distance_px,
            allow_swap=self.allow_hand_side_swap,
        )
        skeleton_camera = merge_pose_and_hands(
            pose_points,
            attached_hands.get("left", {}),
            attached_hands.get("right", {}),
        )
        skeleton_pixels = dict(pose_pixels)
        skeleton_pixels.update(attached_pixels.get("left", {}))
        skeleton_pixels.update(attached_pixels.get("right", {}))

        if not skeleton_camera:
            return reject("NO_VALID_SKELETON", person_mask)

        skeleton_camera = self.spatial_filter.update(skeleton_camera)
        skeleton_base = self.transform_skeleton_to_target(skeleton_camera, stamp)
        temporal_ok, temporal_base, temporal_reason = self.temporal_filter.update(skeleton_base, stamp)
        if not temporal_ok:
            return reject(temporal_reason, person_mask)

        filtered_camera = {
            name: skeleton_camera[name]
            for name in temporal_base
            if name in skeleton_camera
        }
        filtered_pixels = {
            name: skeleton_pixels[name]
            for name in temporal_base
            if name in skeleton_pixels
        }

        status = "OK body=%d hands=%d %s %s %s" % (
            sum(1 for name in temporal_base if name in BODY_LANDMARK_ORDER),
            sum(1 for name in temporal_base if is_hand_landmark(name)),
            "SKELETON_BLOCKERS_DISABLED",
            temporal_reason,
            attach_status,
        )
        if hand_status:
            status += " " + ";".join(hand_status)

        debug_source = registered_bgr
        debug_image = self._make_debug_image(debug_source, filtered_pixels, person_mask, status, fps)
        return {
            "ok": True,
            "reason": status,
            "skeleton_camera": filtered_camera,
            "skeleton_base": temporal_base,
            "skeleton_pixels": filtered_pixels,
            "debug_image_bgr": debug_image,
            "stamp": stamp,
        }

    def publish_frame_result(self, result: Dict[str, object]) -> None:
        stamp = result.get("stamp", rospy.Time.now())
        reason = str(result.get("reason", "UNKNOWN"))
        skeleton_camera = result.get("skeleton_camera", {})
        skeleton_base = result.get("skeleton_base", {})

        if result.get("ok"):
            self.camera_pub.publish(
                skeleton_dict_to_pose_array(skeleton_camera, self.camera_frame, stamp, LANDMARK_ORDER)
            )
            self.base_pub.publish(
                skeleton_dict_to_pose_array(skeleton_base, self.target_frame, stamp, LANDMARK_ORDER)
            )
            self.publish_status(reason)
        else:
            self.publish_empty_skeleton(stamp, reason)

        debug_image = result.get("debug_image_bgr")
        if debug_image is not None:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8"))

    def _on_rate_cmd(self, msg) -> None:
        if not math.isfinite(float(msg.data)):
            rospy.logwarn_throttle(5.0, "[%s] ignoring non-finite rate command", self.camera_name)
            return
        new_rate = clamp_rate(msg.data, self.rate_min, self.rate_max)
        with self._rate_lock:
            changed = abs(new_rate - self._target_rate_hz) > 1e-6
            self._target_rate_hz = new_rate
        if changed:
            rospy.loginfo("[%s] tracker rate command -> %.2f Hz", self.camera_name, new_rate)

    def spin(self) -> None:
        with self._rate_lock:
            applied_rate_hz = self._target_rate_hz
        rate = rospy.Rate(applied_rate_hz)
        while not rospy.is_shutdown():
            with self._rate_lock:
                target_rate_hz = self._target_rate_hz
            if abs(target_rate_hz - applied_rate_hz) > 1e-6:
                applied_rate_hz = target_rate_hz
                rate = rospy.Rate(applied_rate_hz)
                rospy.loginfo("[%s] tracker loop now %.2f Hz", self.camera_name, applied_rate_hz)
            needs_camera = self.listener is None or (
                self._sensor_mode == "rgb_depth" and self.registration is None
            )
            if needs_camera:
                if self.camera_init_delay_s > 0.0 and not self._camera_init_delay_done:
                    rospy.loginfo(
                        "[%s] Waiting %.1fs before camera init (USB stagger)",
                        self.camera_name,
                        self.camera_init_delay_s,
                    )
                    rospy.sleep(self.camera_init_delay_s)
                    self._camera_init_delay_done = True
                    if rospy.is_shutdown():
                        break
                if not self.open_camera():
                    self.publish_empty_skeleton(rospy.Time.now(), "CAMERA_ERROR %s" % self.camera_error)
                    rate.sleep()
                    continue

            try:
                if self._sensor_mode == "rgb_only":
                    result = self.process_rgb_only_frame()
                else:
                    result = self.process_one_frame()
                self.consecutive_frame_timeouts = 0
                self.publish_frame_result(result)
            except KinectFrameTimeout as exc:
                self.consecutive_frame_timeouts += 1
                self.spatial_filter.reset()
                self.temporal_filter.update({}, rospy.Time.now())
                self.publish_empty_skeleton(
                    rospy.Time.now(),
                    "FRAME_TIMEOUT %s count=%d"
                    % (exc, self.consecutive_frame_timeouts),
                )
                rospy.logwarn_throttle(2.0, "Skeleton tracker Kinect frame timeout: %s", exc)
                self.maybe_reopen_camera_after_timeout(str(exc))
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                self.spatial_filter.reset()
                self.temporal_filter.update({}, rospy.Time.now())
                self.publish_empty_skeleton(rospy.Time.now(), "TF_ERROR %s" % exc)
                rospy.logwarn_throttle(2.0, "Skeleton TF transform failed: %s", exc)
            except Exception as exc:
                self.spatial_filter.reset()
                self.temporal_filter.update({}, rospy.Time.now())
                self.publish_empty_skeleton(rospy.Time.now(), "ERROR %s" % exc)
                rospy.logerr_throttle(2.0, "Skeleton tracker error: %s", exc)
            rate.sleep()

    def shutdown(self) -> None:
        self.close_camera()
        if self.holistic is not None:
            try:
                self.holistic.close()
            except Exception:
                pass
        cv2.destroyAllWindows()


def main() -> None:
    tracker = KinectSkeletonTracker()
    try:
        tracker.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        tracker.shutdown()


if __name__ == "__main__":
    main()
