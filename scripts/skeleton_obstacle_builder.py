#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from data_skeleton import pose_array_to_numeric_joint_dict


Point3D = Tuple[float, float, float]
JointId = Union[int, str]
SkeletonDict = Dict[JointId, Point3D]
ConnectionPair = Tuple[JointId, JointId]

DEFAULT_TRACKED_JOINT_IDS = [0, 11, 12, 13, 14, 15, 16, 23, 24]
ARM_JOINT_IDS: frozenset = frozenset({13, 14, 15, 16})  # MediaPipe: left/right elbow + wrist
DEFAULT_CONNECTION_PAIRS: List[ConnectionPair] = [
    (11, 13),
    (13, 15),
    (12, 14),
    (14, 16),
    (11, 12),
    (11, 23),
    (12, 24),
    (23, 24),
]


def _parse_joint_id(value: Any) -> JointId:
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, int):
        return value
    text = str(value).strip()
    try:
        return int(text)
    except ValueError:
        return text


def _int_list_param(name: str, default: Iterable[int]) -> List[int]:
    raw_value = rospy.get_param(name, list(default))
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
            rospy.logwarn("Ignoring invalid integer value '%s' in param %s", item, name)

    return result if result else list(default)


def _connection_pairs_param(
    name: str,
    default: Iterable[ConnectionPair],
) -> List[ConnectionPair]:
    raw_value = rospy.get_param(name, ["%s,%s" % pair for pair in default])
    pairs: List[ConnectionPair] = []

    if isinstance(raw_value, str):
        raw_pairs = [item.strip() for item in raw_value.split(";") if item.strip()]
        for raw_pair in raw_pairs:
            parts = [part.strip() for part in raw_pair.split(",") if part.strip()]
            if len(parts) != 2:
                rospy.logwarn("Ignoring invalid connection pair '%s' in param %s", raw_pair, name)
                continue
            pairs.append((_parse_joint_id(parts[0]), _parse_joint_id(parts[1])))
        return pairs if pairs else list(default)

    if isinstance(raw_value, (list, tuple)):
        for item in raw_value:
            if isinstance(item, str):
                parts = [part.strip() for part in item.split(",") if part.strip()]
            elif isinstance(item, (list, tuple)):
                parts = list(item)
            else:
                rospy.logwarn("Ignoring invalid connection pair '%s' in param %s", item, name)
                continue

            if len(parts) != 2:
                rospy.logwarn("Ignoring invalid connection pair '%s' in param %s", item, name)
                continue
            pairs.append((_parse_joint_id(parts[0]), _parse_joint_id(parts[1])))

    return pairs if pairs else list(default)


def is_finite_point(point: Any) -> bool:
    if not isinstance(point, (tuple, list)) or len(point) != 3:
        return False
    try:
        x, y, z = float(point[0]), float(point[1]), float(point[2])
    except (TypeError, ValueError):
        return False
    return math.isfinite(x) and math.isfinite(y) and math.isfinite(z)


def point_distance(a: Point3D, b: Point3D) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _quaternion_from_z_axis(direction: Point3D) -> Tuple[float, float, float, float]:
    length = math.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2)
    if length <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)

    vx = direction[0] / length
    vy = direction[1] / length
    vz = direction[2] / length

    dot = _clamp(vz, -1.0, 1.0)
    if dot > 1.0 - 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1e-9:
        return (1.0, 0.0, 0.0, 0.0)

    axis_x = -vy
    axis_y = vx
    axis_z = 0.0
    axis_len = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
    if axis_len <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)

    angle = math.acos(dot)
    sin_half = math.sin(angle * 0.5)
    return (
        axis_x / axis_len * sin_half,
        axis_y / axis_len * sin_half,
        axis_z / axis_len * sin_half,
        math.cos(angle * 0.5),
    )


class SkeletonObstacleBuilder:
    def __init__(self) -> None:
        rospy.init_node("skeleton_obstacle_builder")

        self.skeleton_topic = rospy.get_param("~skeleton_topic", "/human_skeleton_base")
        self.collision_object_topic = rospy.get_param(
            "~collision_object_topic",
            "/human_collision_object",
        )
        self.status_topic = rospy.get_param("~status_topic", "/human_obstacle_status")
        self.visualization_marker_topic = rospy.get_param(
            "~visualization_marker_topic",
            "/human_collision_markers",
        )
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.object_id = rospy.get_param("~object_id", "human_skeleton")
        self.joint_radius = float(rospy.get_param("~joint_radius", 0.03))
        self.bone_radius = float(rospy.get_param("~bone_radius", 0.03))
        self.body_joint_radius = float(rospy.get_param("~body_joint_radius", self.joint_radius))
        self.body_bone_radius = float(rospy.get_param("~body_bone_radius", self.bone_radius))
        self.arm_joint_radius = float(rospy.get_param("~arm_joint_radius", self.body_joint_radius))
        self.arm_bone_radius = float(rospy.get_param("~arm_bone_radius", self.body_bone_radius))
        self.static_safety_padding = float(rospy.get_param("~static_safety_padding", 0.0))
        self.speed_padding_gain = float(rospy.get_param("~speed_padding_gain", 0.0))
        self.speed_padding_deadband = float(rospy.get_param("~speed_padding_deadband", 0.05))
        self.max_dynamic_padding = float(rospy.get_param("~max_dynamic_padding", 0.0))
        self.padding_smoothing_alpha = _clamp(
            float(rospy.get_param("~padding_smoothing_alpha", 0.70)),
            0.0,
            1.0,
        )
        self.max_reasonable_joint_speed = float(rospy.get_param("~max_reasonable_joint_speed", 2.0))
        self.min_valid_joints = int(rospy.get_param("~min_valid_joints", 1))
        self.min_bone_length = float(rospy.get_param("~min_bone_length", 0.01))
        self.max_bone_length = float(rospy.get_param("~max_bone_length", 0.90))
        self.timeout_remove_sec = float(rospy.get_param("~timeout_remove_sec", 0.5))
        self.status_interval_sec = float(rospy.get_param("~status_interval_sec", 1.0))
        self.tracked_joint_ids = _int_list_param("~tracked_joint_ids", DEFAULT_TRACKED_JOINT_IDS)
        self.connection_pairs = _connection_pairs_param("~connection_pairs", DEFAULT_CONNECTION_PAIRS)
        self.delta_threshold = float(rospy.get_param("~delta_threshold", 0.02))
        self.heartbeat_interval_sec = max(
            0.0,
            float(rospy.get_param("~heartbeat_interval_sec", 0.20)),
        )
        self.visualization_marker_lifetime_sec = max(
            0.0,
            float(rospy.get_param("~visualization_marker_lifetime_sec", 1.0)),
        )

        self.prev_skeleton: Optional[SkeletonDict] = None
        self.prev_stamp: Optional[rospy.Time] = None
        self._prev_positions_np: Optional[np.ndarray] = None
        self._prev_stamp_sec: Optional[float] = None
        self.prev_dynamic_padding = 0.0
        self.last_msg_time: Optional[rospy.Time] = None
        self.object_active = False
        self.last_status_text = ""
        self.last_status_time = rospy.Time(0)
        self.last_valid_joint_count = 0
        self.last_collision_object: Optional[CollisionObject] = None
        self.last_publish_time: Optional[rospy.Time] = None

        self.collision_pub = rospy.Publisher(
            self.collision_object_topic,
            CollisionObject,
            queue_size=10,
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher(
            self.visualization_marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(self.skeleton_topic, PoseArray, self.skeleton_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.cleanup_timer)

        rospy.loginfo("skeleton_obstacle_builder started")
        rospy.loginfo("skeleton_topic: %s", self.skeleton_topic)
        rospy.loginfo("collision_object_topic: %s", self.collision_object_topic)
        rospy.loginfo("status_topic: %s", self.status_topic)
        rospy.loginfo("visualization_marker_topic: %s", self.visualization_marker_topic)
        rospy.loginfo("frame_id: %s", self.frame_id)
        rospy.loginfo("object_id: %s", self.object_id)
        rospy.loginfo("tracked_joint_ids: %s", self.tracked_joint_ids)
        rospy.loginfo("connection_pairs: %s", self.connection_pairs)
        rospy.loginfo("arm_joint_radius: %.3f", self.arm_joint_radius)
        rospy.loginfo("arm_bone_radius: %.3f", self.arm_bone_radius)
        rospy.loginfo("delta_threshold: %.3f", self.delta_threshold)

    def decode_pose_array(self, msg: PoseArray) -> SkeletonDict:
        decoded = pose_array_to_numeric_joint_dict(msg, self.tracked_joint_ids)
        return {
            joint_id: decoded[joint_id]
            for joint_id in self.tracked_joint_ids
            if joint_id in decoded
        }

    def publish_collision_object(self, obj: CollisionObject, stamp: rospy.Time) -> None:
        obj.header.stamp = stamp
        self.collision_pub.publish(obj)
        self.marker_pub.publish(self.build_visualization_markers(obj))
        self.last_collision_object = obj
        self.last_publish_time = rospy.Time.now()

    def build_visualization_markers(self, obj: CollisionObject) -> MarkerArray:
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header = obj.header
        delete_all.ns = "human_collision"
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        lifetime = rospy.Duration(self.visualization_marker_lifetime_sec)
        for index, (primitive, pose) in enumerate(zip(obj.primitives, obj.primitive_poses)):
            marker = Marker()
            marker.header = obj.header
            marker.ns = "human_collision"
            marker.id = index
            marker.action = Marker.ADD
            marker.pose = pose
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.90
            marker.lifetime = lifetime

            if primitive.type == SolidPrimitive.SPHERE:
                marker.type = Marker.SPHERE
                diameter = primitive.dimensions[SolidPrimitive.SPHERE_RADIUS] * 2.0
                marker.scale.x = diameter
                marker.scale.y = diameter
                marker.scale.z = diameter
            elif primitive.type == SolidPrimitive.CYLINDER:
                marker.type = Marker.CYLINDER
                marker.scale.x = primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS] * 2.0
                marker.scale.y = primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS] * 2.0
                marker.scale.z = primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT]
            else:
                continue

            marker_array.markers.append(marker)

        return marker_array

    def publish_delete_markers(self, stamp: rospy.Time) -> None:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "human_collision"
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_heartbeat_if_due(self, stamp: rospy.Time) -> bool:
        if self.last_collision_object is None or self.heartbeat_interval_sec <= 0.0:
            return False
        if self.last_publish_time is not None:
            age = (rospy.Time.now() - self.last_publish_time).to_sec()
            if age < self.heartbeat_interval_sec:
                return False
        self.publish_collision_object(self.last_collision_object, stamp)
        return True

    def sanitize_skeleton(self, skeleton: SkeletonDict) -> SkeletonDict:
        cleaned: SkeletonDict = {}
        for joint_id, point in skeleton.items():
            if not is_finite_point(point):
                continue
            cleaned[joint_id] = (float(point[0]), float(point[1]), float(point[2]))

        self.last_valid_joint_count = len(cleaned)
        if len(cleaned) < self.min_valid_joints:
            return {}
        return cleaned

    def _max_joint_delta(self, prev: Optional[SkeletonDict], current: SkeletonDict) -> float:
        if prev is None:
            return float("inf")

        common = set(prev.keys()) & set(current.keys())
        if not common:
            return float("inf")

        max_delta = 0.0
        for joint_id in common:
            max_delta = max(max_delta, point_distance(current[joint_id], prev[joint_id]))
        return max_delta

    def _extract_positions_np(self, skeleton: SkeletonDict) -> np.ndarray:
        return np.array(
            [skeleton.get(joint_id, (0.0, 0.0, 0.0)) for joint_id in self.tracked_joint_ids],
            dtype=np.float64,
        )

    def _compute_speed_vectorized(self, positions_np: np.ndarray, stamp: rospy.Time) -> float:
        if self._prev_positions_np is None or self._prev_stamp_sec is None:
            return 0.0

        dt = max(stamp.to_sec() - self._prev_stamp_sec, 1e-6)
        deltas = positions_np - self._prev_positions_np
        speeds = np.linalg.norm(deltas, axis=1) / dt
        speeds = np.where(speeds > self.max_reasonable_joint_speed, 0.0, speeds)
        return float(speeds.max()) if speeds.size else 0.0

    def compute_dynamic_padding(self, max_speed: float) -> float:
        speed_over_deadband = max(0.0, max_speed - self.speed_padding_deadband)
        dynamic_padding_raw = self.speed_padding_gain * speed_over_deadband
        return min(self.max_dynamic_padding, dynamic_padding_raw)

    def smooth_dynamic_padding(self, dynamic_padding_clamped: float) -> float:
        smoothed = (
            self.padding_smoothing_alpha * self.prev_dynamic_padding
            + (1.0 - self.padding_smoothing_alpha) * dynamic_padding_clamped
        )
        self.prev_dynamic_padding = smoothed
        return smoothed

    def final_joint_radius(self, dynamic_padding: float) -> float:
        return self.body_joint_radius + self.static_safety_padding + dynamic_padding

    def final_bone_radius(self, dynamic_padding: float) -> float:
        return self.body_bone_radius + self.static_safety_padding + dynamic_padding

    def final_arm_joint_radius(self, dynamic_padding: float) -> float:
        return self.arm_joint_radius + self.static_safety_padding + dynamic_padding

    def final_arm_bone_radius(self, dynamic_padding: float) -> float:
        return self.arm_bone_radius + self.static_safety_padding + dynamic_padding

    def make_sphere(self, point: Point3D, radius: float) -> Tuple[SolidPrimitive, Pose]:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [radius]

        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        pose.orientation.w = 1.0
        return primitive, pose

    def make_cylinder_between(
        self,
        p1: Point3D,
        p2: Point3D,
        radius: float,
    ) -> Optional[Tuple[SolidPrimitive, Pose]]:
        height = point_distance(p1, p2)
        if height < self.min_bone_length or height > self.max_bone_length:
            return None

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]

        pose = Pose()
        pose.position.x = (p1[0] + p2[0]) * 0.5
        pose.position.y = (p1[1] + p2[1]) * 0.5
        pose.position.z = (p1[2] + p2[2]) * 0.5

        direction = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
        qx, qy, qz, qw = _quaternion_from_z_axis(direction)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return primitive, pose

    def build_collision_object(
        self,
        skeleton: SkeletonDict,
        stamp: rospy.Time,
        dynamic_padding: float,
    ) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self.frame_id
        obj.header.stamp = stamp
        obj.id = self.object_id
        obj.operation = CollisionObject.ADD

        for joint_id in self.tracked_joint_ids:
            point = skeleton.get(joint_id)
            if point is None:
                continue
            if joint_id in ARM_JOINT_IDS:
                joint_radius = self.final_arm_joint_radius(dynamic_padding)
            else:
                joint_radius = self.final_joint_radius(dynamic_padding)
            primitive, pose = self.make_sphere(point, joint_radius)
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)

        for joint_a, joint_b in self.connection_pairs:
            p1 = skeleton.get(joint_a)
            p2 = skeleton.get(joint_b)
            if p1 is None or p2 is None:
                continue
            if joint_a in ARM_JOINT_IDS or joint_b in ARM_JOINT_IDS:
                bone_radius = self.final_arm_bone_radius(dynamic_padding)
            else:
                bone_radius = self.final_bone_radius(dynamic_padding)
            cylinder = self.make_cylinder_between(p1, p2, bone_radius)
            if cylinder is not None:
                primitive, pose = cylinder
                obj.primitives.append(primitive)
                obj.primitive_poses.append(pose)

        return obj

    def remove_object(self) -> None:
        if not self.object_active:
            return

        obj = CollisionObject()
        obj.header.frame_id = self.frame_id
        obj.header.stamp = rospy.Time.now()
        obj.id = self.object_id
        obj.operation = CollisionObject.REMOVE
        self.collision_pub.publish(obj)
        self.publish_delete_markers(obj.header.stamp)
        self.object_active = False
        self.last_collision_object = None
        self.last_publish_time = rospy.Time.now()

    def publish_status(self, text: str, force: bool = False) -> None:
        now = rospy.Time.now()
        elapsed = (now - self.last_status_time).to_sec() if self.last_status_time != rospy.Time(0) else self.status_interval_sec
        if not force and text == self.last_status_text and elapsed < self.status_interval_sec:
            return

        self.status_pub.publish(String(data=text))
        self.last_status_text = text
        self.last_status_time = now

    def skeleton_callback(self, msg: PoseArray) -> None:
        now = rospy.Time.now()
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else now
        self.last_msg_time = now

        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.publish_status(
                "SKELETON_FRAME_MISMATCH input=%s expected=%s" % (msg.header.frame_id, self.frame_id),
                force=True,
            )
            return

        raw_skeleton = self.decode_pose_array(msg)
        skeleton = self.sanitize_skeleton(raw_skeleton)
        if not skeleton:
            self.remove_object()
            if self.last_valid_joint_count == 0:
                self.publish_status("SKELETON_EMPTY_REMOVE", force=True)
            else:
                self.publish_status(
                    "SKELETON_TOO_FEW_JOINTS valid=%d min=%d"
                    % (self.last_valid_joint_count, self.min_valid_joints),
                    force=True,
                )
            self.prev_skeleton = None
            self.prev_stamp = None
            self._prev_positions_np = None
            self._prev_stamp_sec = None
            self.prev_dynamic_padding = 0.0
            return

        delta = self._max_joint_delta(self.prev_skeleton, skeleton)
        if delta < self.delta_threshold:
            heartbeat = self.publish_heartbeat_if_due(stamp)
            self.publish_status(
                "SKIP_SMALL_DELTA delta=%.3f threshold=%.3f heartbeat=%s"
                % (delta, self.delta_threshold, heartbeat)
            )
            return

        positions_np = self._extract_positions_np(skeleton)
        max_speed = self._compute_speed_vectorized(positions_np, stamp)
        dynamic_padding_clamped = self.compute_dynamic_padding(max_speed)
        dynamic_padding = self.smooth_dynamic_padding(dynamic_padding_clamped)
        padding_state = "PADDING_ACTIVE" if dynamic_padding > 1e-6 else "PADDING_ZERO"

        obj = self.build_collision_object(skeleton, stamp, dynamic_padding)
        if not obj.primitives:
            self.remove_object()
            self.publish_status("SKELETON_EMPTY_REMOVE no_valid_primitives", force=True)
            self.prev_skeleton = None
            self.prev_stamp = None
            self._prev_positions_np = None
            self._prev_stamp_sec = None
            self.prev_dynamic_padding = 0.0
            return

        self.publish_collision_object(obj, stamp)
        self.object_active = True
        self.prev_skeleton = dict(skeleton)
        self.prev_stamp = stamp
        self._prev_positions_np = positions_np.copy()
        self._prev_stamp_sec = stamp.to_sec()

        self.publish_status(
            "OBSTACLE_PUBLISHED %s speed=%.3f padding=%.3f primitives=%d topic=%s"
            % (
                padding_state,
                max_speed,
                dynamic_padding,
                len(obj.primitives),
                self.collision_object_topic,
            ),
            force=padding_state == "PADDING_ACTIVE",
        )

    def cleanup_timer(self, _event: Any) -> None:
        if self.last_msg_time is None:
            return

        age = (rospy.Time.now() - self.last_msg_time).to_sec()
        if age <= self.timeout_remove_sec:
            return

        if self.object_active:
            self.remove_object()
            self.publish_status(
                "OBSTACLE_TIMEOUT_REMOVE timeout=%.2fs topic=%s"
                % (age, self.collision_object_topic),
                force=True,
            )

        self.prev_skeleton = None
        self.prev_stamp = None
        self._prev_positions_np = None
        self._prev_stamp_sec = None
        self.prev_dynamic_padding = 0.0


if __name__ == "__main__":
    SkeletonObstacleBuilder()
    rospy.spin()
