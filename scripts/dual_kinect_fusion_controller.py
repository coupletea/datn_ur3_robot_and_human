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
