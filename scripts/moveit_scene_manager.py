#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Any, Iterable, List, Optional, Tuple

import rospy
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import (
    ApplyPlanningScene,
    ApplyPlanningSceneRequest,
    GetPlanningScene,
    GetPlanningSceneRequest,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, String


OK = "OK"


def _string_list_param(name: str, default: Iterable[str]) -> List[str]:
    raw_value = rospy.get_param(name, ",".join(default))
    if isinstance(raw_value, str):
        items = [item.strip() for item in raw_value.split(",") if item.strip()]
    elif isinstance(raw_value, (list, tuple)):
        items = [str(item).strip() for item in raw_value if str(item).strip()]
    else:
        items = list(default)
    return items if items else list(default)


def _color_rgba_param(name: str, default: Tuple[float, float, float, float]) -> ColorRGBA:
    raw_value = rospy.get_param(name, ",".join(str(value) for value in default))
    if isinstance(raw_value, str):
        items = [item.strip() for item in raw_value.split(",") if item.strip()]
    elif isinstance(raw_value, (list, tuple)):
        items = list(raw_value)
    else:
        items = []

    try:
        values = [float(value) for value in items]
    except (TypeError, ValueError):
        values = list(default)
    if len(values) != 4:
        values = list(default)
    values = [max(0.0, min(1.0, value)) for value in values]
    return ColorRGBA(r=values[0], g=values[1], b=values[2], a=values[3])


def is_finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _now() -> rospy.Time:
    try:
        return rospy.Time.now()
    except rospy.ROSInitException:
        return rospy.Time(0)


def primitive_type_name(primitive_type: int) -> str:
    names = {
        SolidPrimitive.BOX: "BOX",
        SolidPrimitive.SPHERE: "SPHERE",
        SolidPrimitive.CYLINDER: "CYLINDER",
        SolidPrimitive.CONE: "CONE",
    }
    return names.get(primitive_type, "UNKNOWN_%s" % primitive_type)


class MoveItSceneManager:
    def __init__(self) -> None:
        self.human_collision_object_topic = rospy.get_param(
            "~human_collision_object_topic",
            "/human_collision_object",
        )
        self.moveit_scene_status_topic = rospy.get_param(
            "~moveit_scene_status_topic",
            "/moveit_scene_status",
        )
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.object_id = rospy.get_param("~object_id", "human_skeleton")
        self.apply_planning_scene_service = rospy.get_param(
            "~apply_planning_scene_service",
            "/apply_planning_scene",
        )
        self.get_planning_scene_service = rospy.get_param(
            "~get_planning_scene_service",
            "/get_planning_scene",
        )
        self.wait_for_service_timeout_sec = float(rospy.get_param("~wait_for_service_timeout_sec", 10.0))
        self.remove_timeout_sec = float(rospy.get_param("~remove_timeout_sec", 0.5))
        self.status_interval_sec = float(rospy.get_param("~status_interval_sec", 1.0))
        self.max_collision_primitives = int(rospy.get_param("~max_collision_primitives", 120))
        self.max_object_age_sec = float(rospy.get_param("~max_object_age_sec", 1.0))
        self.enable_acm_debug = bool(rospy.get_param("~enable_acm_debug", False))
        self.allow_empty_add_object = bool(rospy.get_param("~allow_empty_add_object", False))
        self.allowed_primitive_types = [
            item.upper() for item in _string_list_param("~allowed_primitive_types", ["SPHERE", "CYLINDER", "BOX"])
        ]
        self.min_apply_interval_sec = float(rospy.get_param("~min_apply_interval_sec", 0.02))
        self.reject_bad_quaternion = bool(rospy.get_param("~reject_bad_quaternion", True))
        self.remove_on_shutdown = bool(rospy.get_param("~remove_on_shutdown", True))
        self.object_color = _color_rgba_param("~object_color_rgba", (1.0, 0.0, 0.0, 0.90))

        self.object_active = False
        self.last_object_time: Optional[rospy.Time] = None
        self.last_apply_time: Optional[rospy.Time] = None
        self.last_valid_object: Optional[CollisionObject] = None
        self.last_status_text = ""
        self.last_status_time = rospy.Time(0)

        self.apply_scene_srv: Optional[rospy.ServiceProxy] = None
        self.get_scene_srv: Optional[rospy.ServiceProxy] = None

        self.status_pub = rospy.Publisher(
            self.moveit_scene_status_topic,
            String,
            queue_size=10,
            latch=True,
        )

        self.wait_for_moveit_services()

        self.object_sub = rospy.Subscriber(
            self.human_collision_object_topic,
            CollisionObject,
            self.collision_object_callback,
            queue_size=1,
        )
        self.cleanup_timer_handle = rospy.Timer(rospy.Duration(0.1), self.cleanup_timer)

        rospy.loginfo("moveit_scene_manager started")
        rospy.loginfo("human_collision_object_topic: %s", self.human_collision_object_topic)
        rospy.loginfo("moveit_scene_status_topic: %s", self.moveit_scene_status_topic)
        rospy.loginfo("target_frame: %s", self.target_frame)
        rospy.loginfo("object_id: %s", self.object_id)

    def pose_is_finite(self, pose: Any) -> bool:
        fields = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not all(is_finite_number(value) for value in fields):
            return False

        if not self.reject_bad_quaternion:
            return True

        norm = math.sqrt(
            pose.orientation.x * pose.orientation.x
            + pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z
            + pose.orientation.w * pose.orientation.w
        )
        return math.isfinite(norm) and norm > 1e-6

    def primitive_dimensions_are_valid(self, primitive: SolidPrimitive) -> bool:
        dims = list(primitive.dimensions)
        required_dims = {
            SolidPrimitive.SPHERE: 1,
            SolidPrimitive.CYLINDER: 2,
            SolidPrimitive.BOX: 3,
            SolidPrimitive.CONE: 2,
        }.get(primitive.type)

        if required_dims is None or len(dims) < required_dims:
            return False

        for dim in dims[:required_dims]:
            if not is_finite_number(dim) or float(dim) <= 0.0:
                return False

        return True

    def object_age_sec(self, obj: CollisionObject) -> Optional[float]:
        stamp = obj.header.stamp
        if stamp == rospy.Time(0):
            return None
        now = _now()
        if now == rospy.Time(0):
            return None
        return (now - stamp).to_sec()

    def validate_object_identity(self, obj: CollisionObject) -> Tuple[bool, str]:
        if obj.id != self.object_id:
            return False, "bad_object_id expected=%s actual=%s" % (self.object_id, obj.id)
        return True, OK

    def validate_object_frame(self, obj: CollisionObject) -> Tuple[bool, str]:
        if not obj.header.frame_id:
            return False, "missing_frame"
        if obj.header.frame_id != self.target_frame:
            return False, "bad_frame expected=%s actual=%s" % (self.target_frame, obj.header.frame_id)
        return True, OK

    def validate_object_operation(self, obj: CollisionObject) -> Tuple[bool, str]:
        if obj.operation not in (CollisionObject.ADD, CollisionObject.REMOVE):
            return False, "bad_operation value=%s" % obj.operation
        return True, OK

    def validate_object_primitives(self, obj: CollisionObject) -> Tuple[bool, str]:
        if obj.operation == CollisionObject.REMOVE:
            return True, OK

        primitive_count = len(obj.primitives)
        pose_count = len(obj.primitive_poses)

        if primitive_count == 0 and not self.allow_empty_add_object:
            return False, "empty_add_object"
        if primitive_count != pose_count:
            return False, "primitive_pose_count_mismatch primitives=%d poses=%d" % (primitive_count, pose_count)
        if primitive_count > self.max_collision_primitives:
            return False, "too_many_primitives count=%d max=%d" % (
                primitive_count,
                self.max_collision_primitives,
            )

        for index, primitive in enumerate(obj.primitives):
            type_name = primitive_type_name(primitive.type)
            if type_name not in self.allowed_primitive_types:
                return False, "primitive_type_not_allowed index=%d type=%s" % (index, type_name)
            if not self.primitive_dimensions_are_valid(primitive):
                return False, "bad_primitive_dimensions index=%d type=%s" % (index, type_name)
            if not self.pose_is_finite(obj.primitive_poses[index]):
                return False, "bad_primitive_pose index=%d" % index

        return True, OK

    def validate_collision_object(self, obj: CollisionObject) -> Tuple[bool, str]:
        checks = [
            self.validate_object_identity(obj),
            self.validate_object_frame(obj),
            self.validate_object_operation(obj),
            self.validate_object_primitives(obj),
        ]
        for ok, reason in checks:
            if not ok:
                return ok, reason

        age = self.object_age_sec(obj)
        if age is not None and age > self.max_object_age_sec:
            return False, "stale_object age=%.3f max=%.3f" % (age, self.max_object_age_sec)

        return True, OK

    def build_planning_scene_diff(self, obj: CollisionObject) -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        if obj.operation == CollisionObject.ADD:
            scene.object_colors.append(ObjectColor(id=obj.id, color=self.object_color))
        return scene

    def apply_scene_diff(self, scene: PlanningScene) -> Tuple[bool, str]:
        if self.apply_scene_srv is None:
            self.apply_scene_srv = rospy.ServiceProxy(
                self.apply_planning_scene_service,
                ApplyPlanningScene,
            )

        request = ApplyPlanningSceneRequest()
        request.scene = scene

        try:
            response = self.apply_scene_srv(request)
        except rospy.ServiceException as exc:
            return False, "service_exception %s" % exc
        except rospy.ROSException as exc:
            return False, "ros_exception %s" % exc

        if not response.success:
            return False, "service_returned_false"

        self.last_apply_time = _now()
        return True, OK

    def apply_collision_object(self, obj: CollisionObject) -> bool:
        valid, reason = self.validate_collision_object(obj)
        if not valid:
            self.publish_status("SCENE_OBJECT_REJECTED reason=%s" % reason, force=True)
            return False

        scene = self.build_planning_scene_diff(obj)
        apply_ok, apply_reason = self.apply_scene_diff(scene)
        if not apply_ok:
            self.publish_status("SCENE_APPLY_FAILED reason=%s" % apply_reason, force=True)
            return False

        self.object_active = obj.operation == CollisionObject.ADD
        self.last_valid_object = obj
        self.last_object_time = _now()

        self.publish_status(
            "SCENE_OBJECT_APPLIED id=%s primitives=%d"
            % (obj.id, len(obj.primitives)),
            force=True,
        )

        self.publish_status("SCENE_OBJECT_CONFIRMED id=%s" % obj.id, force=True)

        return True

    def build_remove_collision_object(self, reason: str) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self.target_frame
        obj.header.stamp = _now()
        obj.id = self.object_id
        obj.operation = CollisionObject.REMOVE
        return obj

    def remove_object(self, reason: str) -> bool:
        obj = self.build_remove_collision_object(reason)
        scene = self.build_planning_scene_diff(obj)
        ok, apply_reason = self.apply_scene_diff(scene)
        if not ok:
            self.publish_status(
                "SCENE_APPLY_FAILED reason=remove_%s %s" % (reason, apply_reason),
                force=True,
            )
            return False

        self.object_active = False
        self.last_valid_object = None
        self.last_object_time = None
        self.publish_status("SCENE_OBJECT_REMOVED reason=%s" % reason, force=True)

        self.publish_status("SCENE_OBJECT_CONFIRMED_REMOVED id=%s" % self.object_id, force=True)

        return True

    def request_planning_scene(self) -> Optional[PlanningScene]:
        if self.get_scene_srv is None:
            self.get_scene_srv = rospy.ServiceProxy(
                self.get_planning_scene_service,
                GetPlanningScene,
            )

        request = GetPlanningSceneRequest()
        request.components = PlanningSceneComponents()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        if self.enable_acm_debug:
            request.components.components |= PlanningSceneComponents.ALLOWED_COLLISION_MATRIX

        try:
            response = self.get_scene_srv(request)
        except rospy.ServiceException as exc:
            rospy.logwarn("Failed to request planning scene: %s", exc)
            return None
        except rospy.ROSException as exc:
            rospy.logwarn("Failed to request planning scene: %s", exc)
            return None

        return response.scene

    def extract_world_object_ids(self, scene: PlanningScene) -> List[str]:
        return [obj.id for obj in scene.world.collision_objects]

    def confirm_object_in_scene(self, object_id: str, expected_present: bool = True) -> bool:
        scene = self.request_planning_scene()
        if scene is None:
            return False

        if self.enable_acm_debug:
            self.log_acm_summary(scene)

        ids = self.extract_world_object_ids(scene)
        found = object_id in ids
        return found == expected_present

    def log_acm_summary(self, scene: PlanningScene) -> None:
        acm = scene.allowed_collision_matrix
        entries = len(acm.entry_names)
        preview = ",".join(acm.entry_names[:10])
        rospy.loginfo("SCENE_ACM_DEBUG entries=%d preview=%s", entries, preview)
        self.publish_status("SCENE_ACM_DEBUG entries=%d" % entries)

    def can_apply_now(self) -> bool:
        if self.last_apply_time is None:
            return True

        now = _now()
        elapsed = (now - self.last_apply_time).to_sec()
        return elapsed >= self.min_apply_interval_sec

    def publish_status(self, text: str, force: bool = False) -> None:
        now = _now()
        elapsed = (now - self.last_status_time).to_sec() if self.last_status_time != rospy.Time(0) else self.status_interval_sec
        if not force and text == self.last_status_text and elapsed < self.status_interval_sec:
            return

        self.status_pub.publish(String(data=text))
        self.last_status_text = text
        self.last_status_time = now

    def collision_object_callback(self, msg: CollisionObject) -> None:
        valid, reason = self.validate_collision_object(msg)
        if not valid:
            self.publish_status("SCENE_OBJECT_REJECTED reason=%s" % reason, force=True)
            return

        if msg.operation == CollisionObject.REMOVE:
            self.remove_object(reason="explicit_remove")
            return

        self.last_object_time = _now()
        self.last_valid_object = msg

        if not self.can_apply_now():
            self.publish_status("SCENE_SKIP_RATE_LIMIT")
            return

        self.apply_collision_object(msg)

    def cleanup_timer(self, _event: Any) -> None:
        if not self.object_active or self.last_object_time is None:
            return

        elapsed = (_now() - self.last_object_time).to_sec()
        if elapsed <= self.remove_timeout_sec:
            return

        self.remove_object(reason="timeout")

    def wait_for_moveit_services(self) -> bool:
        apply_ready = False
        try:
            rospy.wait_for_service(
                self.apply_planning_scene_service,
                timeout=self.wait_for_service_timeout_sec,
            )
            self.apply_scene_srv = rospy.ServiceProxy(
                self.apply_planning_scene_service,
                ApplyPlanningScene,
            )
            apply_ready = True
            self.publish_status("SCENE_SERVICE_READY", force=True)
            rospy.loginfo("Connected to %s", self.apply_planning_scene_service)
        except rospy.ROSException as exc:
            self.apply_scene_srv = None
            self.publish_status("SCENE_SERVICE_NOT_READY reason=%s" % exc, force=True)
            rospy.logwarn("MoveIt apply planning scene service is not ready: %s", exc)

        if self.enable_acm_debug:
            try:
                rospy.wait_for_service(
                    self.get_planning_scene_service,
                    timeout=self.wait_for_service_timeout_sec,
                )
                self.get_scene_srv = rospy.ServiceProxy(
                    self.get_planning_scene_service,
                    GetPlanningScene,
                )
                rospy.loginfo("Connected to %s", self.get_planning_scene_service)
            except rospy.ROSException as exc:
                self.get_scene_srv = None
                rospy.logwarn("MoveIt get planning scene service is not ready: %s", exc)

        return apply_ready

    def shutdown_hook(self) -> None:
        if not self.remove_on_shutdown or not self.object_active:
            return

        try:
            self.remove_object(reason="shutdown")
        except Exception as exc:  # noqa: BLE001 - shutdown should never crash ROS teardown
            rospy.logwarn("Failed to remove human object on shutdown: %s", exc)


def main() -> None:
    rospy.init_node("moveit_scene_manager")
    manager = MoveItSceneManager()
    rospy.on_shutdown(manager.shutdown_hook)
    rospy.spin()


if __name__ == "__main__":
    main()
