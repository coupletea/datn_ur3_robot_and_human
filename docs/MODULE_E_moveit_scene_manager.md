# MODULE E — `moveit_scene_manager.py`

> **Node name:** `moveit_scene_manager`
> **Vai trò:** Nhận `CollisionObject` từ `skeleton_obstacle_builder`, apply vào MoveIt planning scene, publish trạng thái scene cho planner.

---

## Tổng quan pipeline

```
/human_collision_object (CollisionObject)
    │
    ▼ collision_object_callback()
validate_collision_object()
    │
    ├─ op=ADD/MOVE:
    │    gắn ObjectColor đỏ
    │    apply_collision_object()  → /apply_planning_scene service
    │    publish: SCENE_OBJECT_APPLIED / SCENE_OBJECT_CONFIRMED
    │
    └─ op=REMOVE:
         remove_object()
         publish: SCENE_OBJECT_REMOVED / SCENE_OBJECT_CONFIRMED_REMOVED

/moveit_scene_status (String)
```

---

## Class: `MoveItSceneManager`

### Nhóm: Khởi tạo

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load ROS params, khởi tạo `/apply_planning_scene`, subscriber `/human_collision_object`, publish `/moveit_scene_status`; `/get_planning_scene` chỉ dùng khi ACM debug |

### Nhóm: Xử lý CollisionObject

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `collision_object_callback(msg)` | `CollisionObject` message | Callback chính: validate → gọi apply hoặc remove theo `msg.operation` |
| `validate_collision_object(obj) → (ok, reason)` | `CollisionObject` | Kiểm tra object hợp lệ: primitives khớp poses, quaternion chuẩn hóa, ID không rỗng |
| `apply_collision_object(obj)` | `CollisionObject` | Gọi `/apply_planning_scene` service để áp dụng object vào scene |
| `build_planning_scene_diff(obj)` | `CollisionObject` | Tạo scene diff; ADD được gắn `ObjectColor` đỏ cho RViz |
| `remove_object(object_id)` | str | Tạo CollisionObject với `operation=REMOVE`, gọi `/apply_planning_scene` |

### Nhóm: Xác nhận Scene

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `get_scene_object(object_id) → Optional[CollisionObject]` | str | Lấy CollisionObject hiện tại từ scene qua `/get_planning_scene` service |
| `confirm_object_in_scene(object_id, expected_present) → bool` | str, bool | Helper giữ lại để debug thủ công, không còn được gọi trong pipeline chính |

### Nhóm: Dọn dẹp & Status

| Phương thức | Chức năng |
|-------------|-----------|
| `cleanup_stale_objects()` | Dọn các object quá `max_object_age_sec` (timer callback) |
| `publish_status(text, force)` | Publish `/moveit_scene_status` với throttling; `force=True` bỏ qua throttle |

---

## Trạng thái Scene (Status Keywords)

| Keyword | Ý nghĩa |
|---------|---------|
| `SCENE_OBJECT_APPLIED` | CollisionObject đã được gửi apply vào scene |
| `SCENE_OBJECT_CONFIRMED` | Status phát ngay sau khi `/apply_planning_scene` trả success cho ADD |
| `SCENE_OBJECT_REMOVED` | Lệnh REMOVE đã gửi |
| `SCENE_OBJECT_CONFIRMED_REMOVED` | Status phát ngay sau khi `/apply_planning_scene` trả success cho REMOVE |
| `SCENE_EMPTY` | Scene không có collision object người |
| `SCENE_READY` | Scene khởi động xong, sẵn sàng nhận object |
| `SCENE_ERROR` | Lỗi khi apply hoặc validate |

> **Planner** (`planner_ab_replan_node`) đọc topic `/moveit_scene_status` và chỉ cho phép lập kế hoạch khi status chứa `SCENE_OBJECT_CONFIRMED` hoặc `SCENE_EMPTY`.

---

## Cấu hình — ROS Parameters

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~object_color_rgba` | `1.0,0.0,0.0,0.90` | Màu CollisionObject trong MoveIt PlanningScene/RViz |
| `~max_object_age_sec` | `1.0` | Object cũ hơn ngưỡng bị reject |
| `~remove_timeout_sec` | `0.5` | Timeout không nhận object mới trước khi REMOVE; launch override |
| `~status_interval_sec` | `1.0` | Status throttle |

---

## ROS Topics & Services

| Tên | Type | Chiều | Mô tả |
|-----|------|-------|-------|
| `/human_collision_object` | `moveit_msgs/CollisionObject` | Subscribe | Collision object đầu vào |
| `/moveit_scene_status` | `std_msgs/String` | Publish | Trạng thái scene |
| `/apply_planning_scene` | `moveit_msgs/ApplyPlanningScene` | Service Client | Apply CollisionObject vào scene |
| `/get_planning_scene` | `moveit_msgs/GetPlanningScene` | Service Client | Chỉ dùng khi gọi helper confirm/debug thủ công hoặc ACM debug |

## Ghi chú thay đổi blocker

- Confirm loop qua `/get_planning_scene` không còn chạy trong ADD/REMOVE.
- `SCENE_OBJECT_CONFIRMED` và `SCENE_OBJECT_CONFIRMED_REMOVED` nghĩa là service apply thành công, không phải scene đã được verify lại. Rủi ro: nếu MoveIt apply fail âm thầm sau response success, planner vẫn xem scene sẵn sàng.
- Module D heartbeat object cũ mỗi `0.20s` khi skeleton đứng yên để scene không bị remove do delta gate.
- Single launch dùng `remove_timeout_sec=2.0s`; Dual launch dùng `1.0s`. Timeout chỉ là fallback khi builder ngừng publish.
- ADD scene diff gắn `ObjectColor` đỏ; PlanningScene RViz chỉ hiện object khi skeleton hợp lệ và scene chưa REMOVE.

---

*Xem thêm: [MODULE_D_skeleton_obstacle_builder.md](MODULE_D_skeleton_obstacle_builder.md) — tạo CollisionObject input.*
*Xem thêm: [MODULE_F_planner_ab_replan_node.md](MODULE_F_planner_ab_replan_node.md) — đọc scene status để quyết định có plan không.*
