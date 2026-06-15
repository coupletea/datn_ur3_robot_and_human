# MODULE D — `skeleton_obstacle_builder.py`

> **Node name:** `skeleton_obstacle_builder`
> **Vai trò:** Nhận skeleton `PoseArray`, xây dựng `CollisionObject` cho MoveIt bằng body joints/bones.

---

## Tổng quan pipeline

```
/human_skeleton_base (PoseArray)
    │
    ▼ skeleton_callback()
last_msg_time = now()  ← cập nhật trước decode/sanitize/skip
    │
decode_pose_array()
    │
sanitize_skeleton()
    │
_max_joint_delta(prev, current)
    ├─ delta < ~delta_threshold → SKIP_SMALL_DELTA + heartbeat object cũ
    └─ delta >= threshold       → rebuild
    │
_extract_positions_np() → _compute_speed_vectorized()
    │
compute_dynamic_padding() → smooth_dynamic_padding()
    │
    └─ make_sphere() + make_cylinder_between()
    │
publish /human_collision_object (CollisionObject ADD)
publish /human_collision_markers (MarkerArray đỏ, cùng geometry)
```

---

## Class: `SkeletonObstacleBuilder`

### Nhóm: Khởi tạo

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load ROS params, khởi tạo publisher, subscriber `/human_skeleton_base`, cleanup timer |

### Nhóm: Parse, Validate, Skip

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `decode_pose_array(msg) → SkeletonDict` | `PoseArray` | Decode canonical 51/9-pose schema rồi chọn `tracked_joint_ids` |
| `sanitize_skeleton(skeleton) → SkeletonDict` | skeleton dict | Loại bỏ NaN/Inf, kiểm tra ≥ `min_valid_joints` |
| `_max_joint_delta(prev, current) → float` | previous/current skeleton | Max displacement giữa common joints; `prev=None` hoặc không có common joint → `inf` |

### Nhóm: Dynamic Safety Padding

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `_extract_positions_np(skeleton) → np.ndarray` | skeleton dict | Tạo mảng `(N,3)` theo `tracked_joint_ids` |
| `_compute_speed_vectorized(positions_np, stamp) → float` | NumPy positions, ROS stamp | Tính max joint speed vectorized, bỏ speed > `max_reasonable_joint_speed` |
| `compute_dynamic_padding(max_speed) → float` | float (m/s) | Tính padding thô: `gain × max(0, speed - deadband)`, clamp tới `max_dynamic_padding` |
| `smooth_dynamic_padding(padding) → float` | float (m) | EMA smoothing cho padding động |
| `final_joint_radius(dynamic_padding) → float` | float (m) | Body joint radius cuối: `body_joint_radius + static_safety_padding + dynamic_padding` |
| `final_bone_radius(dynamic_padding) → float` | float (m) | Body bone radius cuối: `body_bone_radius + static_safety_padding + dynamic_padding` |
| `final_arm_joint_radius(dynamic_padding) → float` | float (m) | Arm joint radius cuối: `arm_joint_radius + static_safety_padding + dynamic_padding` |
| `final_arm_bone_radius(dynamic_padding) → float` | float (m) | Arm bone radius cuối: `arm_bone_radius + static_safety_padding + dynamic_padding` |

**Công thức Dynamic Padding:**
```
max_speed (m/s)
    │
    ▼
speed_over_deadband = max(0, speed - deadband)
    │
    ▼
dynamic_padding_raw = gain × speed_over_deadband
    │
    ▼
dynamic_padding_clamped = min(max_dynamic_padding, raw)
    │  EMA alpha = padding_smoothing_alpha
    ▼
dynamic_padding_smooth
    │
    ▼
final_radius = body_radius + static_safety_padding + dynamic_padding_smooth
```

### Nhóm: Tạo Collision Primitives

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `make_sphere(point, radius) → (SolidPrimitive, Pose)` | tọa độ 3D, bán kính | Tạo primitive hình cầu |
| `make_cylinder_between(p1, p2, radius) → Optional` | 2 điểm, bán kính | Tạo cylinder giữa 2 điểm, căn hướng theo trục p1→p2 |

### Nhóm: Build & Publish

| Phương thức | Chức năng |
|-------------|-----------|
| `build_collision_object(skeleton, stamp, padding) → CollisionObject` | Duyệt body joints + connection pairs, append primitives trực tiếp, không giới hạn primitive count |
| `build_visualization_markers(obj) → MarkerArray` | Chuyển sphere/cylinder CollisionObject thành marker đỏ cùng pose/kích thước |
| `publish_delete_markers(stamp)` | Xóa marker RViz khi CollisionObject bị REMOVE |
| `remove_object()` | Gửi `CollisionObject` với `operation=REMOVE` |
| `publish_status(text, force)` | Publish `/human_obstacle_status` với throttling |
| `skeleton_callback(msg)` | Callback chính: update timestamp, validate, delta gate, tính speed/padding, build/publish |
| `cleanup_timer(event)` | Nếu không nhận skeleton quá `timeout_remove_sec` → gửi REMOVE và reset state |

---

## Hình học

Mỗi joint hợp lệ tạo đúng một `SPHERE`; mỗi connection pair hợp lệ tạo đúng một `CYLINDER`.

---

## Cấu hình — ROS Parameters

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~skeleton_topic` | `/human_skeleton_base` | Skeleton input |
| `~collision_object_topic` | `/human_collision_object` | CollisionObject output |
| `~status_topic` | `/human_obstacle_status` | Status output |
| `~visualization_marker_topic` | `/human_collision_markers` | MarkerArray đỏ hiển thị đúng collision geometry trong RViz |
| `~visualization_marker_lifetime_sec` | `1.0` | Marker tự hết hạn nếu builder ngừng heartbeat |
| `~frame_id` | `base_link` | Frame của CollisionObject |
| `~object_id` | `human_skeleton` | ID object trong MoveIt scene |
| `~tracked_joint_ids` | `[0,11,12,13,14,15,16,23,24]` | Body joints dùng để build obstacles |
| `~connection_pairs` | 8 body bones | Cặp joint tạo bone obstacles |
| `~joint_radius` | `0.03` | Fallback joint radius |
| `~bone_radius` | `0.03` | Fallback bone radius |
| `~body_joint_radius` | `~joint_radius` | Body joint radius (torso, hips, shoulders) |
| `~body_bone_radius` | `~bone_radius` | Body bone radius (torso, hip bones) |
| `~arm_joint_radius` | `~body_joint_radius` | Radius riêng cho arm joints (elbows 13,14 + wrists 15,16) |
| `~arm_bone_radius` | `~body_bone_radius` | Radius riêng cho arm bones (upper arm + forearm cylinders) |
| `~static_safety_padding` | `0.0` | Padding tĩnh body |
| `~speed_padding_gain` | `0.0` | Dynamic padding gain |
| `~speed_padding_deadband` | `0.05` | Deadband tốc độ (m/s) |
| `~max_dynamic_padding` | `0.0` | Dynamic padding max |
| `~padding_smoothing_alpha` | `0.70` | EMA alpha |
| `~max_reasonable_joint_speed` | `2.0` | Speed lớn hơn ngưỡng bị bỏ |
| `~delta_threshold` | `0.02` | Rebuild only khi max joint delta ≥ threshold (m) |
| `~heartbeat_interval_sec` | `0.20` | Republish object cũ khi delta skip để giữ MoveIt scene fresh |
| `~min_valid_joints` | `1` | Số joint hợp lệ tối thiểu |
| `~min_bone_length` | `0.01` | Bone ngắn hơn bị bỏ |
| `~max_bone_length` | `0.90` | Bone dài hơn bị bỏ |
| `~timeout_remove_sec` | `0.5` | Timeout không nhận skeleton → REMOVE |
| `~status_interval_sec` | `1.0` | Status throttle |

**Params đã xóa khỏi Module D:**

| Tham số | Lý do |
|---------|------|
| `~hand_joint_radius` | Module D không tạo hand collision primitives nữa |
| `~hand_bone_radius` | Module D không tạo hand collision primitives nữa |
| `~hand_static_safety_padding` | Module D không tạo hand collision primitives nữa |
| `~max_primitives_per_object` | Primitive cap bị bỏ; build trực tiếp toàn bộ primitives |
| `~min_joint_delta_to_publish` | Thay bằng `~delta_threshold` |
| `~min_padding_delta_to_publish` | Không còn publish gate theo padding |
| Alternate box geometry params | Builder chỉ tạo sphere joints và cylinder bones |

---

## ROS Topics

| Topic | Type | Chiều | Mô tả |
|-------|------|-------|-------|
| `/human_skeleton_base` | `PoseArray` | Subscribe | Skeleton đầu vào từ tracker/fusion |
| `/human_collision_object` | `moveit_msgs/CollisionObject` | Publish | Collision object cho MoveIt scene |
| `/human_collision_markers` | `visualization_msgs/MarkerArray` | Publish | Marker đỏ khớp sphere/cylinder CollisionObject |
| `/human_obstacle_status` | `String` | Publish | Trạng thái builder |

---

## Safety & Limitations

- `last_msg_time` update trước decode/sanitize/delta skip, nên repeated frames không kích hoạt cleanup timeout sai.
- Delta skip giữ geometry cũ và heartbeat CollisionObject mỗi `heartbeat_interval_sec`.
- Heartbeat ngăn scene manager timeout-remove khi người đứng yên.
- Marker đỏ dùng trực tiếp primitive pose/dimensions của CollisionObject; không tạo obstacle geometry khác.
- Marker có lifetime và nhận `DELETEALL` khi object bị REMOVE, tránh giữ hình cũ khi mất skeleton.
- PoseArray decode canonical trước khi chọn subset, tránh tráo joint giữa tracker và builder.
- Single và Dual launch dùng sphere/cylinder radius cố định `0.05m`; static/dynamic padding đặt `0.0`.
- Hand detection/tracking upstream không đổi, nhưng Module D chỉ dùng body joints/bones để tạo MoveIt obstacle.
- Không còn primitive cap trong Module D; `moveit_scene_manager` vẫn có lớp validate/guard riêng cho CollisionObject đầu vào.
- Nếu `delta_threshold` quá cao, chuyển động chậm có thể không rebuild kịp; default `0.02m` cần tune theo setup thật.

---

## Verification

```bash
python3 -m py_compile scripts/skeleton_obstacle_builder.py
python3 test/test_collision_visualization.py -v
rostopic echo -n 1 /human_collision_markers
rg "hand_joint_radius|hand_bone_radius|hand_static_safety_padding|min_joint_delta_to_publish|min_padding_delta_to_publish" launch/system.launch launch/dual_kinect_system.launch
```

*Xem thêm: [MODULE_E_moveit_scene_manager.md](MODULE_E_moveit_scene_manager.md) — nhận CollisionObject từ module này.*
