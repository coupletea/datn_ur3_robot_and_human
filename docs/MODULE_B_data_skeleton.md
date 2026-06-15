# MODULE B — `data_skeleton.py`

> **Loại:** Thư viện tiện ích (không phải ROS node)
> **Vai trò:** Cung cấp tất cả hàm xử lý camera, depth map, YOLO, MediaPipe, skeleton. Được import bởi `kinect_skeleton_tracker.py`.

---

## Tổng quan

Module này **không chạy độc lập** — là tập hợp hàm thuần Python phục vụ pipeline nhận diện skeleton:

```
data_skeleton.py cung cấp:
  ├── Camera I/O        → initialize_camera, read_registered_frames, read_color_frame_only
  ├── YOLO              → load_yolo_segment_model, run_yolo_person_segmentation, select_best_person_mask
  ├── MediaPipe         → load_mediapipe_holistic, run_holistic
  ├── Landmark 3D       → extract_pose_landmarks_3d, extract_hand_landmarks_3d
  ├── Validate          → validate_body_geometry, validate_hand_geometry
  ├── Merge             → attach_hands_to_pose_wrists, merge_pose_and_hands
  ├── Encode/Decode     → skeleton_dict_to_pose_array
  ├── Visualize         → draw_filtered_skeleton
  └── Constants         → LANDMARK_ORDER, BODY_LANDMARK_ORDER, KinectFrameTimeout
```

---

## Nhóm: Camera I/O (libfreenect2)

| Hàm | Tham số | Chức năng |
|-----|---------|-----------|
| `initialize_camera(device_index, serial, packet_pipeline, color_only)` | index hoặc serial, loại pipeline (CPU/OpenGL/CUDA), cờ color-only | Mở Kinect qua libfreenect2. RGB-D trả `(freenect2, device, listener, registration)`, color-only trả `registration=None` |
| `read_registered_frames(listener, registration, timeout_ms)` | listener, registration object, timeout (ms) | Đọc 1 cặp frame RGB-D đã đăng ký (aligned). Raise `KinectFrameTimeout` nếu quá hạn |
| `read_color_frame_only(listener, timeout_ms)` | listener color-only, timeout (ms) | Đọc 1 color frame, trả `(color_bgr, color_rgb)`. Dùng cho RGB-only mode, không đọc depth |

**Sensor mode note:**
- RGB-D giữ listener `FrameType.Color | FrameType.Depth` và tạo `Registration`.
- RGB-only chỉ đăng ký `FrameType.Color`, không tạo depth registration.
- Depth-only không hỗ trợ trong v1 vì pipeline skeleton hiện tại cần RGB cho MediaPipe Holistic.

**Exception:**
```python
class KinectFrameTimeout(Exception):
    """Raise khi listener.waitForNewFrame() vượt timeout_ms."""
```

---

## Nhóm: YOLO Person Segmentation

| Hàm | Tham số | Chức năng |
|-----|---------|-----------|
| `load_yolo_segment_model(model_path, device)` | đường dẫn `.pt`, `"cpu"/"cuda"` | Load model YOLOv8-seg từ file |
| `run_yolo_person_segmentation(model, rgb, conf, iou, class_id)` | model, ảnh RGB, conf threshold, iou threshold, class id (0=person) | Chạy inference YOLO. Trả về list `{mask, bbox, conf, area}` |
| `select_best_person_mask(detections, shape, min_area_ratio, prefer_center, select_mode, depth_map, min_depth_px)` | detections, shape, area gate, mode, aligned depth map, minimum depth pixels | `legacy` giữ score cũ; `nearest` chọn median depth nhỏ nhất, tie/depth lỗi fallback legacy |
| `dilate_mask(mask, px)` | mask binary, số pixel | Giãn nở mask bằng morphological dilation (mở rộng vùng người) |

`nearest` đo depth trên raw mask chưa dilate, chỉ dùng pixel hữu hạn và `> 0`. Hàm không mutate mask đầu vào. `depth_map` thiếu/sai shape hoặc không có candidate đáng tin sẽ fallback `legacy`.

---

## Nhóm: MediaPipe Holistic

| Hàm | Tham số | Chức năng |
|-----|---------|-----------|
| `load_mediapipe_holistic(model_complexity, min_detection_confidence, min_tracking_confidence, enable_segmentation, refine_face_landmarks)` | các hyperparameter MediaPipe | Khởi tạo và trả về `mp.solutions.holistic.Holistic` instance |
| `run_holistic(holistic, rgb_image)` | holistic instance, ảnh RGB (numpy) | Chạy inference MediaPipe Holistic. Trả về `results` object chứa pose + hand landmarks |

---

## Nhóm: Trích xuất Landmarks 3D

| Hàm | Tham số | Chức năng |
|-----|---------|-----------|
| `extract_pose_landmarks_3d(results, registration, undistorted, mask, width, height, min_visibility, landmark_ids)` | kết quả holistic, depth registration, depth frame, người mask, kích thước ảnh, ngưỡng visibility, list ID cần trích | Chiếu pose landmark 2D → pixel → lấy depth từ depth map → tọa độ 3D camera frame. Trả về `{landmark_id: (x,y,z)}` |
| `extract_hand_landmarks_3d(hand_landmarks, hand_side, registration, undistorted, mask, width, height, min_valid_points)` | landmarks 1 bàn tay, `"left"/"right"`, depth objects, minimum số điểm hợp lệ | Trích xuất 21 landmark bàn tay → tọa độ 3D. Trả về `{landmark_id: (x,y,z)}` |

**Cơ chế:** landmark 2D (normalized) → pixel (u,v) → `registration.getPointXYZ(undistorted, v, u)` → (x,y,z) trong camera frame.

---

## Nhóm: Validation Hình học

| Hàm | Tham số | Chức năng |
|-----|---------|-----------|
| `validate_body_geometry(pose_points, min_core_points, shoulder_width_range, torso_length_range)` | skeleton dict, số điểm core tối thiểu, range vai (min,max) m, range thân (min,max) m | Kiểm tra hình học cơ thể: đủ điểm core, khoảng cách 2 vai hợp lý, chiều dài thân hợp lý. Trả `(ok, reason)` |
| `validate_hand_geometry(points, min_valid_points, max_finger_span, min_palm_size, max_palm_size)` | dict 21 points bàn tay, các ngưỡng | Kiểm tra hình học bàn tay: đủ điểm hợp lệ, kích thước lòng bàn tay trong ngưỡng. Trả `(ok, reason)` |

---

## Nhóm: Merge Skeleton

| Hàm | Tham số | Chức năng |
|-----|---------|-----------|
| `attach_hands_to_pose_wrists(pose_points, hand_points_by_side, hand_pixels_by_side, wrist_ids, max_attach_dist_m)` | skeleton body, dict tay theo side, dict pixel tay, IDs của cổ tay, ngưỡng khoảng cách (m) | Gắn hand landmark 100-120 (trái) và 200-220 (phải) vào đúng cổ tay body. Căn chỉnh offset để wrist hand trùng với wrist body |
| `merge_pose_and_hands(pose_points, left_hand_points, right_hand_points)` | dict body, dict tay trái, dict tay phải | Ghép 3 dict thành 1 skeleton dict đầy đủ (body + 2 tay) |

---

## Nhóm: Encode / Decode

| Hàm | Tham số | Chức năng |
|-----|---------|-----------|
| `skeleton_dict_to_pose_array(skeleton, frame_id, stamp, landmark_order)` | dict `{id: (x,y,z)}`, frame, stamp, thứ tự landmark | Chuyển skeleton dict → `geometry_msgs/PoseArray`. Joint thiếu → `Pose` với vị trí NaN |
| `numeric_joint_order_for_pose_array(pose_array, fallback_order)` | PoseArray, optional IDs | Chọn numeric schema canonical: tracker 51 pose hoặc fusion 9 pose |
| `pose_array_to_numeric_joint_dict(pose_array, fallback_order)` | PoseArray, optional IDs | Decode canonical PoseArray trước khi downstream chọn subset |

---

## Nhóm: Visualization

| Hàm | Tham số | Chức năng |
|-----|---------|-----------|
| `draw_filtered_skeleton(image, skeleton_pixels, connection_pairs, joint_colors, bone_color, joint_radius, bone_thickness)` | ảnh BGR numpy, dict pixel, list cặp kết nối, màu, kích thước | Vẽ skeleton (khớp + xương) lên ảnh BGR để debug. In-place |

---

## Nhóm: Helpers

| Hàm | Chức năng |
|-----|-----------|
| `is_body_landmark(name) → bool` | Trả `True` nếu tên landmark thuộc MediaPipe Pose (thân người) |
| `is_hand_landmark(name) → bool` | Trả `True` nếu tên landmark thuộc MediaPipe Hands |

---

## Constants

### `LANDMARK_ORDER` — Thứ tự joint đầy đủ

```python
LANDMARK_ORDER = [
    # Body (MediaPipe Pose IDs 0–32, subset):
    "nose",           # 0
    "left_shoulder",  # 11
    "right_shoulder", # 12
    "left_elbow",     # 13
    "right_elbow",    # 14
    "left_wrist",     # 15
    "right_wrist",    # 16
    "left_hip",       # 23
    "right_hip",      # 24
    # ... (tùy cấu hình tracked_joint_ids)

    # Left Hand (ID 100–120):
    "left_hand_wrist",       # 100
    "left_thumb_cmc",        # 101
    # ... 21 joints
    "left_pinky_tip",        # 120

    # Right Hand (ID 200–220):
    "right_hand_wrist",      # 200
    # ... 21 joints
    "right_pinky_tip",       # 220
]
```

### `BODY_LANDMARK_ORDER`

Chỉ chứa landmarks thân người (không có tay). Dùng cho pipeline Single Kinect body-only.

### Sơ đồ ID joint

```
Body (MediaPipe Pose):
  0  = nose
  11 = left_shoulder    12 = right_shoulder
  13 = left_elbow       14 = right_elbow
  15 = left_wrist       16 = right_wrist
  23 = left_hip         24 = right_hip

Left Hand (100–120):      Right Hand (200–220):
  100 = wrist               200 = wrist
  101 = thumb_cmc           201 = thumb_cmc
  102 = thumb_mcp           202 = thumb_mcp
  103 = thumb_ip            203 = thumb_ip
  104 = thumb_tip           204 = thumb_tip
  105 = index_mcp           205 = index_mcp
  106 = index_pip           206 = index_pip
  107 = index_dip           207 = index_dip
  108 = index_tip           208 = index_tip
  109 = middle_mcp          209 = middle_mcp
  ...                       ...
  117 = pinky_mcp           217 = pinky_mcp
  118 = pinky_pip           218 = pinky_pip
  119 = pinky_dip           219 = pinky_dip
  120 = pinky_tip           220 = pinky_tip
```

---

*Xem thêm: [MODULE_A_kinect_skeleton_tracker.md](MODULE_A_kinect_skeleton_tracker.md) — module sử dụng thư viện này.*
