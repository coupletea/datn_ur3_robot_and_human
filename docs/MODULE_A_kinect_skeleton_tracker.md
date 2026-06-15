# MODULE A — `kinect_skeleton_tracker.py`

> **Node name:** `kinect_skeleton_tracker`
> **Vai trò:** Đọc RGB-D từ Kinect2, nhận diện skeleton người qua YOLO + MediaPipe Holistic, publish PoseArray.

---

## Sensor modes

| Mode | Điều kiện | Hành vi |
|------|-----------|---------|
| `rgb_depth` | `~rgb_only_mode=false`, `~enable_depth=true` | Pipeline RGB-D hiện tại: đọc color + depth registered, trích skeleton 3D, TF sang `base_link`. |
| `rgb_only` | `~rgb_only_mode=true` hoặc `~enable_depth=false` | Chỉ đọc color frame, chạy YOLO + MediaPipe Holistic để debug RGB, publish skeleton rỗng vì không có depth 3D. |

Depth-only không hỗ trợ trong v1: MediaPipe Holistic cần RGB để detect pose/hand, nên mode không có RGB sẽ không tạo skeleton an toàn.

## Tổng quan pipeline

```
Kinect2 (RGB-D, default)
    │ libfreenect2
    ▼
read_registered_frames()
    │
    ├─► [YOLO] run_yolo_person_segmentation()
    │          select_best_person_mask()
    │          nearest: median depth nhỏ nhất
    │
    ├─► [MediaPipe] run_holistic(selected-mask RGB)
    │          extract_pose_landmarks_3d()
    │          validate đủ 8 core joints (Single launch)
    │          extract_hand_landmarks_3d()
    │          attach_hands_to_pose_wrists()
    │          merge_pose_and_hands()
    │
    ├─► SpatialArmFilter.update()       ← median + jump + arm-length lock
    ├─► transform_skeleton_to_target()  ← TF: camera → base_link
    └─► TemporalSkeletonFilter.update() ← EMA + confirm/lost
            │
    /human_skeleton_base  (PoseArray)
    /human_skeleton_camera (PoseArray)
    /kinect_skeleton/image_raw (Image debug)

Kinect2 (RGB-only)
    │ libfreenect2
    ▼
read_color_frame_only()
    ├─► [YOLO] debug mask nếu bật
    ├─► [MediaPipe] 2D debug landmarks nếu bật
    └─► publish_empty_skeleton("RGB_ONLY no_depth")
```

---

## Class: `SpatialArmFilter`

Lọc spatial trong camera frame trước TF: median từng trục, reject jump từng joint, hiệu chuẩn và khóa độ dài 4 xương tay. Hip clamp theo trục `+Y` camera được hỗ trợ nhưng mặc định tắt. Mỗi tracker có state riêng; state reset khi mất detection, timeout, TF error hoặc runtime error.

## Class: `TemporalSkeletonFilter`

Lọc thời gian: EMA smoothing + confirm/lost frames. Code vẫn giữ jump threshold để tương thích, nhưng Single/Dual launch đặt `0.0`; spatial filter xử lý jump.

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `__init__` | `confirm_frames, lost_frames, max_jump_m, smoothing_alpha, landmark_order` | Khởi tạo bộ lọc temporal |
| `reset()` | — | Reset toàn bộ trạng thái bộ lọc |
| `update(skeleton, stamp)` | skeleton dict, ROS stamp | Cập nhật EMA, reject nếu jump > `max_jump_m`; cần đủ `confirm_frames` trước khi publish. Trả `(ok, filtered_skeleton, reason)` |

**Cơ chế hoạt động:**
- Nhận skeleton mới → so sánh với skeleton trước (EMA)
- Nếu bất kỳ joint nào dịch chuyển > `max_jump_m` → reject frame
- Cần tích lũy ≥ `confirm_frames` liên tiếp mới publish
- Nếu mất detection ≥ `lost_frames` → reset về trạng thái ban đầu

---

## Class: `KinectSkeletonTracker`

Node chính quản lý toàn bộ pipeline.

### Nhóm: Khởi tạo & Camera

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load ROS params, khởi tạo YOLO model, MediaPipe Holistic, TF buffer, tất cả publishers/subscribers |
| `open_camera() → bool` | Mở kết nối Kinect qua libfreenect2 (ưu tiên `camera_serial` hơn `camera_device_index`) |
| `close_camera()` | Đóng kết nối Kinect an toàn (giải phóng device, pipeline) |
| `maybe_reopen_camera_after_timeout(reason)` | Xử lý timeout: thử reopen; nếu vượt ngưỡng → `os._exit(75)` để roslaunch respawn |

### Nhóm: Pipeline frame

| Phương thức | Chức năng |
|-------------|-----------|
| `process_one_frame() → dict` | **Pipeline chính cho 1 frame:** đọc ảnh → YOLO mask → Holistic → trích landmarks → validate body/hand → TF transform → temporal filter. Trả về dict kết quả |
| `process_rgb_only_frame() → dict` | Pipeline RGB-only: đọc color → YOLO/Holistic debug 2D → publish ảnh debug và skeleton rỗng |
| `publish_frame_result(result)` | Publish PoseArray (camera frame + base frame) và debug image từ kết quả `process_one_frame()` |

### Nhóm: TF

| Phương thức | Chức năng |
|-------------|-----------|
| `transform_skeleton_to_target(skeleton_camera, stamp) → skeleton_base` | Chuyển tọa độ skeleton từ `camera_frame` sang `target_frame` (thường là `base_link`) qua TF lookup |

### Nhóm: Publish & Vòng lặp

| Phương thức | Chức năng |
|-------------|-----------|
| `publish_status(text, force)` | Publish `/human_skeleton_status` với throttling (không spam) |
| `publish_empty_skeleton(stamp, reason)` | Publish PoseArray rỗng khi không detect được người |
| `publish_raw_image(image_bgr, stamp)` | Publish ảnh gốc (raw) nếu cần debug |
| `spin()` | Vòng lặp chính: đọc frame → `process_one_frame()` → `publish_frame_result()`, lặp theo `rate_hz` |
| `shutdown()` | Dọn dẹp: đóng camera, giải phóng MediaPipe, YOLO |

---

## Cấu hình — ROS Parameters

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~camera_frame` | `kinect2_ir_optical_frame` | Frame của camera trong TF |
| `~target_frame` | `base_link` | Frame robot (đích chuyển đổi TF) |
| `~camera_name` | `kinect` | Tên định danh camera (dùng trong topic prefix) |
| `~camera_device_index` | `0` | Index USB device Kinect |
| `~camera_serial` | `""` | Serial number Kinect (ưu tiên hơn `device_index`) |
| `~rate_hz` | `30.0` | Tốc độ xử lý frame (Hz) |
| `~rgb_only_mode` | `False` | Bật RGB-only. Khi bật, node không đăng ký depth listener và publish skeleton rỗng |
| `~enable_depth` | `True` | Cho phép RGB-D pipeline. Nếu `False` và `rgb_only_mode=False`, node fallback sang `rgb_only` |
| `~enable_yolo_person_segmentation` | `True` | Bật/tắt YOLO segmentation mask |
| `~yolo_model_path` | `yolov8n-seg.pt` | Đường dẫn model YOLO |
| `~yolo_conf` | `0.5` | Ngưỡng confidence YOLO |
| `~person_select_mode` | `legacy` | `legacy` giữ selector cũ; `nearest` chọn mask có median depth nhỏ nhất |
| `~nearest_min_depth_px` | `50` | Số pixel depth hợp lệ tối thiểu để tin median của mask |
| `~body_min_core_points` | `5` | Số core joint tối thiểu; Single launch đặt `8` để bắt buộc đủ vai/khuỷu/cổ tay/hông |
| `~enable_holistic` | `True` | Bật MediaPipe Holistic |
| `~holistic_model_complexity` | `0` | Độ phức tạp model (0=nhanh, 1=cân bằng, 2=chính xác) |
| `~enable_hand_detection` | `True` | Bật nhận diện tay (21 joint/tay) |
| `~temporal_confirm_frames` | `1` | Số frame xác nhận trước khi publish lần đầu |
| `~temporal_lost_frames` | `99` | Số frame mất detection trước khi reset filter |
| `~temporal_smoothing_alpha` | `0.0` | Hệ số EMA; `0.0` giữ skeleton frame hiện tại, không làm mượt |
| `~temporal_max_jump_m` | `999.0` | Giữ để tương thích; launch Single/Dual đặt `0.0` vì spatial filter xử lý jump |
| `~enable_arm_spatial_filter` | `True` | Bật spatial filter trước TF |
| `~arm_median_window` | `2` | Cửa sổ median từng joint |
| `~arm_max_jump_m` | `0.5` | Ngưỡng jump reject từng joint trong camera frame |
| `~enable_arm_length_lock` | `True` | Khóa độ dài 4 xương tay sau calibration |
| `~arm_calib_frames` | `50` | Số frame đủ 4 xương để calibration |
| `~enable_hip_clamp` | `False` | Ép hip thấp hơn shoulder theo trục camera |
| `~hip_clamp_offset_m` | `0.30` | Offset hip clamp theo `+Y` camera |
| `~debug_image_flip_horizontal` | `False` | Chỉ flip ảnh debug sau khi vẽ skeleton; Single launch mặc định tắt để ảnh/overlay không bị lật ngang |

---

## ROS Topics

| Topic | Type | Chiều | Mô tả |
|-------|------|-------|-------|
| `/human_skeleton_camera` | `geometry_msgs/PoseArray` | Publish | Skeleton trong camera frame |
| `/human_skeleton_base` | `geometry_msgs/PoseArray` | Publish | Skeleton trong `base_link` frame |
| `/human_skeleton_status` | `std_msgs/String` | Publish | Trạng thái tracker (throttled) |
| `/kinect_skeleton/image_raw` | `sensor_msgs/Image` | Publish | Ảnh debug với skeleton vẽ lên |
| `/kinect_raw/image_raw` | `sensor_msgs/Image` | Publish | Ảnh RGB gốc (raw) |

> **Dual Kinect:** Tất cả topics có prefix namespace, ví dụ `/kinect_front/human_skeleton_base`, `/kinect_back/human_skeleton_base`.

---

## Ghi chú triển khai

- **Single Kinect** (`system.launch`): `person_select_mode=nearest`, `body_min_core_points=8`; chọn người gần trước rồi reject nếu thiếu core joint, không chuyển sang người xa.
- Nearest dùng raw YOLO mask để đo median depth, sau đó dùng dilated selected mask che RGB trước Holistic để ép model theo đúng người.
- Nearest RGB-D ép YOLO chạy sync vì selection cần depth cùng frame. RGB-only fallback selector legacy và warning throttled.
- **Safety:** gate đủ 8 core joint tránh publish skeleton thiếu tay/thân, nhưng occlusion một joint sẽ làm output rớt frame.
- **Known limitation:** YOLO gộp hai người vào cùng mask có thể làm median depth không đại diện đúng một người.
- **Dual Kinect front** (`dual_kinect_system.launch`): `rate_hz=10`, `holistic_complexity=0`, `enable_yolo=true`
- **Dual Kinect back**: `rate_hz=5`, `holistic_complexity=0`, `enable_yolo=false` (nhẹ hơn)
- Launch args mới:
  `system.launch rgb_only_mode:=true enable_depth:=false`,
  `dual_kinect_system.launch front_rgb_only_mode:=true front_enable_depth:=false`,
  `back_kinect_perception_test.launch rgb_only_mode:=true enable_depth:=false`.
- `rgb_only` giữ raw/debug image nhưng không publish 3D skeleton; trong dual Kinect, MODULE C sẽ fallback sang camera còn RGB-D hoặc `NO_INPUT` nếu cả hai camera đều RGB-only.
- Khi frame timeout vượt ngưỡng → node tự `os._exit(75)` để roslaunch respawn tự động
- Robot false-positive filter và reject skeleton dưới `base_link` đã bị bỏ khỏi pipeline để skeleton vẫn publish khi người đứng gần robot. Rủi ro: robot tự lọt vào detector có thể tạo skeleton giả.
- Spatial jump reject là lớp jump duy nhất trong launch mặc định; `temporal_max_jump_m=0.0` tránh double filtering.
- Hip clamp mặc định tắt vì có thể làm sai tư thế cúi/ngồi; chỉ bật sau test camera thực tế.

## Runtime rate control (thêm trong feat/dual-kinect-fusion-controller)

Module A hỗ trợ điều chỉnh tốc độ vòng lặp tại runtime thông qua ROS topic, phục vụ `DutyScheduler` trong MODULE C.

### Hàm helper

```python
clamp_rate(value, rate_min, rate_max) -> float
```

Clamp giá trị rate vào `[rate_min, rate_max]`. Giá trị không finite bị bỏ qua.

### Params mới

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~rate_cmd_topic` | `""` (rỗng) | Topic nhận lệnh rate (`std_msgs/Float32`). Để trống → feature **tắt** hoàn toàn |
| `~rate_min` | `1.0` | Rate tối thiểu cho phép (Hz) |
| `~rate_max` | `60.0` | Rate tối đa cho phép (Hz); 60.0 đảm bảo launch hiện tại với `~rate_hz=30` không bị clamp |

### Subscriber

Khi `~rate_cmd_topic` được đặt (không rỗng), node subscribe topic đó với type `std_msgs/Float32`.

```
_on_rate_cmd(msg):
  1. Bỏ qua nếu giá trị không finite (NaN, Inf)
  2. Clamp vào [rate_min, rate_max]
  3. Ghi giá trị mới vào biến target_rate (dưới lock)

spin():
  Đọc target_rate (dưới lock)
  Nếu rate thay đổi → tạo lại rospy.Rate với target_rate mới
```

### Default-OFF

- **Single-cam launches** (`system.launch`, `system_back.launch`): không đặt `~rate_cmd_topic` → hành vi **hoàn toàn giống trước**, `~rate_hz` tĩnh được dùng.
- `~rate_max=60.0` (mặc định) đảm bảo launch có `~rate_hz=30` không bị giới hạn.

### Safety

- Giá trị không finite bị bỏ qua (ignore) — không cập nhật rate.
- Clamp vào `[rate_min, rate_max]` — không cho stall (rate=0) hoặc runaway.
- Lock thread-safe giữa callback và `spin()`.

---

## Verification

```bash
python3 test/test_person_mask_selection.py -v
python3 test/test_spatial_arm_filter.py -v
python3 -m py_compile scripts/data_skeleton.py scripts/kinect_skeleton_tracker.py
```

---

*Xem thêm: [MODULE_B_data_skeleton.md](MODULE_B_data_skeleton.md) — thư viện tiện ích được import bởi module này.*
