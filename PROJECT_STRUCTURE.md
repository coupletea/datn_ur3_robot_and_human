# PROJECT STRUCTURE — UR3 HRC Planner

> Hệ thống lập kế hoạch robot UR3 với cộng tác người-robot (Human-Robot Collaboration).
> Nhận diện skeleton người qua Kinect, xây dựng vật cản động, lập kế hoạch đường đi an toàn.

---

## Mục lục

1. [Tổng quan hệ thống](#1-tổng-quan-hệ-thống)
2. [Sơ đồ khối — Single Kinect](#2-sơ-đồ-khối--single-kinect)
3. [Sơ đồ khối — Dual Kinect](#3-sơ-đồ-khối--dual-kinect)
4. [Cấu trúc thư mục](#4-cấu-trúc-thư-mục)
5. [Module: kinect\_skeleton\_tracker.py](#5-module-kinect_skeleton_trackerpy)
6. [Module: data\_skeleton.py](#6-module-data_skeletonpy)
7. [Module: dual\_kinect\_fusion\_controller.py](#7-module-dual_kinect_fusion_controllerpy)
8. [Module: skeleton\_obstacle\_builder.py](#8-module-skeleton_obstacle_builderpy)
9. [Module: moveit\_scene\_manager.py](#9-module-moveit_scene_managerpy)
10. [Module: planner\_ab\_replan\_node.py](#10-module-planner_ab_replan_nodepy)
11. [Module: astar\_improved\_3d.py](#11-module-astar_improved_3dpy)
11b. [Module: astar\_lpa\_3d.py](#11b-module-astar_lpa_3dpy)
12. [ROS Topics & TF](#12-ros-topics--tf)
13. [Launch Files](#13-launch-files)
14. [So sánh Single Kinect vs Dual Kinect](#14-so-sánh-single-kinect-vs-dual-kinect)

---

## 1. Tổng quan hệ thống

```
Người lao động <──────────────────────────────────────────────────────────────────┐
     │                                                                             │
     │ (RGB-D stream)                                                              │ (robot tránh)
     ▼                                                                             │
[Kinect2 Camera(s)]                                                        [UR3 Robot Arm]
     │                                                                             ▲
     │ libfreenect2                                                                │
     ▼                                                                        [Execute]
[kinect_skeleton_tracker] ──skeleton──► [skeleton_obstacle_builder] ──CollisionObject──►
     │                                                                   [moveit_scene_manager]
     │ (Single Kinect)                                                             │
     │ hoặc                                                               MoveIt scene
     ▼                                                                             │
[dual_kinect_fusion_controller]  ◄───────────────────────────────────             │
     │ (Dual Kinect: fusion + duty-cycle)                                          │
     │                                                                             │
     └──────────► /human_skeleton_base ──────────► [planner_ab_replan_node] ──────┘
                    (PoseArray)                        │
                                                  ARA* (AStarImproved3D) + MoveIt
                                                  lập kế hoạch đường đi
                                                  kiểm tra an toàn
```

---

## 2. Sơ đồ khối — Single Kinect

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          SINGLE KINECT PIPELINE                                  │
│                        (launch: system.launch)                                   │
└─────────────────────────────────────────────────────────────────────────────────┘

  ┌───────────────┐    RGB+Depth     ┌──────────────────────────────────────────┐
  │  Kinect2 v2   │ ──────────────► │         kinect_skeleton_tracker           │
  │  (libfreenect2│                  │                                           │
  │   USB 3.0)    │                  │  1. initialize_camera()                   │
  └───────────────┘                  │  2. read_registered_frames()              │
                                     │  3. [YOLO] run_yolo_person_segmentation() │
         Static TF                   │     select_best_person_mask()             │
    base_link ──► kinect2_           │  4. [MediaPipe] run_holistic()            │
    ir_optical_frame                 │     extract_pose_landmarks_3d()           │
                                     │     extract_hand_landmarks_3d()           │
                                     │     attach_hands_to_pose_wrists()         │
                                     │     merge_pose_and_hands()                │
                                     │  5. SpatialArmFilter.update()             │
                                     │  6. transform_skeleton_to_target()  (TF)  │
                                     │  7. TemporalSkeletonFilter.update()       │
                                     └──────────────────────────────────────────┘
                                            │                  │
                              /human_skeleton_base        /kinect_skeleton/
                              (PoseArray, base_link)       image_raw (debug)
                                            │
                    ┌───────────────────────┼───────────────────────┐
                    ▼                       ▼                       ▼
        ┌─────────────────────┐  ┌──────────────────┐  ┌─────────────────────┐
        │ skeleton_obstacle_  │  │ planner_ab_replan│  │  moveit_scene_      │
        │ builder             │  │ _node             │  │  manager            │
        │                     │  │                   │  │                     │
        │ Joints → Spheres    │  │ ARA* guard        │  │ Apply CollisionObj  │
        │ Bones  → Cylinders  │  │ MoveIt planning   │  │ to MoveIt scene    │
        │ Fixed radius 0.05m │  │ Trajectory safety │  │ Scene confirmation  │
        └─────────────────────┘  │ Emergency stop    │  └─────────────────────┘
                 │               │ A→B→A cycles      │           │
         /human_collision_object └──────────────────┘   /moveit_scene_status
         (CollisionObject)                │
                 │                  /ur3_fixed_joint
                 └──────────────►   _path_status
                                         │
                                    UR3 Robot Arm
```

---

## 3. Sơ đồ khối — Dual Kinect

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          DUAL KINECT PIPELINE                                    │
│                      (launch: dual_kinect_system.launch)                         │
└─────────────────────────────────────────────────────────────────────────────────┘

  ┌──────────────────┐  RGB+Depth   ┌─────────────────────────────────────────┐
  │  Kinect FRONT    │ ────────────►│  kinect_skeleton_tracker (ns: kinect_   │
  │  (device_index=0)│              │  front)                                  │
  │  Serial:         │              │  camera_name = "front"                   │
  │  196605135147    │              │  camera_frame = kinect_front_            │
  └──────────────────┘              │                ir_optical_frame          │
                                    └─────────────────────────────────────────┘
   Static TF:                                         │
   base_link ──►                        /kinect_front/human_skeleton_base
   kinect_front_ir_optical_frame        /kinect_front/human_skeleton_camera
                                        /kinect_front/kinect_skeleton/image_raw
                                                      │
                                                      ▼
  ┌──────────────────┐  RGB+Depth   ┌─────────────────────────────────────────┐
  │  Kinect BACK     │ ────────────►│  kinect_skeleton_tracker (ns: kinect_   │
  │  (device_index=1)│              │  back)                                   │
  │  Serial:         │              │  camera_name = "back"                    │
  │  299150235147    │              │  camera_frame = kinect_back_             │
  └──────────────────┘              │                ir_optical_frame          │
                                    └─────────────────────────────────────────┘
   Static TF:                                         │
   base_link ──►                        /kinect_back/human_skeleton_base
   kinect_back_ir_optical_frame         /kinect_back/human_skeleton_camera
                                        /kinect_back/kinect_skeleton/image_raw
                                                      │
                           ┌──────────────────────────┘
                           ▼
              ┌──────────────────────────────────────────────────────────────┐
              │              dual_kinect_fusion_controller                    │
              │                                                               │
              │  Timer-driven (20 Hz):                                        │
              │                                                               │
              │  SkeletonFuser (per-joint occlusion-fill):                    │
              │    1. decode cả 2 cache → numeric joint dict                  │
              │    2. freshness check (max_input_age_sec = 0.35 s)            │
              │    3. per-joint: both-fresh+close → average                   │
              │                  both-fresh+far   → front (primary)           │
              │                  front-only        → front                    │
              │                  back-only         → back  (occlusion fill)   │
              │                  both-stale         → NaN                     │
              │    4. mode: BOTH/FILL_FROM_BACK/FRONT_ONLY/BACK_ONLY/NO_INPUT│
              │                                                               │
              │  DutyScheduler (back rate duty-cycle):                        │
              │    5. đếm front_missing joints                                │
              │    6. boost back HIGH nếu miss ≥ 2 trong 3 tick liên tiếp    │
              │    7. giữ HIGH thêm boost_hold_sec (hysteresis) khi front hồi│
              │    8. hạ back về LOW sau hysteresis                           │
              └──────────────────────────────────────────────────────────────┘
                                        │
                              /human_skeleton_base          (PoseArray fused)
                              /human_skeleton_fusion_status (String chi tiết: mode + duty)
                              /kinect_front/tracker_rate_cmd (Float32, Hz)
                              /kinect_back/tracker_rate_cmd  (Float32, Hz)
                                        │
                    ┌───────────────────┼───────────────────┐
                    ▼                   ▼                   ▼
       ┌──────────────────┐  ┌──────────────────┐  ┌────────────────────┐
       │skeleton_obstacle_│  │planner_ab_replan │  │moveit_scene_manager│
       │builder           │  │_node             │  │                    │
       └──────────────────┘  └──────────────────┘  └────────────────────┘
                │                      │                      │
       /human_collision_object   UR3 Robot Arm        /moveit_scene_status
```

---

## 4. Cấu trúc thư mục

```
catkin_ws/
├── src/
│   ├── ur3_hrc_planner/                  # Package chính
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── PROJECT_STRUCTURE.md          # (file này)
│   │   │
│   │   ├── AGENTS.md                         # Hướng dẫn agent/AI workflow
│   │   ├── CONVERT_DSTAR_TO_ARASTAR.md       # Migration note D*Lite → ARA*
│   │   ├── EXECUTION_PHASES.md               # Kế hoạch các phase thực thi
│   │   ├── docs/
│   │   │   ├── PHASE1_IMPLEMENTATION_PLAN.md
│   │   │   └── SYSTEM_FLOW_DIAGRAMS.md           # Sơ đồ luồng tổng quát + chi tiết từng module
│   │   ├── test_astar_vs_dstar.py            # Benchmark ARA* vs D*Lite
│   │   ├── test/
│   │   │   ├── test_spatial_arm_filter.py     # Unit test spatial filter
│   │   │   ├── test_pose_array_schema.py      # Regression test PoseArray schema
│   │   │   ├── test_person_mask_selection.py  # Unit test legacy/nearest person selector
│   │   │   ├── test_collision_visualization.py # Collision marker + scene color tests
│   │   │   └── test_planner_detour_retry.py    # Bounded detour retry, ARA* gate, HOLD tests
│   │   │
│   │   ├── scripts/                          # Các ROS node Python
│   │   │   ├── kinect_skeleton_tracker.py        # [MODULE A] Tracker 1 Kinect (+ runtime rate control)
│   │   │   ├── data_skeleton.py                  # [MODULE B] Tiện ích xử lý ảnh/skeleton
│   │   │   ├── dual_kinect_fusion_controller.py  # [MODULE C] Fusion 2 Kinect + duty-cycle rate
│   │   │   ├── skeleton_obstacle_builder.py      # [MODULE D] Xây dựng vật cản
│   │   │   ├── moveit_scene_manager.py           # [MODULE E] Quản lý MoveIt scene
│   │   │   ├── planner_ab_replan_node.py         # [MODULE F] Lập kế hoạch + thực thi
│   │   │   ├── astar_improved_3d.py              # [MODULE G] ARA* (Anytime Repairing A*)
│   │   │   ├── astar_lpa_3d.py                   # [MODULE G] LPA* (Lifelong Planning A*) - guard planner thay thế, chọn bằng ~guard_planner_type
│   │   │   └── test_astar_lpa_3d.py              # [TEST] self-test offline cho LPAStar3D (python3 scripts/test_astar_lpa_3d.py)
│   │   │
│   │   ├── launch/
│   │   │   ├── system.launch                     # Khởi động Single Kinect (front)
│   │   │   ├── system_back.launch                # Khởi động Single Kinect (back)
│   │   │   ├── dual_kinect_system.launch         # Khởi động Dual Kinect
│   │   │   ├── rviz.launch                       # Visualization
│   │   │   └── back_kinect_perception_test.launch # Test nhận diện back Kinect
│   │   │
│   │   └── logs/                                 # Log sự kiện plan (ghi runtime bởi PlanLogger)
│   │       ├── README.md                         # Mô tả format CSV/.log + cấu hình
│   │       └── plan_log_<timestamp>.csv/.log     # Tạo tự động mỗi lần chạy planner
│   │
│   ├── iai_kinect2/                      # Driver Kinect2 (libfreenect2)
│   │   ├── kinect2_bridge/               # Bridge ROS-Kinect
│   │   │   ├── data/
│   │   │   │   ├── 196605135147/         # Calibration Kinect FRONT
│   │   │   │   │   ├── calib_color.yaml
│   │   │   │   │   ├── calib_depth.yaml
│   │   │   │   │   ├── calib_ir.yaml
│   │   │   │   │   └── calib_pose.yaml
│   │   │   │   └── 299150235147/         # Calibration Kinect BACK
│   │   │   │       ├── calib_color.yaml
│   │   │   │       ├── calib_ir.yaml
│   │   │   │       └── calib_pose.yaml
│   │   │   └── launch/
│   │   │       └── kinect2_bridge.launch
│   │   ├── kinect2_calibration/          # Tool hiệu chỉnh camera
│   │   └── kinect2_registration/         # Đăng ký ảnh color-depth
│   │
│   └── rviz/                             # Cấu hình RViz
└── devel/, build/                        # Build artifacts (catkin)
```

### Test Module F: `test/test_planner_detour_retry.py`

- **File role:** Unit/regression test.
- **Main inputs:** Mock `PoseStamped`, ROS shutdown/sleep, MoveIt group và planner results.
- **Main outputs:** Assertions cho bounded retry, pose ARA* gate, planning timeout restore, HOLD không replan và detour pose không mutate waypoint.
- **Related module:** Module F — planner / A-B trajectory / replan node.

### Test Module B: `test/test_person_mask_selection.py`

- **File role:** Unit/regression test.
- **Main inputs:** YOLO detection masks giả lập và aligned depth map NumPy.
- **Main outputs:** Assertions cho legacy selection, nearest median depth, area/depth fallback, tie handling và mask immutability.
- **Related module:** Module B — person mask selection; runtime code không cần camera thật để chạy test.

---

## 5. Module: `kinect_skeleton_tracker.py`

**Node name:** `kinect_skeleton_tracker`
**Chức năng:** Đọc RGB-D hoặc RGB-only từ Kinect, nhận diện skeleton người, publish PoseArray.

### Sensor modes

| Mode | Điều kiện | Hành vi |
|------|-----------|---------|
| `rgb_depth` | `~rgb_only_mode=false`, `~enable_depth=true` | Đọc color + depth registered, extract skeleton 3D, TF sang `base_link` |
| `rgb_only` | `~rgb_only_mode=true` hoặc `~enable_depth=false` | Chỉ đọc color, chạy YOLO + Holistic để debug RGB, publish PoseArray rỗng vì không có depth |

Depth-only chưa hỗ trợ trong v1. Nếu cả hai Kinect đều RGB-only trong dual pipeline, MODULE C sẽ nhận input rỗng và chuyển sang `NO_INPUT`.

### Classes

#### `SpatialArmFilter`
Lọc skeleton trong camera frame: median, per-joint jump reject, calibration/lock độ dài 4 xương tay, optional hip clamp. Mỗi tracker giữ state riêng.

#### `TemporalSkeletonFilter`
Lọc thời gian cho skeleton (EMA smoothing + confirm/lost). Single/Dual launch đặt `temporal_max_jump_m=0.0` để spatial filter là lớp jump duy nhất.

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__(confirm_frames, lost_frames, max_jump_m, smoothing_alpha, landmark_order)` | Khởi tạo bộ lọc temporal |
| `reset()` | Reset trạng thái bộ lọc |
| `update(skeleton, stamp) → (ok, filtered_skeleton, reason)` | Cập nhật EMA + confirm/lost; launch tắt temporal jump để tránh double filtering |

#### `KinectSkeletonTracker`
Node chính quản lý toàn bộ pipeline nhận diện.

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load params, khởi tạo YOLO, MediaPipe Holistic, TF buffer, publishers |
| `open_camera() → bool` | Mở kết nối Kinect qua libfreenect2 |
| `close_camera()` | Đóng kết nối Kinect an toàn |
| `maybe_reopen_camera_after_timeout(reason)` | Xử lý timeout frame: reopen hoặc os._exit(75) để roslaunch respawn |
| `process_one_frame() → dict` | **Pipeline chính một frame:** đọc ảnh → YOLO nearest/legacy mask → mask RGB cho nearest → Holistic → extract/validate → spatial filter → TF → temporal filter |
| `process_rgb_only_frame() → dict` | RGB-only: nearest fallback legacy, đọc color → YOLO/Holistic debug 2D → publish skeleton rỗng |
| `publish_frame_result(result)` | Publish PoseArray (camera + base frame) và debug image |
| `transform_skeleton_to_target(skeleton_camera, stamp) → skeleton_base` | Chuyển tọa độ skeleton từ camera frame sang base_link qua TF |
| `publish_status(text, force)` | Publish status với throttling |
| `publish_empty_skeleton(stamp, reason)` | Publish PoseArray rỗng khi không detect được |
| `publish_raw_image(image_bgr, stamp)` | Publish ảnh raw nếu cần debug |
| `spin()` | Vòng lặp chính: đọc frame → xử lý → publish |
| `shutdown()` | Dọn dẹp tài nguyên |

### Tham số ROS quan trọng

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~camera_frame` | `kinect2_ir_optical_frame` | Frame của camera |
| `~target_frame` | `base_link` | Frame robot |
| `~camera_name` | `kinect` | Tên định danh camera |
| `~camera_device_index` | `0` | Index USB device |
| `~camera_serial` | `""` | Serial number (ưu tiên hơn index) |
| `~rate_hz` | `30.0` | Tốc độ xử lý frame |
| `~rgb_only_mode` | `False` | Bật RGB-only, không đăng ký depth listener |
| `~enable_depth` | `True` | Cho phép RGB-D; nếu `False` khi `rgb_only_mode=False`, fallback sang RGB-only |
| `~enable_yolo_person_segmentation` | `True` | Bật YOLO segmentation |
| `~yolo_model_path` | `yolov8n-seg.pt` | Đường dẫn model YOLO |
| `~person_select_mode` | `legacy` | `legacy` giữ score cũ; `nearest` chọn raw mask theo median depth |
| `~nearest_min_depth_px` | `50` | Số pixel depth hợp lệ tối thiểu/mask |
| `~body_min_core_points` | `5` | Gate core joint; Single launch đặt `8` |
| `~enable_holistic` | `True` | Bật MediaPipe Holistic |
| `~holistic_model_complexity` | `0` | Độ phức tạp model (0/1/2) |
| `~enable_hand_detection` | `True` | Bật nhận diện tay |
| `~temporal_confirm_frames` | `1` | Số frame xác nhận trước khi publish |
| `~temporal_lost_frames` | `99` | Số frame mất detection trước khi reset filter |
| `~temporal_smoothing_alpha` | `0.0` | Hệ số EMA; default giữ frame hiện tại |
| `~temporal_max_jump_m` | `999.0` | Ngưỡng reject jump; default gần như không chặn |
| `~enable_arm_spatial_filter` | `True` | Bật spatial filter trước TF |
| `~arm_median_window` | `2` | Cửa sổ median |
| `~arm_max_jump_m` | `0.5` | Ngưỡng spatial jump reject |
| `~enable_arm_length_lock` | `True` | Bật calibration/lock độ dài xương tay |
| `~arm_calib_frames` | `50` | Số frame calibration |
| `~enable_hip_clamp` | `False` | Optional hip clamp, mặc định tắt |
| `~hip_clamp_offset_m` | `0.30` | Offset hip clamp theo `+Y` camera |
| `~debug_image_flip_horizontal` | `False` | Flip ngang ảnh debug; Single launch mặc định tắt để ảnh và skeleton overlay cùng chiều |

### Topics (Single Kinect)

| Topic | Type | Chiều | Mô tả |
|-------|------|-------|-------|
| `/human_skeleton_camera` | `PoseArray` | Publish | Skeleton trong camera frame |
| `/human_skeleton_base` | `PoseArray` | Publish | Skeleton trong base_link frame |
| `/human_skeleton_status` | `String` | Publish | Trạng thái tracker |
| `/kinect_skeleton/image_raw` | `Image` | Publish | Ảnh debug có skeleton |
| `/kinect_raw/image_raw` | `Image` | Publish | Ảnh raw gốc |

---

## 6. Module: `data_skeleton.py`

**Chức năng:** Thư viện tiện ích, không phải ROS node. Import bởi `kinect_skeleton_tracker.py`.

### Hàm chính

| Hàm | Chức năng |
|-----|-----------|
| `initialize_camera(device_index, serial, packet_pipeline, color_only)` | Mở Kinect qua libfreenect2, trả về (freenect, device, listener, registration); color-only trả `registration=None` |
| `read_registered_frames(listener, registration, timeout_ms)` | Đọc frame RGB-D đã đăng ký, raise `KinectFrameTimeout` nếu quá hạn |
| `read_color_frame_only(listener, timeout_ms)` | Đọc color frame cho RGB-only mode, trả `(color_bgr, color_rgb)` |
| `load_yolo_segment_model(model_path, device)` | Load model YOLO segmentation từ file |
| `run_yolo_person_segmentation(model, rgb, conf, iou, class_id)` | Chạy YOLO, trả về danh sách detections |
| `select_best_person_mask(detections, shape, min_area_ratio, prefer_center, select_mode, depth_map, min_depth_px)` | Chọn mask bằng legacy score hoặc median depth gần nhất; fallback legacy khi depth không đáng tin |
| `dilate_mask(mask, px)` | Giãn nở mask bằng morphological dilation |
| `load_mediapipe_holistic(...)` | Khởi tạo MediaPipe Holistic model |
| `run_holistic(holistic, rgb_image)` | Chạy inference MediaPipe Holistic |
| `extract_pose_landmarks_3d(results, registration, undistorted, mask, width, height, min_visibility, landmark_ids)` | Trích xuất tọa độ 3D pose landmarks từ depth map |
| `extract_hand_landmarks_3d(hand_landmarks, hand_side, registration, undistorted, mask, width, height, min_valid_points)` | Trích xuất tọa độ 3D hand landmarks |
| `validate_body_geometry(pose_points, min_core_points, shoulder_width_range, torso_length_range)` | Kiểm tra hình học cơ thể (vai, thân người) |
| `validate_hand_geometry(points, min_valid_points, max_finger_span, min_palm_size, max_palm_size)` | Kiểm tra hình học bàn tay |
| `attach_hands_to_pose_wrists(pose_points, hand_points_by_side, hand_pixels_by_side, ...)` | Gắn hand landmarks vào wrist của pose |
| `merge_pose_and_hands(pose_points, left_hand_points, right_hand_points)` | Ghép pose body + 2 bàn tay thành 1 skeleton dict |
| `skeleton_dict_to_pose_array(skeleton, frame_id, stamp, landmark_order)` | Chuyển skeleton dict → ROS PoseArray |
| `draw_filtered_skeleton(image, skeleton_pixels, ...)` | Vẽ skeleton lên ảnh để debug |
| `is_body_landmark(name) → bool` | Kiểm tra là landmark thân người |
| `is_hand_landmark(name) → bool` | Kiểm tra là landmark bàn tay |

### Constants

| Hằng | Giá trị | Mô tả |
|------|---------|-------|
| `LANDMARK_ORDER` | List[str] | Thứ tự tất cả landmarks (body + 2 tay) |
| `BODY_LANDMARK_ORDER` | List[str] | Chỉ landmarks thân người |
| `KinectFrameTimeout` | Exception | Exception khi timeout đọc frame |

### Sơ đồ ID joint

```
Body joints (MediaPipe Pose):
  0=nose, 11=left_shoulder, 12=right_shoulder
  13=left_elbow, 14=right_elbow
  15=left_wrist, 16=right_wrist
  23=left_hip, 24=right_hip

Left Hand joints (100-120):   Right Hand joints (200-220):
  100=wrist                     200=wrist
  101-104=thumb                 201-204=thumb
  105-108=index                 205-208=index
  109-112=middle                209-212=middle
  113-116=ring                  213-216=ring
  117-120=pinky                 217-220=pinky
```

---

## 7. Module: `dual_kinect_fusion_controller.py`

**Node name:** `dual_kinect_fusion_controller`
**Chức năng:** Fuse skeleton từ 2 Kinect thành 1 skeleton đầu ra; điều phối duty-cycle tốc độ tracker back theo tình trạng occlusion.
**Kiến trúc:** **Timer-driven** tại 20 Hz. Hai component nội bộ: `SkeletonFuser` (fusion logic) và `DutyScheduler` (rate duty-cycle).

### Ba classes

#### `SkeletonFuser` (pure logic)

Per-joint occlusion-fill — không Kalman, không conflict map, không side-swap:

| Điều kiện | Kết quả |
|-----------|---------|
| Cả 2 fresh, `dist <= max_merge_dist` | Average (trung bình) |
| Cả 2 fresh, `dist > max_merge_dist` | Dùng front (primary) |
| Chỉ front fresh | Dùng front |
| Chỉ back fresh | Dùng back (occlusion fill) |
| Cả 2 stale | NaN — joint bị bỏ |

Modes: `BOTH`, `FILL_FROM_BACK`, `FRONT_ONLY`, `BACK_ONLY`, `NO_INPUT`.

#### `DutyScheduler` (pure logic)

State machine điều phối rate tracker back:

- Front luôn HIGH (primary)
- Back mặc định LOW; boost lên HIGH khi front thiếu >= `miss_threshold` (2) joints trong `confirm_frames` (3) tick liên tiếp
- Giữ HIGH thêm `boost_hold_sec` (1.5 s) sau khi front phục hồi, rồi hạ về LOW
- Chỉ publish Float32 rate command khi giá trị thay đổi

#### `DualKinectFusionController` (ROS node)

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load params, khởi tạo subscribers/publishers, Fuser, DutyScheduler, timer 20 Hz |
| `_front_cb(msg)` | Cache front PoseArray + thời gian nhận (mutex-guarded) |
| `_back_cb(msg)` | Cache back PoseArray + thời gian nhận (mutex-guarded) |
| `_timer_cb(event)` | Timer 20 Hz: đọc cache → fuse → duty tick → publish; exception-guarded |
| `spin()` | `rospy.spin()` |

### Tham số ROS quan trọng

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~fusion_rate_hz` | `20` | Tần số timer (Hz) |
| `~max_input_age_sec` | `0.35` | Tuổi tối đa để camera còn fresh |
| `~max_merge_dist` | `0.20` | Khoảng cách tối đa (m) để average 2 joint |
| `~miss_threshold` | `2` | Số joint front thiếu để trigger boost back |
| `~confirm_frames` | `3` | Số tick liên tiếp thiếu trước khi boost |
| `~boost_hold_sec` | `1.5` | Hysteresis giữ back HIGH sau khi front phục hồi |
| `~front_high_rate` | `10` | Rate front HIGH (Hz) |
| `~back_high_rate` | `6` | Rate back HIGH (Hz) |
| `~back_low_rate` | `2` | Rate back LOW (Hz) |
| `~empty_publish_on_no_input` | `true` | Publish skeleton NaN khi cả 2 stale |

### Topics

| Topic | Type | Chiều | Mô tả |
|-------|------|-------|-------|
| `/kinect_front/human_skeleton_base` | `PoseArray` | Subscribe | Skeleton front camera (base_link) |
| `/kinect_back/human_skeleton_base` | `PoseArray` | Subscribe | Skeleton back camera (base_link) |
| `/human_skeleton_base` | `PoseArray` | Publish | Skeleton đã fusion (byte-compatible với downstream) |
| `/human_skeleton_fusion_status` | `String` | Publish | Mode + duty state + joint counts |
| `/kinect_front/tracker_rate_cmd` | `Float32` | Publish | Lệnh rate front tracker |
| `/kinect_back/tracker_rate_cmd` | `Float32` | Publish | Lệnh rate back tracker |

---

## 8. Module: `skeleton_obstacle_builder.py`

**Node name:** `skeleton_obstacle_builder`
**Chức năng:** Nhận skeleton PoseArray, xây dựng CollisionObject cho MoveIt bằng body joints/bones.

### Class: `SkeletonObstacleBuilder`

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load params, khởi tạo publishers/subscribers/timer |
| `decode_pose_array(msg) → SkeletonDict` | Decode canonical 51/9-pose schema rồi chọn builder joints |
| `sanitize_skeleton(skeleton) → SkeletonDict` | Loại điểm NaN/Inf, kiểm tra ≥ min_valid_joints |
| `_max_joint_delta(prev, current) → float` | Tính max displacement giữa common joints để skip rebuild khi nhỏ hơn `delta_threshold` |
| `_extract_positions_np(skeleton) → np.ndarray` | Tạo mảng vị trí `(N,3)` theo `tracked_joint_ids` |
| `_compute_speed_vectorized(positions_np, stamp) → float` | Tính tốc độ tối đa của joint bằng NumPy |
| `compute_dynamic_padding(max_speed) → float` | Tính padding động = gain × (speed - deadband) |
| `smooth_dynamic_padding(padding) → float` | EMA cho dynamic padding |
| `final_joint_radius(dynamic_padding) → float` | Body joint radius cuối = base + static + dynamic |
| `final_bone_radius(dynamic_padding) → float` | Body bone radius cuối = base + static + dynamic |
| `make_sphere(point, radius) → (SolidPrimitive, Pose)` | Tạo primitive hình cầu |
| `make_cylinder_between(p1, p2, radius) → Optional` | Tạo primitive hình trụ giữa 2 điểm |
| `build_collision_object(skeleton, stamp, padding) → CollisionObject` | Tạo CollisionObject từ body joints/bones, không cap primitive count |
| `build_visualization_markers(obj) → MarkerArray` | Tạo marker đỏ từ đúng pose/kích thước CollisionObject |
| `publish_delete_markers(stamp)` | Xóa marker RViz khi obstacle bị REMOVE |
| `remove_object()` | Gửi lệnh REMOVE CollisionObject |
| `publish_status(text, force)` | Publish trạng thái với throttling |
| `skeleton_callback(msg)` | Callback nhận skeleton, update timestamp, delta gate, build/publish khi cần |
| `cleanup_timer(event)` | Timer: remove object nếu không nhận được data quá lâu |

**File role:** runtime ROS node.
**Main inputs:** `/human_skeleton_base` (`PoseArray`).
**Main outputs:** `/human_collision_object` (`CollisionObject`), `/human_collision_markers` (`MarkerArray` đỏ cùng geometry), `/human_obstacle_status` (`String`).
**Related module:** Module D — obstacle modeling from skeleton.

### Hình học CollisionObject

Mỗi joint hợp lệ tạo một sphere; mỗi connection pair hợp lệ tạo một cylinder.

### Dynamic Safety Padding

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
     │
     ▼ EMA smoothing (alpha = 0.7)
dynamic_padding_smooth
     │
     ▼
final_radius = base_radius + static_padding + dynamic_padding_smooth
```

Single/Dual launch dùng sphere/cylinder radius cố định `0.05m`; static/dynamic padding đặt `0.0`.

### Delta Gate & Removed Params

- `last_msg_time` được cập nhật trước decode/sanitize/skip để cleanup timer không xóa object khi skeleton đứng yên.
- `~delta_threshold` mặc định `0.02m`; nếu delta nhỏ hơn ngưỡng, node giữ geometry cũ và heartbeat CollisionObject mỗi `0.20s`.
- Numeric PoseArray decode theo schema canonical trước khi chọn subset, tránh tráo joint giữa tracker, builder, planner và fusion.
- Module D không còn tạo hand collision primitives; upstream hand detection/fusion vẫn có thể tồn tại cho planner/module khác.
- Đã bỏ `append_limited()` và `~max_primitives_per_object`; primitive count không bị cap trong builder.
- Params cũ đã bỏ khỏi Module D: `~hand_joint_radius`, `~hand_bone_radius`, `~hand_static_safety_padding`, `~min_joint_delta_to_publish`, `~min_padding_delta_to_publish`.

### Topics

| Topic | Type | Chiều | Mô tả |
|-------|------|-------|-------|
| `/human_skeleton_base` | `PoseArray` | Subscribe | Skeleton đầu vào |
| `/human_collision_object` | `CollisionObject` | Publish | Vật cản cho MoveIt |
| `/human_collision_markers` | `MarkerArray` | Publish | Marker đỏ dùng đúng pose/kích thước sphere/cylinder CollisionObject |
| `/human_obstacle_status` | `String` | Publish | Trạng thái |

---

## 9. Module: `moveit_scene_manager.py`

**Node name:** `moveit_scene_manager`
**Chức năng:** Nhận CollisionObject, apply vào MoveIt planning scene, verify và publish trạng thái.

ADD scene diff gắn `ObjectColor` đỏ (`~object_color_rgba`) để PlanningScene RViz hiển thị obstacle người màu đỏ.

### Class: `MoveItSceneManager`

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load params, kết nối `/apply_planning_scene`, khởi tạo subscribers |
| `collision_object_callback(msg)` | Nhận CollisionObject, validate, gửi vào xử lý |
| `apply_collision_object(obj)` | Gọi `/apply_planning_scene` service để áp dụng |
| `get_scene_object(object_id) → Optional[CollisionObject]` | Lấy CollisionObject hiện tại từ scene |
| `confirm_object_in_scene(object_id, expected_present) → bool` | Helper debug thủ công, không còn gọi trong pipeline chính |
| `remove_object(object_id)` | Gửi lệnh REMOVE vào scene |
| `cleanup_stale_objects()` | Dọn object hết hạn (quá max_object_age_sec) |
| `publish_status(text, force)` | Publish `/moveit_scene_status` |
| `validate_collision_object(obj) → (ok, reason)` | Kiểm tra object hợp lệ (primitives, quaternion) |

### Trạng thái scene (status keywords)

| Trạng thái | Ý nghĩa |
|------------|---------|
| `SCENE_OBJECT_APPLIED` | Object đã được apply vào scene |
| `SCENE_OBJECT_CONFIRMED` | `/apply_planning_scene` đã trả success cho ADD |
| `SCENE_OBJECT_REMOVED` | Object đã remove khỏi scene |
| `SCENE_OBJECT_CONFIRMED_REMOVED` | `/apply_planning_scene` đã trả success cho REMOVE |
| `SCENE_EMPTY` | Scene không có object người |
| `SCENE_READY` | Scene sẵn sàng |

### Topics & Services

| Tên | Type | Chiều | Mô tả |
|----|------|-------|-------|
| `/human_collision_object` | `CollisionObject` | Subscribe | Object đầu vào |
| `/moveit_scene_status` | `String` | Publish | Trạng thái scene |
| `/apply_planning_scene` | Service | Client | Apply scene |
| `/get_planning_scene` | Service | Client | Chỉ dùng cho helper confirm/debug hoặc ACM debug |

---

## 10. Module: `planner_ab_replan_node.py`

**Node name:** `planner_ab_replan_node` (`ur3_fixed_joint_ab_node`)
**Chức năng:** Lập kế hoạch + thực thi robot UR3 đi A→B→A liên tục, có kiểm tra an toàn với người.

### Class: `UR3FixedJointABNode`

#### Khởi tạo & Setup

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load params, kết nối MoveIt, khởi tạo `AStarImproved3D` (ARA*), build waypoints |
| `create_move_group() → MoveGroupCommander` | Kết nối MoveIt với retry timeout |
| `get_safety_link_names() → List[str]` | Lấy danh sách link robot để kiểm tra an toàn |
| `build_path_a_to_b() → List[JointMap]` | Định nghĩa 7 waypoints khớp A→B |
| `resample_joint_path(path, target_count) → List[JointMap]` | Nội suy lại path thành N waypoints đều nhau |

#### Nhận dữ liệu người

| Phương thức | Chức năng |
|-------------|-----------|
| `human_callback(msg)` | Nhận `PointStamped` (legacy mode) |
| `human_skeleton_callback(msg)` | Nhận `PoseArray` skeleton, chọn điểm cho ARA* |
| `moveit_scene_status_callback(msg)` | Nhận trạng thái MoveIt scene |
| `decode_skeleton_pose_array(msg) → Dict[int, Point]` | Decode canonical 51/9-pose schema → dict |
| `select_planner_human_points(skeleton_by_id) → List[Point]` | Chọn tập điểm người để dùng trong ARA* và safety check |

#### ARA* Guard — `AStarImproved3D` (kiểm tra đường đi)

| Phương thức | Chức năng |
|-------------|-----------|
| `astar_path_is_available(joint_map) → bool` | Chạy ARA*, kiểm tra có đường đến waypoint không; replan nếu goal không đổi |
| `astar_path_is_available_to_pose(target_pose) → bool` | Chạy ARA* guard trực tiếp cho pose detour trước OMPL |
| `active_human_voxels() → Set[Voxel]` | Voxel bị người chiếm = quanh khớp (ổn định qua cache) ∪ dọc bones |
| `_bone_obstacle_voxels() → Set[Voxel]` | Inflate voxel dọc đoạn nối khớp (`bone_connection_pairs`) → A* thấy chi/thân |
| `stable_human_points() → List[Point]` | Obstacle stability cache: giữ anchor khi điểm lệch ≤ `obstacle_stability_threshold`, grace hold `obstacle_cache_hold_sec`, loại điểm invalid → chống drop voxel A* |
| `render_astar_markers()` / `_obstacle_timer_cb()` | Timer `obstacle_publish_rate` Hz: luôn build + publish obstacle độc lập chu kỳ plan & khoảng cách robot |
| `store_plan_viz(start, goal, path)` | Lưu overlay path/start/goal plan gần nhất (thread-safe) cho renderer |
| `PlanLogger` (class) + `_nearest_human_xyz()` / `_joint_map_goal_xyz()` | Ghi log sự kiện plan (OK/BLOCKED/MOVEIT_FAILED/NOT_EXECUTED/TOO_CLOSE/EXECUTE_*) ra `logs/` kèm tọa độ + lý do |
| `BreadcrumbCache` (class) + `_record_breadcrumb_if_due()` / `_try_breadcrumb_hop()` | Nhớ pose đã đi (joint+TCP, 0.5s), trim loop-closure, cap; khi blocked hop tới node cache còn-valid để warm-start (qua đủ safety gate) |
| `_setup_region_preference()` | FK waypoint A→B → bbox vùng; penalize voxel ngoài vùng (soft) để A* ưu tiên vùng shoulder_pan (cable limit) |
| `_setup_pan_limit()` | HARD: JointConstraint giới hạn `shoulder_pan` trong dải sweep → mọi MoveIt plan giữ pan trong vùng (chống đứt dây khí nén) |
| `inflate_voxel(center, radius) → Set[Voxel]` | Giãn nở voxel: cầu (`obstacle_inflate_sphere`, giảm ~3.8× voxel) hoặc cube đặc |
| `world_to_voxel(x, y, z) → Voxel` | Chuyển tọa độ thế giới → voxel index |
| `voxel_to_world(voxel) → (x, y, z)` | Chuyển ngược voxel → tọa độ |
| `current_tcp_voxel() → Voxel` | Voxel hiện tại của TCP robot |
| `publish_astar_markers(start, goal, path, obstacles)` | Visualize ARA* path trong RViz |

#### Forward Kinematics (FK)

| Phương thức | Chức năng |
|-------------|-----------|
| `target_tcp_pose_for_joint_map(joint_map) → Optional[PoseStamped]` | FK cho waypoint đích |
| `fk_pose_for_joint_positions(names, positions) → Optional[PoseStamped]` | FK cho trạng thái khớp bất kỳ |
| `fk_poses_for_joint_positions(names, positions, link_names) → List[PoseStamped]` | FK nhiều links |
| `fk_poses_for_robot_state(robot_state, link_names) → List[PoseStamped]` | FK cho robot state |

#### Điểm người

| Phương thức | Chức năng |
|-------------|-----------|
| `filter_human_points_near_link_poses(points, link_poses, context, threshold) → List[Point]` | Helper false-positive filter giữ lại, không còn gọi trong pipeline chính |
| `filter_human_points_near_plan_path(points, plan, sample_indexes) → List[Point]` | Helper false-positive filter giữ lại, không còn gọi trong pipeline chính |
| `latest_human_points() → List[Point]` | Lấy điểm người raw từ skeleton hoặc legacy point |

#### Kiểm tra an toàn trajectory

| Phương thức | Chức năng |
|-------------|-----------|
| `trajectory_is_safe(plan) → bool` | Duyệt trajectory, kiểm tra khoảng cách robot-người ≥ clearance |
| `current_robot_hand_min_distance() → Optional[(float, str)]` | Khoảng cách tối thiểu robot-người hiện tại |
| `trajectory_speed_margin(prev_poses, curr_poses, dt) → float` | Tính thêm margin an toàn theo tốc độ robot |
| `hand_blocks_target_pose(pose) → bool` | Tay người có chặn waypoint đích không? |
| `plan_and_execute_detour_with_retry(pose, label) → bool` | Chạy detour tối đa số attempts cấu hình, có backoff |
| `hold_until_waypoint_clear(target_pose, index) → bool` | HOLD không replan sau khi hết attempts; poll tới khi waypoint clear |
| `run_detour_if_hand_blocks_waypoint(joint_map, index) → bool` | Chạy detour +Z hoặc chuyển HOLD nếu detour thất bại |

#### Lập kế hoạch & Thực thi

| Phương thức | Chức năng |
|-------------|-----------|
| `plan_to_joint_map(joint_map) → Optional[Plan]` | Plan đến 1 waypoint joint |
| `plan_to_pose(pose_goal, planning_time=None) → Optional[Plan]` | ARA* gate rồi plan pose với timeout tùy chọn, sau đó restore timeout chính |
| `execute_plan(plan) → bool` | Thực thi plan + emergency stop nếu người quá gần |
| `retime_plan(plan) → Plan` | Retime trajectory theo velocity/acceleration scale |
| `run_waypoint(joint_map, index, total, direction) → bool` | Chạy 1 waypoint với replan loop |
| `run_path(path, direction) → bool` | Chạy toàn bộ path |
| `spin()` | Vòng lặp A→B→A |

#### Điều kiện cho phép plan

| Phương thức | Chức năng |
|-------------|-----------|
| `can_plan_to_waypoint() → bool` | Kiểm tra scene OK; không còn chặn khi skeleton thiếu/stale |
| `moveit_scene_ready() → bool` | Scene MoveIt có sẵn sàng chưa? |
| `scene_ready_for_planning(human_active) → bool` | Kết hợp: sync enabled + scene status |
| `scene_status_is_fresh() → bool` | Status nhận được có còn mới không? |

### Flow lập kế hoạch

```
spin() loop (A → B → A):
  └─► run_path(path_a_to_b)
        └─► run_waypoint(joint_map, idx)
              │
              ├─ run_detour_if_hand_blocks_waypoint()
              │    └─ hand_blocks_target_pose()
              │         ├─ ARA* gate → tối đa 5 detour attempts
              │         └─ fail → HOLD/poll tới khi clear
              │
              └─ loop: plan_to_joint_map(joint_map)
                    │
                    ├─ can_plan_to_waypoint()
                    │    └─ scene_ready_for_planning()?
                    │
                    ├─ astar_path_is_available(joint_map)
                    │    ├─ active_human_voxels()
                    │    ├─ AStarImproved3D.plan_with_info() / replan_with_info()
                    │    │    (ARA*: epsilon_start→final, time budget ara_max_time_ms)
                    │    └─ path found? → continue
                    │
                    ├─ MoveIt group.plan() [OMPL]
                    │
                    ├─ trajectory_is_safe(plan)
                    │    └─ FK each trajectory sample → distance check
                    │
                    └─ execute_plan(plan)
                         └─ monitor loop: current_robot_hand_min_distance()
                              └─ distance < required? → group.stop() → return False
```

### Topics & Services

| Tên | Type | Chiều | Mô tả |
|----|------|-------|-------|
| `/human_skeleton_base` | `PoseArray` | Subscribe | Skeleton người |
| `/moveit_scene_status` | `String` | Subscribe | Trạng thái scene |
| `/ur3_fixed_joint_path_status` | `String` | Publish | Trạng thái planner |
| `/hrc_path_text` | `String` | Publish | ARA* path/status dạng text (`ASTAR_OK ...` / `NO_ASTAR_PATH ...`) |
| `/hrc_astar_voxel_markers` | `MarkerArray` | Publish | ARA* voxel visualization |
| `/hrc_planning_time_ms` | `Float32` | Publish | Tổng thời gian plan |
| `/hrc_astar_planning_time_ms` | `Float32` | Publish | Thời gian ARA* |
| `/hrc_moveit_planning_time_ms` | `Float32` | Publish | Thời gian MoveIt |
| `/hrc_execution_time_ms` | `Float32` | Publish | Thời gian thực thi |
| `/compute_fk` | Service | Client | Forward kinematics |

---

## 11. Module: `astar_improved_3d.py`

**Chức năng:** Thuật toán **ARA\* (Anytime Repairing A\*)** trong không gian voxel 3D. Thư viện thuần Python, không phải ROS node. **Đây là guard planner đang dùng** (`planner_ab_replan_node.py` import `AStarImproved3D`).

### Class: `AStarImproved3D`

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__(size_x, size_y, size_z, diagonal, epsilon_start, epsilon_final, epsilon_decay, max_time_ms, max_steps)` | Khởi tạo lưới voxel + ARA* params |
| `plan_with_info(start, goal, obstacles) → PlanResult` | Lập kế hoạch lần đầu (ARA*: nhiều lần _improve_path với ε giảm dần) |
| `replan_with_info(new_start, obstacles) → PlanResult` | Tái lập kế hoạch khi obstacle thay đổi (reuse INCONS) |
| `plan(start, goal, obstacles) → List[Voxel]` | Wrapper trả về path list |
| `replan(new_start, obstacles) → List[Voxel]` | Wrapper replan trả về path list |
| `neighbors(s) → List[Voxel]` | Lấy voxel kề (6 hoặc 26 hướng) |
| `cost(a, b) → float` | Chi phí di chuyển giữa 2 voxel kề |
| `heuristic(a, b) → float` | Ước lượng khoảng cách Euclidean đến goal |
| `path_cost(path) → float` | Tổng chi phí path |
| `filter_obstacles(obstacles) → (Set, valid_count, invalid_count)` | Lọc và validate voxel obstacle |

### ARA* hoạt động

```
plan_with_info():
  ε = epsilon_start  (ví dụ 3.0)
  while ε ≥ epsilon_final AND time_budget còn:
      _improve_path(ε)    ← weighted A* với heuristic×ε
      ε -= epsilon_decay  ← giảm ε → solution tốt hơn
      nếu path tìm được → rebuild, tiếp tục cải thiện
  trả về PlanResult với path tốt nhất tìm được + epsilon_satisfied

replan_with_info():
  Dùng lại INCONS từ lần plan trước
  Chỉ update các voxel thay đổi (changed_obstacle_count)
  Chạy _ara_search() mới từ start mới
```

### Dataclass

```python
@dataclass
class PlanResult:           # (cũng export là DStarResult để tương thích)
    path: List[Voxel]
    success: bool
    reason: str             # OK / NO_PATH / GOAL_BLOCKED / START_BLOCKED /
                            # MAX_STEPS_REACHED / TIMEOUT / INVALID_* / ...
    metrics: Dict[str, object]  # expanded_steps, epsilon_satisfied, elapsed_ms,
                                # changed_obstacle_count, ...
```

### Tham số ARA* (khởi tạo)

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `epsilon_start` | `3.0` | Hệ số heuristic ban đầu (càng cao → tìm nhanh nhưng không tối ưu) |
| `epsilon_final` | `1.0` | Hệ số heuristic tối thiểu (1.0 = A* chính xác) |
| `epsilon_decay` | `0.5` | Giảm ε mỗi iteration |
| `max_time_ms` | `50.0` | Ngân sách thời gian tối đa mỗi lần plan |
| `max_steps` | `50000` | Số bước expand tối đa |

### Hằng số reason

| Hằng | Mô tả |
|------|-------|
| `OK` | Tìm được đường |
| `NO_PATH` | Không tìm được đường |
| `GOAL_BLOCKED` | Goal bị obstacle |
| `START_BLOCKED` | Start bị obstacle |
| `GOAL_OUT_OF_BOUNDS` | Goal ngoài biên voxel map |
| `START_OUT_OF_BOUNDS` | Start ngoài biên voxel map |
| `MAX_STEPS_REACHED` | Vượt quá bước tối đa |
| `TIMEOUT` | Hết thời gian ngân sách |
| `PATH_EXTRACTION_FAILED` | Không reconstruct được path |
| `REPLAN_NOT_INITIALIZED` | Gọi replan trước khi plan lần đầu |
| `EMPTY_GRID` | Lưới voxel không hợp lệ |

---

## 11b. Module: `astar_lpa_3d.py`

**Chức năng:** Thuật toán **LPA\* (Lifelong Planning A\*)** trong voxel grid 3D. Thư viện thuần Python (không ROS node). Guard planner **thay thế** cho `AStarImproved3D`, **drop-in cùng API** (`plan_with_info` / `replan_with_info` / `plan` / `replan` / `set_penalty_cells`, cùng `PlanResult` + reason constants — import từ `astar_improved_3d`). Chọn bằng param `~guard_planner_type` trong Module F. Mặc định Module F vẫn dùng `ara_star`; đặt `lpa_star` để bật LPA*.

**Ý tưởng:** giữ lại search cũ, khi obstacle người thay đổi cục bộ thì chỉ **repair** các cell bị ảnh hưởng (`g`/`rhs`, update_vertex) thay vì plan lại từ đầu. Khi goal đổi (Module F tự gọi `plan_with_info`) hoặc start dịch chuyển hoặc obstacle diff quá lớn → **reset** (`initialize_search`).

### Class: `LPAStar3D`

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__(size_x, size_y, size_z, diagonal, max_time_ms, max_steps, smooth, epsilon, start_reuse_radius_voxels, max_changed_obstacles_for_repair)` | Khởi tạo grid + LPA* params |
| `plan_with_info(start, goal, obstacles) → PlanResult` | RESET: `initialize_search` + `compute_shortest_path` |
| `replan_with_info(new_start, obstacles) → PlanResult` | Giữ goal nội bộ; REPAIR (`update_obstacles`) nếu start không đổi và diff nhỏ, ngược lại RESET |
| `plan` / `replan` | Wrapper trả path list |
| `set_penalty_cells(cells, weight)` | Soft region-preference, tương thích Module F (gộp vào cost cạnh) |
| `update_vertex(u)` / `compute_shortest_path()` / `update_obstacles(new)` / `reconstruct_path()` | Core LPA* |

### Metrics (`PlanResult.metrics`)

`algorithm="LPA*"`, `success`, `reason`, `path_length`, `expanded_steps`, `planning_time_ms`, `obstacle_count`, `changed_obstacle_count`, `reuse_mode` (`RESET`/`REPAIR`).

### Tham số LPA* (khởi tạo)

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `epsilon` | `1.0` | 1.0 = tối ưu. >1 (weighted) để sau |
| `max_time_ms` / `max_steps` | từ `ara_max_time_ms` / `ara_max_steps` | Ngân sách dùng chung với ARA* |
| `start_reuse_radius_voxels` | `1` | **Reserved**: v1 chỉ REPAIR khi start không đổi; mọi dịch chuyển start → RESET (re-root để v2) |
| `max_changed_obstacles_for_repair` | `500` | Diff > ngưỡng → RESET (repair đắt hơn reset). Cần calibrate theo grid thật |

### Giới hạn đã biết (v1)

- Reason codes external giống ARA* (reuse hằng số từ `astar_improved_3d`); phân biệt reset/repair nằm ở `reuse_mode`.
- Khác ARA* (anytime): hết ngân sách → trả `TIMEOUT` **không có path**, không trả best-so-far. Với grid lớn cần đặt `ara_max_time_ms` đủ rộng.
- REPAIR khi diff lớn có thể chậm hơn RESET → dùng `max_changed_obstacles_for_repair`.
- Chưa có: shadow mode, weighted-ε, D* Lite, benchmark harness (xem spec `docs/superpowers/specs/2026-06-20-lpa-star-3d-guard-planner-design.md`).

**Test:** `python3 scripts/test_astar_lpa_3d.py` (6 case: empty/static/start-blocked/goal-blocked/repair/reset, cross-check cost với ARA*).

---

## 12. ROS Topics & TF

### Single Kinect — Tổng hợp topics

```
                    kinect_skeleton_tracker
                          │
    ┌─────────────────────┼──────────────────────┐
    │                     │                      │
/human_skeleton_camera  /human_skeleton_base  /human_skeleton_status
(PoseArray)             (PoseArray)            (String)
    │                     │
/kinect_skeleton/     /kinect_raw/
image_raw             image_raw
(Image,debug)         (Image,raw)
                          │
                    skeleton_obstacle_builder
                          │
    ┌─────────────────────┘
    │
/human_collision_object          /human_obstacle_status
(CollisionObject)                (String)
    │
moveit_scene_manager
    │
/moveit_scene_status
(String)
    │                              planner_ab_replan_node
    └──────────────────────────────────────────────────────
                                        │
    ┌───────────────────────────────────┼────────────────────┐
    │                                   │                    │
/ur3_fixed_joint_path_status    /hrc_astar_voxel_markers  /hrc_*_time_ms
(String)                        (MarkerArray)              (Float32 x4)
```

### Dual Kinect — Tổng hợp topics

```
kinect_skeleton_tracker (ns: kinect_front)
    ├── /kinect_front/human_skeleton_camera   (PoseArray)
    ├── /kinect_front/human_skeleton_base     (PoseArray)
    ├── /kinect_front/human_skeleton_status   (String)
    ├── /kinect_front/kinect_raw/image_raw    (Image)
    └── /kinect_front/kinect_skeleton/image_raw (Image)

kinect_skeleton_tracker (ns: kinect_back)
    ├── /kinect_back/human_skeleton_base      (PoseArray) ─┐
    ├── /kinect_back/human_skeleton_camera    (PoseArray)  │
    ├── /kinect_back/human_skeleton_status    (String)     │
    └── /kinect_back/kinect_skeleton/image_raw (Image)     │
                                                           │
dual_kinect_fusion_controller ◄──────────────────────────────┘
    │  [Timer-driven 20 Hz: SkeletonFuser + DutyScheduler]
    ├── /human_skeleton_base              (PoseArray, fused — byte-compatible)
    ├── /human_skeleton_fusion_status     (String: mode + duty state + joint counts)
    ├── /kinect_front/tracker_rate_cmd    (Float32, Hz — duty command front)
    └── /kinect_back/tracker_rate_cmd     (Float32, Hz — duty command back)
```

### TF Tree

**Single Kinect:**
```
world
  └── base_link
        └── kinect2_link
              └── kinect2_ir_optical_frame
                    (robot links: shoulder_link, upper_arm_link, ...)
```

**Dual Kinect:**
```
world
  └── base_link
        ├── kinect_front_link
        │     └── kinect_front_ir_optical_frame
        ├── kinect_back_link
        │     └── kinect_back_ir_optical_frame
        └── (robot links)
```

---

## 13. Launch Files

### `system.launch` — Single Kinect

```
Nodes khởi động:
  1. static_transform_publisher × 2  (base_link → kinect2_link + kinect2_ir_optical)
  2. kinect_skeleton_tracker           [MODULE A]
  3. skeleton_obstacle_builder         [MODULE D]
  4. moveit_scene_manager              [MODULE E]
  5. planner_ab_replan_node            [MODULE F]

Cấu hình đặc trưng:
  - enable_yolo = true, holistic_complexity = 2
  - person_select_mode=nearest, nearest_min_depth_px=50; YOLO async bị tắt nếu nearest RGB-D
  - selected mask che RGB trước Holistic; body_min_core_points=8 bắt buộc đủ vai/khuỷu/cổ tay/hông
  - rgb_only_mode=false, enable_depth=true (default RGB-D); có thể chạy RGB-only bằng rgb_only_mode:=true enable_depth:=false
  - skeleton robot filter không còn chạy trong code
  - rate_hz = 30 Hz
  - tracked_joint_ids: chỉ body (0,11-16,23-24) — KHÔNG có tay
  - connection_pairs: đủ 8 xương tay + thân
  - spatial filter bật; temporal jump tắt; hip clamp tắt
  - obstacle sphere/cylinder radius 0.05m, padding = 0
  - planner voxel_size = 0.04, human_inflate_radius = 1
  - human_safety_distance = 0.01, robot_body_radius = 0.02, human_body_radius = 0.02 → base_clearance = 0.04m
    (⚠ giảm theo yêu cầu user để obstacle mỏng — radius 3→1, ~17.6× ít voxel/khớp; **giảm cả biên an toàn** trajectory/emergency-stop)
  - bone obstacles: enable_bone_obstacles=true, bone_inflate_radius=0 (chỉ line, không phình ống)
  - resampled_waypoint_count = 5 (giảm từ 7 để ít MoveIt plan/leg)
  - ARA* params: ara_epsilon_start=3.0, ara_epsilon_final=1.0,
                 ara_epsilon_decay=0.5, ara_max_time_ms=50.0, ara_max_steps=50000
  - detour: max 5 attempts, 0.75s/attempt, backoff/HOLD poll 0.5s, +Z 0.15m
  - point false-positive filters giữ lại trong code nhưng không còn gọi trong pipeline chính
```

### `system_back.launch` — Single Kinect (BACK camera)

```
Biến thể single-cam của system.launch dùng Kinect phía sau (back).
Định danh camera + static TF lấy từ dual_kinect_system.launch (back camera).
Toàn bộ voxel / tốc độ / obstacle / moveit / planner copy y nguyên từ system.launch.

Khác system.launch:
  - camera_name = back, camera_serial = 501294742442
  - camera_packet_pipeline = cpu (theo cấu hình back trong dual; đổi sang cuda nếu cần nhanh hơn)
  - camera_frame = kinect_back_ir_optical_frame
  - debug_image_flip_horizontal = true
  - static TF: args="-2.46 0.26 1.44 1.5708 0.0 2.22" → kinect_back_link + kinect_back_ir_optical_frame
  - node TF đổi tên: base_to_kinect_back_link, base_to_kinect_back_ir_optical

Giữ nguyên như system.launch:
  - topic toàn cục /human_skeleton_base, /human_collision_object, /moveit_scene_status...
  - enable_yolo=true, holistic_complexity=2, rate_hz=30
  - planner voxel_size=0.04, human_inflate_radius=1, planner_ab_replan_node (loop=true)
```

### `dual_kinect_system.launch` — Dual Kinect

```
Nodes khởi động:
  1. static_transform_publisher × 4        (front + back links + optical frames)
  2. kinect_skeleton_tracker (ns: kinect_front)   [MODULE A] — rate_cmd_topic wired, 1–12 Hz range
  3. kinect_skeleton_tracker (ns: kinect_back)    [MODULE A] — rate_cmd_topic wired, 1–8 Hz range
  4. dual_kinect_fusion_controller                 [MODULE C] — timer 20 Hz, fuse + duty-cycle
  5. skeleton_obstacle_builder                     [MODULE D]
  6. moveit_scene_manager                          [MODULE E]
  7. planner_ab_replan_node                        [MODULE F]

Fusion: dual_kinect_fusion_controller thay thế multi_kinect_skeleton_controller (đã xóa).
  /human_skeleton_base có publisher trở lại → downstream obstacle/moveit/planner hoạt động bình thường.

Thay đổi so với bản cũ của launch:
  - Thêm node dual_kinect_fusion_controller với các params fusion + duty-cycle.
  - Thêm cho mỗi tracker: rate_cmd_topic, rate_min, rate_max (front 1–12 Hz, back 1–8 Hz).
  - Xóa 4 args chết không dùng: min_valid_joints, enable_yolo_person_segmentation,
    holistic_model_complexity, tracker_rate_hz.

Cấu hình đặc trưng (tracker giữ tuning dual, KHÔNG flatten về system.launch):
  - front: enable_yolo=true, rate ban đầu 10 Hz; back: enable_yolo=false, rate ban đầu 5 Hz (default LOW=2)
  - front/back default RGB-D: front_rgb_only_mode=false, front_enable_depth=true, back_rgb_only_mode=false, back_enable_depth=true
  - holistic_complexity = 0 (nhẹ, cả 2 tracker)
  - tracked_joint_ids: 9 body joints (0,11,12,13,14,15,16,23,24)
  - detour: max 5 attempts, 0.75s/attempt, backoff/HOLD poll 0.5s, +Z 0.20m
  - enable_hand_detection = false (default)
  - spatial filter bật trên từng tracker; temporal jump tắt; hip clamp tắt
  - obstacle sphere/cylinder radius 0.03/0.05m, padding = 0
  - planner giữ safety margin dual (voxel 0.065, human_safety_distance 0.15, human_body_radius 0.08)
    — rộng hơn system.launch, KHÔNG hạ theo §10 safety
  - scaffolding dual giữ nguyên: LIBFREENECT2 env, respawn, reopen_on_frame_timeout, launch-prefix sleep 6
  - ARA* params: ara_epsilon_start=3.0, ara_epsilon_final=1.0,
                 ara_epsilon_decay=0.5, ara_max_time_ms=50.0, ara_max_steps=50000
```

### `back_kinect_perception_test.launch`

Test riêng Kinect phía sau (back) để kiểm tra nhận diện trước khi chạy hệ thống đầy đủ.
Hỗ trợ `rgb_only_mode` và `enable_depth` giống single tracker để test RGB-only/back-camera fallback.

### `test/test_spatial_arm_filter.py`

**File role:** unit test.
**Main inputs:** skeleton dict camera-frame giả lập.
**Main outputs:** assertions cho median, jump reject, arm-length lock, hip clamp và reset.
**Related module:** Module A — perception/skeleton filtering.

### `test/test_pose_array_schema.py`

**File role:** regression unit test.
**Main inputs:** tracker 51-pose và fusion 9-pose messages giả lập.
**Main outputs:** assertions canonical numeric joint mapping và builder subset selection.
**Related modules:** B, C, D, F.

### `test/test_collision_visualization.py`

**File role:** regression unit test.
**Main inputs:** CollisionObject sphere/cylinder giả lập.
**Main outputs:** assertions marker geometry và PlanningScene ObjectColor đỏ.
**Related modules:** D, E, RViz.

### `rviz.launch`

Khởi động RViz với cấu hình sẵn hiển thị: robot, marker obstacle người màu đỏ, voxel ARA*, planning scene.

---

## 14. So sánh Single Kinect vs Dual Kinect

| Tiêu chí | Single Kinect | Dual Kinect |
|----------|---------------|-------------|
| **Số camera** | 1 | 2 (front + back) |
| **Launch file** | `system.launch` | `dual_kinect_system.launch` |
| **Rate tracker** | 30 Hz | front 2–10 Hz (duty), back 2–6 Hz (duty) |
| **Rate đầu ra** | 30 Hz | 20 Hz (timer dual_kinect_fusion_controller) |
| **Joint tracking** | Body only (9 joints) | Body only (9 joints, dual fused) |
| **YOLO** | Bật (better occlusion) | Front bật, Back tắt (nhẹ hơn) |
| **Holistic complexity** | 2 (chính xác) | 0 (nhanh) |
| **Robot filter** | Bật | Tắt (mặc định) |
| **Fusion module** | Không cần | `dual_kinect_fusion_controller` (MODULE C) |
| **Side swap detection** | Không cần | Không (bỏ trong node mới) |
| **Duty-cycle** | Không | DutyScheduler điều phối back rate |
| **Vùng phủ** | 1 phía | 360° |
| **Điểm blind spot** | Nhiều khi người quay lưng | Giảm đáng kể |
| **TF frames** | 1 camera frame | 2 camera frames |
| **Topics skeleton** | `/human_skeleton_base` trực tiếp | Qua fusion → `/human_skeleton_base` |
| **Debug topics** | 1 bộ image | 2 bộ image + fusion_status String |

### Khi nào dùng Single / Dual?

```
Single Kinect:
  ✓ Môi trường đơn giản, 1 hướng làm việc
  ✓ CPU/GPU hạn chế (dùng YOLO complexity=2)
  ✓ Debug nhanh, cấu hình đơn giản
  ✗ Blind spot khi người quay lưng

Dual Kinect:
  ✓ Theo dõi người 360° đầy đủ
  ✓ Cần track tay để tránh va chạm chính xác hơn
  ✓ Môi trường phức tạp, nhiều hướng tiếp cận
  ✗ Cần 2 USB 3.0 controller riêng biệt
  ✗ Cần hiệu chỉnh eye-to-hand calibration cho cả 2 camera
```

---

## Calibration Files (Kinect serials)

| Serial | Vị trí | Files |
|--------|--------|-------|
| `196605135147` | Kinect **FRONT** | `calib_color.yaml`, `calib_depth.yaml`, `calib_ir.yaml`, `calib_pose.yaml` |
| `299150235147` | Kinect **BACK** | `calib_color.yaml`, `calib_ir.yaml`, `calib_pose.yaml` |

Đường dẫn: `src/iai_kinect2/kinect2_bridge/data/<serial>/`

---

## Repository Files

### `.gitignore`

**File role:** repository hygiene configuration.
**Main inputs:** generated caches, runtime logs, tests, documentation, and local editor/agent metadata.
**Main outputs:** clean source-only Git staging scope.
**Related module:** Documentation and project structure.
**File type:** configuration.

---

*Cập nhật: 2026-06-15 — Sync feat/dual-kinect-fusion-controller: dual_kinect_fusion_controller thay multi_kinect_skeleton_controller, duty-cycle rate control, tracker rate_cmd_topic, launch args cleaned up*
