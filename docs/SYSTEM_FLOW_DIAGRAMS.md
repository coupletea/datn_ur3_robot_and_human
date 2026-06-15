# Sơ đồ luồng hệ thống UR3 HRC Planner

> Cập nhật: 2026-05-29

---

## Mục lục

1. [Luồng tổng quát — Single Kinect](#1-luồng-tổng-quát--single-kinect)
2. [Luồng tổng quát — Dual Kinect](#2-luồng-tổng-quát--dual-kinect)
3. [Chi tiết MODULE A — kinect\_skeleton\_tracker](#3-chi-tiết-module-a--kinect_skeleton_tracker)
4. [Chi tiết MODULE B — data\_skeleton](#4-chi-tiết-module-b--data_skeleton)
5. [Chi tiết MODULE C — dual\_kinect\_fusion\_controller](#5-chi-tiết-module-c--dual_kinect_fusion_controller)
6. [Chi tiết MODULE D — skeleton\_obstacle\_builder](#6-chi-tiết-module-d--skeleton_obstacle_builder)
7. [Chi tiết MODULE E — moveit\_scene\_manager](#7-chi-tiết-module-e--moveit_scene_manager)
8. [Chi tiết MODULE F — planner\_ab\_replan\_node](#8-chi-tiết-module-f--planner_ab_replan_node)
9. [Chi tiết MODULE G — astar\_improved\_3d](#9-chi-tiết-module-g--astar_improved_3d)
10. [Luồng Safety — 5 lớp bảo vệ](#10-luồng-safety--5-lớp-bảo-vệ)
11. [ROS Topic Map](#11-ros-topic-map)

---

## 1. Luồng tổng quát — Single Kinect

```
                           ┌─────────────────────┐
                           │      👤  NGƯỜI       │
                           │   (vùng làm việc)    │
                           └──────────┬──────────┘
                                      │ RGB + Depth stream
                                      ▼
                           ┌─────────────────────┐
                           │    📷  Kinect v2     │
                           │  libfreenect2        │
                           │  USB 3.0             │
                           └──────────┬──────────┘
                                      │ raw frames
                                      ▼
                    ┌─────────────────────────────────────┐
                    │           MODULE A                  │
                    │    kinect_skeleton_tracker          │
                    │                                     │
                    │  RGB+Depth                          │
                    │    → YOLO person segmentation       │
                    │    → nearest mask by median depth    │
                    │    → mask RGB → MediaPipe Holistic  │
                    │    → extract 3D landmarks           │
                    │    → require 8 core joints           │
                    │    → SpatialArmFilter               │
                    │    → TF transform → base_link       │
                    │    → TemporalSkeletonFilter (EMA)   │
                    └──────────────┬──────────────────────┘
                                   │
                    /human_skeleton_base  (PoseArray)
                                   │
             ┌─────────────────────┼──────────────────────┐
             │                                            │
             ▼                                            ▼
┌────────────────────────┐                 ┌──────────────────────────────┐
│       MODULE D         │                 │          MODULE F             │
│ skeleton_obstacle_     │                 │    planner_ab_replan_node    │
│ builder                │                 │                              │
│                        │                 │  subscribe:                  │
│  joints → Spheres      │                 │  /human_skeleton_base        │
│  bones  → Cylinders    │                 │  /moveit_scene_status        │
│  fixed radius 0.05m    │                 │                              │
└────────────┬───────────┘                 └──────────────┬───────────────┘
             │                                            │
  /human_collision_object                    gọi nội bộ  │
  (CollisionObject)                                       │
  /human_collision_markers (đỏ, cùng geometry → RViz)     │
             │                                            ▼
             ▼                              ┌──────────────────────────────┐
┌────────────────────────┐                 │          MODULE G             │
│       MODULE E         │                 │     astar_improved_3d        │
│ moveit_scene_manager   │                 │         (ARA*)               │
│                        │                 │                              │
│  validate object       │                 │  voxel 3D obstacle map       │
│  /apply_planning_scene │                 │  kiểm tra đường đi           │
│  service call          │                 │  → OK / BLOCKED              │
└────────────┬───────────┘                 └──────────────┬───────────────┘
             │                                            │
  /moveit_scene_status                          path result
  (String)                                               │
             │                                           │
             └─────────────────┬─────────────────────────┘
                               │
                               ▼
              ┌──────────────────────────────────────┐
              │         MoveIt OMPL Planner          │
              │                                      │
              │  group.plan()  → RobotTrajectory     │
              │  trajectory_is_safe() check          │
              │  group.execute() + monitor loop      │
              └──────────────────┬───────────────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │      🦾  UR3 Robot     │
                    │    A → B → A cycle     │
                    │    tránh người an toàn │
                    └────────────────────────┘
```

---

## 2. Luồng tổng quát — Dual Kinect

```
     ┌────────────────────┐          ┌────────────────────┐
     │   📷  Kinect FRONT │          │   📷  Kinect BACK  │
     │  device_index = 0  │          │  device_index = 1  │
     │  Serial:196605...  │          │  Serial:299150...  │
     └────────┬───────────┘          └─────────┬──────────┘
              │ RGB+Depth                       │ RGB+Depth
              ▼                                 ▼
 ┌────────────────────────┐       ┌──────────────────────────┐
 │       MODULE A         │       │       MODULE A            │
 │  kinect_skeleton_      │       │  kinect_skeleton_         │
 │  tracker               │       │  tracker                  │
 │  (ns: kinect_front)    │       │  (ns: kinect_back)        │
 │                        │       │                           │
 │  rate: duty 2–10 Hz    │       │  rate: duty 2–6 Hz        │
 │  YOLO: ON              │       │  YOLO: OFF                │
 │  complexity: 0         │       │  complexity: 0            │
 │  rate_cmd_topic wired  │       │  rate_cmd_topic wired     │
 └────────────┬───────────┘       └────────────┬──────────────┘
              │                                │
 /kinect_front/human_skeleton_base    /kinect_back/human_skeleton_base
              │                                │
              └─────────────┬──────────────────┘
                            │ _front_cb() / _back_cb() → cache
                            ▼
          ┌─────────────────────────────────────────────┐
          │                 MODULE C                    │
          │    dual_kinect_fusion_controller            │
          │                                             │
          │  Timer-driven 20 Hz:                        │
          │                                             │
          │  SkeletonFuser (per-joint occlusion-fill):  │
          │  1. freshness check (max_input_age_sec)     │
          │  2. both fresh + close → average            │
          │  3. both fresh + far  → front (primary)     │
          │  4. front-only        → front               │
          │  5. back-only         → back  (fill)        │
          │  6. both stale        → NaN                 │
          │                                             │
          │  DutyScheduler (back rate duty-cycle):      │
          │  7. count front_missing joints              │
          │  8. boost back HIGH if miss sustained       │
          │  9. hysteresis hold, then back LOW          │
          └──────────────────┬──────────────────────────┘
                             │
              /human_skeleton_base  (PoseArray — fused, 20 Hz)
              /human_skeleton_fusion_status  (BOTH/FILL_FROM_BACK/FRONT_ONLY/BACK_ONLY/NO_INPUT)
              /kinect_front/tracker_rate_cmd  (Float32, Hz)
              /kinect_back/tracker_rate_cmd   (Float32, Hz)
                             │
          ┌──────────────────┼───────────────────────┐
          │                                          │
          ▼                                          ▼
┌─────────────────────┐                ┌──────────────────────────────┐
│      MODULE D       │                │          MODULE F             │
│  skeleton_obstacle_ │                │    planner_ab_replan_node    │
│  builder            │                └──────────────────────────────┘
└────────┬────────────┘                             │ (giống Single Kinect
         │                                          │  từ đây trở xuống)
         ▼                                          ▼
┌─────────────────────┐                ┌──────────────────────────────┐
│      MODULE E       │                │          MODULE G             │
│  moveit_scene_      │                │     astar_improved_3d        │
│  manager            │                └──────────────────────────────┘
└────────┬────────────┘                             │
         │                                          │
         └──────────────┬─────────────────────────── ┘
                        ▼
           ┌────────────────────────┐
           │      🦾  UR3 Robot     │
           │   theo dõi 360°        │
           └────────────────────────┘
```

---

## 3. Chi tiết MODULE A — kinect\_skeleton\_tracker

```
  ┌───────────────────────────────────────────────────────────────────────┐
  │                    spin()  —  vòng lặp 30 Hz                         │
  └───────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────▼────────────────┐
                    │         open_camera()           │
                    │  libfreenect2 connect Kinect    │
                    │  device_index hoặc serial       │
                    └───────────────┬────────────────┘
                                    │
                    ┌───────────────▼────────────────┐
                    │     read_registered_frames()    │
                    │  đọc RGB + Depth đã align       │
                    │  timeout_ms kiểm tra            │
                    └──────────┬──────────┬──────────┘
                               │          │
                         Bình thường   TIMEOUT
                               │          │
                               │          ▼
                               │  ┌─────────────────────────────┐
                               │  │  maybe_reopen_camera_       │
                               │  │  after_timeout()            │
                               │  │                             │
                               │  │  thử reopen camera          │
                               │  │  nếu thất bại liên tiếp     │
                               │  │  → os._exit(75)             │
                               │  │  → roslaunch respawn node   │
                               │  └──────────────┬──────────────┘
                               │                 │ reopen OK
                               └────────┬────────┘
                                        │
               ┌────────────────────────▼─────────────────────────┐
               │  enable_yolo_person_segmentation ?                │
               └──────────────┬─────────────────────┬─────────────┘
                           YES │                     │ NO
                              ▼                      ▼
             ┌────────────────────────┐   ┌──────────────────────┐
             │  run_yolo_person_      │   │  không có mask        │
             │  segmentation()        │   │  dùng toàn bộ frame   │
             │                        │   └──────────┬───────────┘
             │  detect person boxes   │              │
             │  + masks               │              │
             └──────────┬─────────────┘              │
                        │                            │
                        ▼                            │
             ┌──────────────────────┐                │
             │  select_best_person_ │                │
             │  mask()              │                │
             │  lớn nhất / center   │                │
             └──────────┬───────────┘                │
                        └─────────────┬──────────────┘
                                      │
                      ┌───────────────▼────────────────┐
                      │       run_holistic()            │
                      │  MediaPipe Holistic inference   │
                      │  RGB → pose + hand results      │
                      └───────────────┬────────────────┘
                                      │
                      ┌───────────────▼────────────────┐
                      │  extract_pose_landmarks_3d()   │
                      │                                │
                      │  UV pixel (MediaPipe)          │
                      │  → tra depth map → Z coord     │
                      │  → lọc theo YOLO mask          │
                      │  → {joint_id: (x,y,z)}         │
                      │    camera frame                │
                      └───────────────┬────────────────┘
                                      │
                      ┌───────────────▼────────────────┐
                      │    validate_body_geometry()    │
                      │                                │
                      │  shoulder_width_range OK?      │
                      │  torso_length_range OK?        │
                      │  min_core_points đủ?           │
                      └────────┬────────────┬──────────┘
                           OK  │            │ FAIL
                               │            ▼
                               │  ┌─────────────────────┐
                               │  │  publish_empty_      │
                               │  │  skeleton()          │
                               │  │  → next frame        │
                               │  └─────────────────────┘
                               │
               ┌───────────────▼──────────────────────────────┐
               │         enable_hand_detection ?              │
               └────────────────┬───────────────┬─────────────┘
                             YES │               │ NO
                                ▼               │
             ┌────────────────────────────┐     │
             │  extract_hand_landmarks_   │     │
             │  3d()                      │     │
             │                            │     │
             │  Left hand  (ID 100-120)   │     │
             │  Right hand (ID 200-220)   │     │
             └──────────┬─────────────────┘     │
                        │                       │
                        ▼                       │
             ┌──────────────────────┐           │
             │  validate_hand_      │           │
             │  geometry()          │           │
             │  finger span, palm   │           │
             │  size range check    │           │
             └──────────┬───────────┘           │
                        │                       │
                        ▼                       │
             ┌──────────────────────┐           │
             │  attach_hands_to_    │           │
             │  pose_wrists()       │           │
             │  gắn hand vào wrist  │           │
             │  của pose body       │           │
             └──────────┬───────────┘           │
                        │                       │
                        ▼                       │
             ┌──────────────────────┐           │
             │  merge_pose_and_     │           │
             │  hands()             │           │
             │  body + left + right │           │
             │  → skeleton dict     │           │
             └──────────┬───────────┘           │
                        └───────────────────────┘
                                    │
                    ┌───────────────▼────────────────┐
                    │   SpatialArmFilter.update()    │
                    │                                │
                    │  median + per-joint jump       │
                    │  arm-length calibration/lock   │
                    │  hip clamp optional, default off│
                    └───────────────┬────────────────┘
                                    │
                    ┌───────────────▼────────────────┐
                    │   transform_skeleton_to_       │
                    │   target()                     │
                    │                                │
                    │  TF lookup:                    │
                    │  camera_frame → base_link      │
                    │  transform tất cả joints       │
                    │  → skeleton trong base_link    │
                    └───────────────┬────────────────┘
                                    │
                    ┌───────────────▼────────────────┐
                    │   TemporalSkeletonFilter.       │
                    │   update()                      │
                    │                                │
                    │  EMA smoothing (alpha)          │
                    │  confirm_frames trước publish   │
                    │  hold_frames khi mất detect     │
                    └─────────┬─────────────┬────────┘
                           OK │             │ REJECT
                              │             ▼
                              │   ┌──────────────────────┐
                              │   │  publish_empty_       │
                              │   │  skeleton()           │
                              │   │  PoseArray rỗng       │
                              │   │  /human_skeleton_base │
                              │   └──────────────────────┘
                              │
                    ┌─────────▼──────────────────────┐
                    │      publish_frame_result()     │
                    │                                │
                    │  /human_skeleton_camera        │
                    │  /human_skeleton_base          │
                    │  /kinect_skeleton/image_raw    │
                    │  /human_skeleton_status        │
                    └────────────────────────────────┘
                                    │
                                    └──► frame tiếp theo
```

### TemporalSkeletonFilter — trạng thái

```
  ┌────────────────┐   detect liên tiếp     ┌────────────────┐
  │  UNCONFIRMED   │  ≥ confirm_frames       │   CONFIRMED    │
  │                │ ─────────────────────► │                │
  │  chưa publish  │                         │  EMA smooth    │
  │                │ ◄───────────────────── │  publish OK    │
  └──────┬─────────┘   mất ≥ lost_frames     └───────┬────────┘
         │                                            │ mất detect
         │ jump        reset()                        │ < lost_frames
         │ > max ──────────────────────────►          ▼
         │                                  ┌────────────────┐
         │                                  │    HOLDING     │
         │                                  │                │
         │                                  │  publish frame │
         └──────────────────────────────────│  cuối cùng     │
                                            │  detect lại    │
                                            │  → CONFIRMED   │
                                            └────────────────┘
```

---

## 4. Chi tiết MODULE B — data\_skeleton

> Thư viện tiện ích — không phải ROS node. Import bởi MODULE A và MODULE C.

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │                      data_skeleton.py                               │
  │                 (utility library — no ROS node)                     │
  └─────────────────────────────────────────────────────────────────────┘

  ┌──────────────────────────────┐   ┌──────────────────────────────────┐
  │      CAMERA I/O              │   │         YOLO PIPELINE            │
  │                              │   │                                  │
  │  initialize_camera()         │   │  load_yolo_segment_model()       │
  │  ├─ device_index / serial    │   │  ├─ YOLOv8n-seg.pt              │
  │  └─ → freenect, device,      │   │  └─ GPU / CPU device            │
  │       listener, registration │   │                                  │
  │                              │   │  run_yolo_person_segmentation()  │
  │  read_registered_frames()    │   │  ├─ class_id = 0 (person)        │
  │  ├─ RGB + Depth aligned      │   │  ├─ conf, iou thresholds         │
  │  └─ raise KinectFrameTimeout │   │  └─ → list detections            │
  │     nếu quá timeout_ms       │   │                                  │
  │                              │   │  select_best_person_mask()       │
  │                              │   │  ├─ legacy: area / center        │
  │                              │   │  ├─ nearest: median depth        │
  │                              │   │  └─ min_area_ratio + fallback    │
  │                              │   │                                  │
  │                              │   │  dilate_mask()                   │
  │                              │   │  └─ morphological dilation px    │
  └──────────────────────────────┘   └──────────────────────────────────┘

  ┌──────────────────────────────┐   ┌──────────────────────────────────┐
  │    MEDIAPIPE PIPELINE        │   │       VALIDATION                 │
  │                              │   │                                  │
  │  load_mediapipe_holistic()   │   │  validate_body_geometry()        │
  │  └─ complexity 0 / 1 / 2     │   │  ├─ shoulder_width_range         │
  │                              │   │  ├─ torso_length_range           │
  │  run_holistic()              │   │  └─ min_core_points              │
  │  └─ RGB → pose + hand        │   │                                  │
  │       landmarks              │   │  validate_hand_geometry()        │
  │                              │   │  ├─ max_finger_span              │
  │  extract_pose_landmarks_3d() │   │  ├─ min_palm_size                │
  │  ├─ UV pixel → depth lookup  │   │  └─ max_palm_size                │
  │  ├─ → 3D (x,y,z)            │   │                                  │
  │  └─ min_visibility filter    │   │  validate_hand_geometry()        │
  │                              │   │  → (ok, reason)                  │
  │  extract_hand_landmarks_3d() │   │                                  │
  │  ├─ left hand  (ID 100-120)  │   └──────────────────────────────────┘
  │  ├─ right hand (ID 200-220)  │
  │  └─ min_valid_points filter  │   ┌──────────────────────────────────┐
  │                              │   │       MERGE & ENCODE             │
  └──────────────────────────────┘   │                                  │
                                     │  attach_hands_to_pose_wrists()   │
                                     │  └─ hand → wrist pose body       │
                                     │                                  │
                                     │  merge_pose_and_hands()          │
                                     │  └─ body+left+right → dict       │
                                     │                                  │
                                     │  skeleton_dict_to_pose_array()   │
                                     │  └─ dict → ROS PoseArray         │
                                     │     NaN cho joint thiếu          │
                                     │                                  │
                                     │  draw_filtered_skeleton()        │
                                     │  └─ vẽ joints + bones → debug    │
                                     └──────────────────────────────────┘

  Landmark ID map:
  ┌────────────────────────────────────────────────────────────────────┐
  │  Body (MediaPipe Pose):                                            │
  │   0=nose    11=left_shoulder   12=right_shoulder                   │
  │   13=left_elbow  14=right_elbow  15=left_wrist  16=right_wrist     │
  │   23=left_hip    24=right_hip                                      │
  │                                                                    │
  │  Left Hand (100-120):            Right Hand (200-220):             │
  │   100=wrist                       200=wrist                        │
  │   101-104=thumb                   201-204=thumb                    │
  │   105-108=index                   205-208=index                    │
  │   109-112=middle                  209-212=middle                   │
  │   113-116=ring                    213-216=ring                     │
  │   117-120=pinky                   217-220=pinky                    │
  └────────────────────────────────────────────────────────────────────┘
```

---

## 5. Chi tiết MODULE C — dual\_kinect\_fusion\_controller

```
  ┌──────────────────────────────────────────────────────────────────────┐
  │              Kiến trúc Timer-driven 20 Hz                            │
  └──────────────────────────────────────────────────────────────────────┘

  ROS callback thread A          ROS callback thread B
  ┌───────────────────┐          ┌───────────────────┐
  │   _front_cb()     │          │   _back_cb()      │
  │                   │          │                   │
  │  cache front_msg  │          │  cache back_msg   │
  │  + arrival_time   │          │  + arrival_time   │
  │  (mutex-guarded)  │          │  (mutex-guarded)  │
  └───────────────────┘          └───────────────────┘

  Timer thread (20 Hz)
  ┌─────────────────────────────────────────────────────────────────────┐
  │                      _timer_cb()  [exception-guarded]               │
  └──────────────────────────────────┬──────────────────────────────────┘
                                     │
          ┌──────────────────────────▼──────────────────────────────────┐
          │              đọc cache (mutex) + decode                      │
          │  front_dict = decode(front_msg)  hoặc {} nếu stale          │
          │  back_dict  = decode(back_msg)   hoặc {} nếu stale          │
          └──────────────────────────┬──────────────────────────────────┘
                                     │
          ┌──────────────────────────▼──────────────────────────────────┐
          │                  SkeletonFuser.fuse()                        │
          │                                                             │
          │  mỗi joint_id trong tracked_joint_ids:                      │
          │                                                             │
          │  ┌───────────────────────────────────────────────────────┐  │
          │  │ cả 2 fresh:                                           │  │
          │  │   dist ≤ max_merge_dist → average                    │  │
          │  │   dist > max_merge_dist → dùng FRONT (primary)       │  │
          │  │ chỉ front fresh → dùng front                         │  │
          │  │ chỉ back fresh  → dùng back  (occlusion fill)        │  │
          │  │ cả 2 stale      → NaN (joint bị bỏ)                  │  │
          │  └───────────────────────────────────────────────────────┘  │
          │                                                             │
          │  mode: BOTH / FILL_FROM_BACK / FRONT_ONLY /                 │
          │        BACK_ONLY / NO_INPUT                                 │
          └──────────────────────────┬──────────────────────────────────┘
                                     │
          ┌──────────────────────────▼──────────────────────────────────┐
          │                DutyScheduler.tick()                          │
          │                                                             │
          │  front_missing = |tracked_set| - front_valid_fresh_count    │
          │                                                             │
          │  ┌─────────────────── STEADY: back=LOW ──────────────────┐  │
          │  │  front_missing >= miss_threshold (2)                   │  │
          │  │  sustained confirm_frames (3) ticks?                   │  │
          │  │  → command back = HIGH (back_high_rate=6)              │  │
          │  │  → trạng thái = BOOST                                  │  │
          │  └───────────────────────────────────────────────────────┘  │
          │                                                             │
          │  ┌─────────────────── BOOST: back=HIGH ──────────────────┐  │
          │  │  front phục hồi (front_missing < miss_threshold)?      │  │
          │  │  → giữ HIGH thêm boost_hold_sec (1.5 s) — hysteresis  │  │
          │  │  → rồi command back = LOW (back_low_rate=2)            │  │
          │  │  → trạng thái = STEADY                                 │  │
          │  └───────────────────────────────────────────────────────┘  │
          │                                                             │
          │  Rate command: Float32, chỉ publish khi thay đổi           │
          └──────────────────────────┬──────────────────────────────────┘
                                     │
          ┌──────────────────────────▼──────────────────────────────────┐
          │                     Publish outputs                          │
          │                                                             │
          │  /human_skeleton_base           (PoseArray fused, 20 Hz)   │
          │  /human_skeleton_fusion_status  (String: mode + duty)       │
          │  /kinect_front/tracker_rate_cmd (Float32, khi thay đổi)    │
          │  /kinect_back/tracker_rate_cmd  (Float32, khi thay đổi)    │
          └─────────────────────────────────────────────────────────────┘
```

### Fusion Mode — trạng thái

```
              ┌──────────────────────────────────────┐
              │            NO_INPUT                  │
              │       cả 2 camera stale              │
              │   publish skeleton NaN rỗng          │
              └────┬──────────────┬──────────────────┘
                   │              │
          front OK │              │ back OK
          back stale│             │ front stale
                   │              │
                   ▼              ▼
    ┌──────────────────┐    ┌──────────────────┐
    │   FRONT_ONLY     │    │   BACK_ONLY      │
    │  chỉ dùng front  │    │  chỉ dùng back   │
    └────────┬─────────┘    └──────────┬───────┘
             │ back trở lại            │ front trở lại
             └─────────────┬───────────┘
                           │
                           ▼
               ┌─────────────────────────────────────┐
               │   BOTH / FILL_FROM_BACK              │
               │   cả 2 camera fresh                  │
               │   BOTH: all joints from both avail.  │
               │   FILL_FROM_BACK: front thiếu joints │
               │     back bù (occlusion fill)         │
               └─────────────────────────────────────┘
```

### DutyScheduler — trạng thái

```
  [STARTUP]
  front = HIGH (10 Hz)
  back  = LOW  (2 Hz)
       │
       ▼
  ┌──────────────────────────────────┐
  │  STEADY: back = LOW              │
  │  kiểm tra front_missing mỗi tick│
  └─────────┬────────────────────────┘
            │ front_missing >= 2
            │ liên tiếp 3 ticks
            ▼
  ┌──────────────────────────────────┐
  │  BOOST: back = HIGH              │
  │  kiểm tra front phục hồi        │
  └─────────┬────────────────────────┘
            │ front full, đã giữ boost_hold_sec
            ▼
  ┌──────────────────────────────────┐
  │  STEADY: back = LOW              │
  └──────────────────────────────────┘
```

---

## 6. Chi tiết MODULE D — skeleton\_obstacle\_builder

```
  ┌─────────────────────────────────────────────────────────────┐
  │             skeleton_callback(msg)                          │
  │         nhận /human_skeleton_base  (PoseArray)              │
  └──────────────────────────┬──────────────────────────────────┘
                             │
             ┌───────────────▼───────────────┐
             │      decode_pose_array()      │
             │  canonical schema → subset   │
             └───────────────┬───────────────┘
                             │
             ┌───────────────▼───────────────┐
             │      sanitize_skeleton()      │
             │  loại NaN / Inf               │
             │  đếm valid joints             │
             └───────┬───────────────┬───────┘
                 OK  │               │ thiếu joints
                     │               ▼
                     │    ┌──────────────────────┐
                     │    │  remove_object()     │
                     │    │  publish status      │
                     │    └──────────────────────┘
                     │
             ┌───────▼───────────────────────┐
             │      _max_joint_delta()       │
             │  delta < threshold ?          │
             │  ├─ YES → SKIP + heartbeat    │
             │  └─ NO  → rebuild             │
             └───────────────┬───────────────┘
                             │
             ┌───────────────▼───────────────┐
             │ _compute_speed_vectorized()   │
             │  NumPy max speed (m/s)        │
             └───────────────┬───────────────┘
                             │
             ┌───────────────▼───────────────────────────────┐
             │         Dynamic Padding Pipeline              │
             │                                               │
             │  speed_over_deadband                          │
             │    = max(0, speed - deadband)                 │
             │                                               │
             │  raw_padding                                  │
             │    = speed_gain × speed_over_deadband         │
             │                                               │
             │  clamped = min(max_dynamic_padding, raw)      │
             │                                               │
             │  smooth_padding  (EMA alpha=0.7)              │
             │    ← smooth_dynamic_padding(clamped)          │
             └───────────────┬───────────────────────────────┘
                             │
             ┌───────────────▼───────────────────────────────┐
             │     build_collision_object(skeleton, padding)  │
             └───────────────────────────────────────────────┘
                             │
           ┌─────────────────┼──────────────────────┐
           │                                        │
           ▼                                        ▼
 ┌──────────────────────────┐          ┌────────────────────────────────┐
 │  make_sphere()           │          │    make_cylinder_between()     │
 │  mỗi joint hợp lệ        │          │    mỗi cặp joint hợp lệ        │
 │  → đúng 1 sphere         │          │    → đúng 1 cylinder           │
 │                          │          │                                │
 │  final_radius =          │          │    final_radius =              │
 │   body + static + smooth │          │     body + static + smooth     │
 │                          │          │                                │
 │  Single/Dual launch:     │          │    Single/Dual launch:         │
 │  radius=0.05, padding=0  │          │    radius=0.05, padding=0      │
 └──────────────┬───────────┘          └──────────────┬─────────────────┘
                └──────────────────┬───────────────────┘
                                   │
                   ┌───────────────▼───────────────┐
                   │  append primitives trực tiếp  │
                   │  không cap primitive count    │
                   │  tổng hợp vào CollisionObject │
                   └───────────────┬───────────────┘
                                   │
                   ┌───────────────▼───────────────┐
                   │           Publish             │
                   │                               │
                   │  /human_collision_object      │
                   │    (CollisionObject — ADD)    │
                   │  /human_obstacle_status       │
                   │    OBSTACLE_PUBLISHED         │
                   └───────────────────────────────┘

  ─────────────────────── timer riêng ──────────────────────────
  ┌────────────────────────────────────────────────────────────┐
  │                  cleanup_timer(event)                      │
  │                                                            │
  │  Không nhận skeleton > timeout_sec ?                       │
  │  ├─ YES → remove_object()                                  │
  │  │        /human_collision_object (REMOVE)                 │
  │  │        /human_obstacle_status: OBSTACLE_TIMEOUT_REMOVE  │
  │  └─ NO  → nothing                                          │
  └────────────────────────────────────────────────────────────┘
```

---

## 7. Chi tiết MODULE E — moveit\_scene\_manager

```
  ┌─────────────────────────────────────────────────────────────┐
  │           collision_object_callback(msg)                    │
  │        nhận /human_collision_object  (CollisionObject)      │
  └──────────────────────────┬──────────────────────────────────┘
                             │
             ┌───────────────▼───────────────┐
             │    validate_collision_object()│
             │                               │
             │  có primitives?               │
             │  pose count = primitive count?│
             │  quaternion hợp lệ?           │
             └───────┬───────────────┬───────┘
                 OK  │               │ INVALID
                     │               ▼
                     │    ┌──────────────────────┐
                     │    │  log invalid, bỏ qua │
                     │    └──────────────────────┘
                     │
             ┌───────▼───────────────────────┐
             │    apply_collision_object()   │
             │                               │
             │  gọi /apply_planning_scene    │
             │  service (ApplyPlanningScene) │
             │                               │
             │  ADD  → thêm object           │
             │  REMOVE → xóa object          │
             └───────┬───────────────┬───────┘
              OK     │               │ FAIL
                     │               ▼
                     │    ┌──────────────────────────┐
                     │    │  log failure             │
                     │    │  SCENE_APPLY_FAILED       │
                     │    └──────────────────────────┘
                     │
             ┌───────▼─────────────────────────────────────────┐
             │             publish_status()                     │
             │                                                  │
             │  /moveit_scene_status  (String)                  │
             │                                                  │
             │  SCENE_OBJECT_APPLIED           ← sau ADD call   │
             │  SCENE_OBJECT_CONFIRMED         ← service OK     │
             │  SCENE_OBJECT_REMOVED           ← sau REMOVE     │
             │  SCENE_OBJECT_CONFIRMED_REMOVED ← remove OK      │
             └──────────────────┬───────────────────────────────┘
                                │
                                ▼
              ┌─────────────────────────────────┐
              │     MoveIt PlanningScene        │
              │  cập nhật real-time             │
              │  planner có thể dùng ngay       │
              └─────────────────────────────────┘

  ─────────────────────── timer riêng ──────────────────────────
  ┌────────────────────────────────────────────────────────────┐
  │                 cleanup_stale_objects()                    │
  │                                                            │
  │  object tồn tại > max_object_age_sec ?                     │
  │  → remove_object(object_id)                                │
  │  → SCENE_EMPTY                                             │
  └────────────────────────────────────────────────────────────┘
```

### Scene Status — trạng thái

```
  [khởi động]
       │
       ▼
  ┌─────────────┐   nhận CollisionObject ADD    ┌─────────────────────────┐
  │ SCENE_READY │ ────────────────────────────► │  SCENE_OBJECT_APPLIED   │
  └─────────────┘                               └──────────┬──────────────┘
       ▲                                                   │ service OK
       │                                                   ▼
  ┌────────────┐                               ┌──────────────────────────┐
  │ SCENE_EMPTY│ ◄─────────────────────────── │ SCENE_OBJECT_CONFIRMED   │
  └────────────┘    confirmed removed          └──────────┬──────────────┘
       ▲                                                   │ object mới
       │                                    nhận REMOVE    │ (update liên tục)
  ┌────────────────────────────┐                 │         │
  │ SCENE_OBJECT_CONFIRMED_    │ ◄───────────────┘         │
  │ REMOVED                    │                           ▼
  └────────────────────────────┘               ┌──────────────────────────┐
                                               │  SCENE_OBJECT_APPLIED   │
                                               │  (lại từ đầu)           │
                                               └──────────────────────────┘
```

---

## 8. Chi tiết MODULE F — planner\_ab\_replan\_node

```
  ┌─────────────────────────────────────────────────────────────┐
  │                    spin()  — A→B→A loop                     │
  └──────────────────────────┬──────────────────────────────────┘
                             │
             ┌───────────────▼───────────────┐
             │        run_path(path)         │
             │  duyệt 7 waypoints            │
             │  direction: A→B hoặc B→A     │
             └───────────────┬───────────────┘
                             │
   ┌─────────────────────────▼──────────────────────────────────────────┐
   │                   run_waypoint(joint_map, idx)                     │
   └─────────────────────────┬──────────────────────────────────────────┘
                             │
             ┌───────────────▼────────────────────────┐
             │    hand_blocks_target_pose(pose) ?     │
             │    tay người trong vùng waypoint đích? │
             └───────┬───────────────────────┬────────┘
                 NO  │                       │ YES
                     │                       ▼
                     │         ┌─────────────────────────────────┐
                     │         │  run_detour_if_hand_blocks_     │
                     │         │  waypoint()                     │
                     │         │                                 │
                     │         │  ARA* gate detour pose          │
                     │         │  MoveIt tối đa 5 lần, 0.75s/lần│
                     │         │  fail → HOLD, poll tay 0.5s    │
                     │         │  tiếp tục waypoint gốc          │
                     │         └───────────────┬─────────────────┘
                     └──────────────────────────┘
                                    │
             ┌──────────────────────▼────────────────────────┐
             │           can_plan_to_waypoint() ?            │
             │                                               │
             │  moveit_scene_ready() ?                       │
             │  scene_status_is_fresh() ?                    │
             │  scene_ready_for_planning(human_active) ?     │
             └───────┬────────────────────────────┬──────────┘
                 OK  │                            │ NOT READY
                     │                            ▼
                     │              ┌──────────────────────────┐
                     │              │   chờ / retry loop       │
                     │              │   quay lại kiểm tra      │
                     │              └──────────────────────────┘
                     │
             ┌───────▼───────────────────────────────────────┐
             │       astar_path_is_available(joint_map) ?    │
             │                                               │
             │  active_human_voxels()                        │
             │    ← inflate skeleton joints → voxel set      │
             │                                               │
             │  AStarImproved3D.plan_with_info()             │
             │    hoặc .replan_with_info()                   │
             │    (ARA*: ε 3.0 → 1.0, budget 50ms)          │
             └───────┬───────────────────────────┬───────────┘
               OK    │                           │ BLOCKED
                     │                           ▼
                     │             ┌──────────────────────────────┐
                     │             │  NO_PATH / GOAL_BLOCKED      │
                     │             │  log, không plan             │
                     │             │  retry → can_plan_to_        │
                     │             │  waypoint()                  │
                     │             └──────────────────────────────┘
                     │
             ┌───────▼───────────────────────────────────────┐
             │      plan_to_joint_map(joint_map)             │
             │                                               │
             │  MoveIt group.plan()  (OMPL)                  │
             │  → RobotTrajectory                            │
             └───────┬───────────────────────────┬───────────┘
                 OK  │                           │ FAIL
                     │                           ▼
                     │             ┌──────────────────────────────┐
                     │             │  không tìm được plan         │
                     │             │  retry / max_plan_attempts   │
                     │             └──────────────────────────────┘
                     │
             ┌───────▼───────────────────────────────────────┐
             │         trajectory_is_safe(plan) ?            │
             │                                               │
             │  FK mỗi trajectory sample                     │
             │  tất cả robot links → poses                   │
             │  khoảng cách tới skeleton points              │
             │  ≥ human_safety_distance (0.12m) ?           │
             └───────┬───────────────────────────┬───────────┘
                SAFE │                           │ UNSAFE
                     │                           ▼
                     │             ┌──────────────────────────────┐
                     │             │  không execute               │
                     │             │  retry → can_plan_to_waypoint│
                     │             └──────────────────────────────┘
                     │
             ┌───────▼───────────────────────────────────────┐
             │           retime_plan(plan)                   │
             │  scale velocity + acceleration                │
             └───────────────┬───────────────────────────────┘
                             │
             ┌───────────────▼───────────────────────────────┐
             │             execute_plan(plan)                │
             │                                               │
             │  group.execute()                              │
             │                                               │
             │  ┌─── Monitor loop trong khi chạy ──────────┐ │
             │  │                                          │ │
             │  │  current_robot_hand_min_distance()       │ │
             │  │  FK robot links + skeleton points        │ │
             │  │                                          │ │
             │  │  distance < required_clearance?          │ │
             │  │  ├─ YES → group.stop()                   │ │
             │  │  │        EMERGENCY STOP                 │ │
             │  │  │        return False → retry           │ │
             │  │  └─ NO  → tiếp tục execute              │ │
             │  └──────────────────────────────────────────┘ │
             └───────────────┬───────────────────────────────┘
                             │ done OK
                             ▼
             ┌───────────────────────────────────────────────┐
             │          waypoint hoàn thành                  │
             │          → waypoint tiếp theo                 │
             │          → hết path → đảo chiều               │
             │          → A→B→A loop tiếp                    │
             └───────────────────────────────────────────────┘
```

---

## 9. Chi tiết MODULE G — astar\_improved\_3d

```
  ┌──────────────────────────────────────────────────────────────────────┐
  │               AStarImproved3D — ARA* (Anytime Repairing A*)          │
  │               Không phải ROS node — thư viện Python                 │
  │               Import bởi planner_ab_replan_node.py                  │
  └──────────────────────────────────────────────────────────────────────┘

  Khởi tạo:
  ┌─────────────────────────────────────────────────────────────────────┐
  │  AStarImproved3D(                                                   │
  │    size_x, size_y, size_z,   ← kích thước lưới voxel               │
  │    diagonal = True/False,    ← 6 hoặc 26 hướng                     │
  │    epsilon_start = 3.0,      ← ε ban đầu (tìm nhanh)               │
  │    epsilon_final = 1.0,      ← ε cuối (tối ưu)                     │
  │    epsilon_decay = 0.5,      ← giảm ε mỗi iteration                │
  │    max_time_ms   = 50.0,     ← budget thời gian                    │
  │    max_steps     = 50000     ← bước expand tối đa                  │
  │  )                                                                  │
  └─────────────────────────────────────────────────────────────────────┘

  ─────────────────────── plan_with_info() ─────────────────────────────

             ┌───────────────────────────────────────────────┐
             │             Input                             │
             │  start    : Voxel (ix, iy, iz)               │
             │  goal     : Voxel (ix, iy, iz)               │
             │  obstacles: Set[Voxel]                        │
             └───────────────────┬───────────────────────────┘
                                 │
             ┌───────────────────▼───────────────────────────┐
             │         filter_obstacles(obstacles)           │
             │  loại voxel ngoài biên lưới                   │
             │  → valid_count, invalid_count                 │
             └───────────────────┬───────────────────────────┘
                                 │
             ┌───────────────────▼───────────────────────────┐
             │     Kiểm tra start / goal hợp lệ             │
             │                                               │
             │  goal trong obstacle?  → GOAL_BLOCKED         │
             │  start trong obstacle? → START_BLOCKED        │
             │  ngoài biên?           → OUT_OF_BOUNDS        │
             └───────────┬──────────────────────┬────────────┘
                      OK │                      │ BLOCKED
                         │                      ▼
                         │         ┌────────────────────────────────┐
                         │         │  PlanResult(success=False,     │
                         │         │    reason=GOAL_BLOCKED etc.)   │
                         │         └────────────────────────────────┘
                         │
             ┌───────────▼─────────────────────────────────────────┐
             │              ARA* iteration loop                    │
             │                                                     │
             │  ε = ε_start (3.0)                                  │
             │                                                     │
             │  ┌──────────────────────────────────────────────┐   │
             │  │  while ε ≥ ε_final AND time_budget còn:      │   │
             │  │                                              │   │
             │  │    _improve_path(ε)                          │   │
             │  │    Weighted A*: f(s) = g(s) + ε × h(s)      │   │
             │  │      expand OPEN set                         │   │
             │  │      cập nhật OPEN, CLOSED, INCONS           │   │
             │  │      h(s) = Euclidean đến goal               │   │
             │  │                                              │   │
             │  │    tìm được path → rebuild path              │   │
             │  │    ε -= ε_decay (0.5)                        │   │
             │  │    tiếp tục cải thiện với ε nhỏ hơn          │   │
             │  │                                              │   │
             │  │  TIMEOUT (50ms) → dừng, trả kết quả tốt nhất│   │
             │  │  MAX_STEPS → dừng                            │   │
             │  └──────────────────────────────────────────────┘   │
             └───────────────────┬─────────────────────────────────┘
                                 │
             ┌───────────────────▼───────────────────────────┐
             │              PlanResult                       │
             │                                               │
             │  path    : List[Voxel]                        │
             │  success : bool                               │
             │  reason  : OK / NO_PATH / TIMEOUT / ...       │
             │  metrics : {                                   │
             │    expanded_steps,                            │
             │    epsilon_satisfied,                         │
             │    elapsed_ms,                                │
             │    changed_obstacle_count                     │
             │  }                                            │
             └───────────────────────────────────────────────┘

  ─────────────────────── replan_with_info() ───────────────────────────

             ┌───────────────────────────────────────────────┐
             │  Dùng lại INCONS từ lần plan trước            │
             │  Chỉ update voxel thay đổi                    │
             │  → nhanh hơn plan lần đầu                     │
             │  Chạy _ara_search() từ new_start              │
             └───────────────────────────────────────────────┘

  ─────────────────────── ε schedule ───────────────────────────────────

  ε = 3.0 → suboptimal nhưng tìm nhanh (ưu tiên có đường)
  ε = 2.5 → cải thiện
  ε = 2.0 → cải thiện
  ε = 1.5 → cải thiện
  ε = 1.0 → A* chính xác (tối ưu) nếu còn time budget

  ─────────────────────── Reason codes ────────────────────────────────

  ┌────────────────────────────┬──────────────────────────────────────┐
  │  OK                        │  Tìm được đường                      │
  │  NO_PATH                   │  Không có đường (bị block hoàn toàn) │
  │  GOAL_BLOCKED              │  Goal nằm trong obstacle             │
  │  START_BLOCKED             │  Start nằm trong obstacle            │
  │  GOAL_OUT_OF_BOUNDS        │  Goal ngoài biên voxel map           │
  │  START_OUT_OF_BOUNDS       │  Start ngoài biên voxel map          │
  │  MAX_STEPS_REACHED         │  Vượt max_steps (50000)              │
  │  TIMEOUT                   │  Hết 50ms budget                     │
  │  PATH_EXTRACTION_FAILED    │  Không reconstruct được path         │
  │  REPLAN_NOT_INITIALIZED    │  Gọi replan trước plan lần đầu       │
  │  EMPTY_GRID                │  Lưới voxel không hợp lệ             │
  └────────────────────────────┴──────────────────────────────────────┘
```

---

## 10. Luồng Safety — 5 lớp bảo vệ

```
  ┌────────────────────────────────────────────────────────────────────┐
  │                NGƯỜI xuất hiện gần robot                          │
  └────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
  ╔════════════════════════════════════════════════════════════════════╗
  ║  LỚP 1 — Perception  (MODULE A)                                   ║
  ║                                                                   ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  SpatialArmFilter                                        │     ║
  ║  │  per-joint jump > arm_max_jump_m → giữ điểm trước        │     ║
  ║  │  thường. Prevent phantom obstacle từ noise.              │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  validate_body_geometry()                                │     ║
  ║  │  skeleton hình học sai → bỏ qua, không publish           │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ╚════════════════════════════════════════════════════════════════════╝
                                │
                                ▼
  ╔════════════════════════════════════════════════════════════════════╗
  ║  LỚP 2 — Fusion  (MODULE C — Dual Kinect only)                   ║
  ║                                                                   ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  SkeletonFuser.fuse()                                    │     ║
  ║  │  max_merge_dist guard: tránh average joint bị đọc sai    │     ║
  ║  │  both-stale → NaN → downstream removal timeout clear obj │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ╚════════════════════════════════════════════════════════════════════╝
                                │
                                ▼
  ╔════════════════════════════════════════════════════════════════════╗
  ║  LỚP 3 — Collision Object  (MODULE D)                            ║
  ║                                                                   ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  Sphere joints + cylinder bones, radius cố định 0.05m    │     ║
  ║  │  static/dynamic padding tắt trong Single/Dual launch     │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  cleanup_timer()                                         │     ║
  ║  │  mất skeleton → REMOVE obstacle tránh ghost object       │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ╚════════════════════════════════════════════════════════════════════╝
                                │
                                ▼
  ╔════════════════════════════════════════════════════════════════════╗
  ║  LỚP 4 — Planning Guard  (MODULE F + G)                          ║
  ║                                                                   ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  ARA* guard                                             │     ║
  ║  │  NO_PATH / GOAL_BLOCKED → không plan → chờ người rời    │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  trajectory_is_safe()                                    │     ║
  ║  │  FK mỗi sample × clearance → reject plan không an toàn  │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  hand_blocks_target_pose()                               │     ║
  ║  │  tay người chặn waypoint → detour hoặc chờ              │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ╚════════════════════════════════════════════════════════════════════╝
                                │
                                ▼
  ╔════════════════════════════════════════════════════════════════════╗
  ║  LỚP 5 — Emergency Stop  (MODULE F — trong execute_plan)         ║
  ║                                                                   ║
  ║  ┌──────────────────────────────────────────────────────────┐     ║
  ║  │  Monitor loop liên tục trong khi robot đang chạy        │     ║
  ║  │                                                          │     ║
  ║  │  current_robot_hand_min_distance()                       │     ║
  ║  │    FK robot links → tất cả poses                         │     ║
  ║  │    distance tới skeleton points                          │     ║
  ║  │                                                          │     ║
  ║  │  distance < required_clearance                           │     ║
  ║  │    → group.stop()   ← DỪNG ROBOT NGAY LẬP TỨC          │     ║
  ║  │    → return False → replan                              │     ║
  ║  └──────────────────────────────────────────────────────────┘     ║
  ╚════════════════════════════════════════════════════════════════════╝
                                │
                                ▼
  ┌────────────────────────────────────────────────────────────────────┐
  │              Robot dừng hoặc tránh — người an toàn                │
  └────────────────────────────────────────────────────────────────────┘
```

---

## 11. ROS Topic Map

### Single Kinect

```
  ┌──────────────────────────────────────────────────────────────────┐
  │                    MODULE A                                      │
  │              kinect_skeleton_tracker                             │
  └──────┬───────────────────┬──────────────────────────────────────┘
         │                   │
         │ /human_skeleton_base (PoseArray)
         │                   │
         ▼                   ▼
  ┌──────────────┐    ┌─────────────────────────────────┐
  │   MODULE D   │    │           MODULE F              │
  │  obstacle_   │    │     planner_ab_replan_node      │
  │  builder     │    │                                 │
  └──────┬───────┘    │  ◄── /human_skeleton_base       │
         │            │  ◄── /moveit_scene_status        │
         │            │  ◄── /human_obstacle_status      │
         │            │                                 │
  /human_collision_object    /ur3_fixed_joint_path_status│
  (CollisionObject)   │      /hrc_path_text              │
         │            │      /hrc_astar_voxel_markers    │
         ▼            │      /hrc_*_time_ms              │
  ┌──────────────┐    └──────────────────────┬──────────┘
  │   MODULE E   │                           │
  │  moveit_     │  /apply_planning_scene    │ group.plan()
  │  scene_mgr   │──service──► MoveIt ◄──────┘ group.execute()
  └──────┬───────┘    PlanningScene
         │
  /moveit_scene_status
         │
         └──────────────────► MODULE F (subscribe)
```

### Dual Kinect — thêm MODULE C

```
  ┌────────────────────┐      ┌────────────────────────┐
  │    MODULE A        │      │    MODULE A             │
  │  tracker_front     │      │  tracker_back           │
  └─────────┬──────────┘      └──────────┬──────────────┘
            │                            │
  /kinect_front/human_skeleton_base      │
            │             /kinect_back/human_skeleton_base
            └─────────────┬──────────────┘
                          │ front_callback() / back_callback()
                          ▼
  ┌───────────────────────────────────────────────────────────────┐
  │                      MODULE C                                 │
  │            dual_kinect_fusion_controller                      │
  └──────┬────────────────────────────┬────────────────────────────┘
         │                            │
  /human_skeleton_base         /human_skeleton_fusion_status
  (PoseArray fused)            /kinect_front/tracker_rate_cmd
         │                     /kinect_back/tracker_rate_cmd
         │
         └──────► MODULE D, MODULE F  (giống Single Kinect)
```

---

*Cập nhật: 2026-05-29 — ASCII block diagram, không dùng Mermaid*
