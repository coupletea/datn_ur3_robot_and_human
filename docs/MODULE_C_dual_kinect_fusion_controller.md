# MODULE C — `dual_kinect_fusion_controller.py`

> **Node name:** `dual_kinect_fusion_controller`
> **Vai trò:** Fuse skeleton từ 2 Kinect (front + back) thành 1 skeleton đầu ra; điều phối duty-cycle tốc độ từng tracker theo tình trạng occlusion.
> **Kiến trúc:** Timer-driven tại `fusion_rate_hz` (mặc định 20 Hz). Ba class: `SkeletonFuser` (pure logic), `DutyScheduler` (pure logic), `DualKinectFusionController` (ROS node).
> **Thay thế:** `multi_kinect_skeleton_controller.py` (đã xóa). Xem mục [Migration Note](#migration-note).

---

## Tổng quan pipeline

```
/kinect_front/human_skeleton_base  ─┐
                                    ├─► DualKinectFusionController (timer 20 Hz)
/kinect_back/human_skeleton_base   ─┘       │
                                            ├─ SkeletonFuser.fuse()
                                            │     → per-joint occlusion-fill
                                            ├─ DutyScheduler.tick()
                                            │     → back rate cmd (LOW/HIGH)
                                            └─ publish outputs
                                                  /human_skeleton_base           (PoseArray)
                                                  /human_skeleton_fusion_status  (String)
                                                  /kinect_front/tracker_rate_cmd (Float32)
                                                  /kinect_back/tracker_rate_cmd  (Float32)
```

Output giữ nguyên ROS interfaces với downstream:

| Topic | Type | Chiều | Mô tả |
|-------|------|-------|-------|
| `/kinect_front/human_skeleton_base` | `geometry_msgs/PoseArray` | Subscribe | Skeleton front camera (đã trong `base_link`) |
| `/kinect_back/human_skeleton_base` | `geometry_msgs/PoseArray` | Subscribe | Skeleton back camera (đã trong `base_link`) |
| `/human_skeleton_base` | `geometry_msgs/PoseArray` | Publish | Skeleton đã fuse (base_link) |
| `/human_skeleton_fusion_status` | `std_msgs/String` | Publish | Trạng thái fusion + duty-cycle chi tiết |
| `/kinect_front/tracker_rate_cmd` | `std_msgs/Float32` | Publish | Lệnh tốc độ cho tracker front |
| `/kinect_back/tracker_rate_cmd` | `std_msgs/Float32` | Publish | Lệnh tốc độ cho tracker back |

---

## Body joints và PoseArray convention

Module C fuse 9 body joints (mặc định `tracked_joint_ids`):

```
[0, 11, 12, 13, 14, 15, 16, 23, 24]
```

| ID | Tên | Loại |
|----|-----|------|
| 0 | nose | body_core |
| 11 | left_shoulder | body_core |
| 12 | right_shoulder | body_core |
| 13 | left_elbow | arm |
| 14 | right_elbow | arm |
| 15 | left_wrist | arm |
| 16 | right_wrist | arm |
| 23 | left_hip | body_core |
| 24 | right_hip | body_core |

Output PoseArray schema: 1 pose per `tracked_joint_ids` (theo thứ tự), NaN cho joint thiếu. **Byte-compatible với output của node cũ** → downstream obstacle builder / planner không cần thay đổi.

---

## Class: `SkeletonFuser` (pure — không có ROS)

Logic fusion per-joint, không Kalman, không conflict map, không side-swap.

### Bảng fusion logic

| Front joint | Back joint | Kết quả |
|-------------|------------|---------|
| fresh + valid | fresh + valid | `dist <= max_merge_dist` → **trung bình** (average) |
| fresh + valid | fresh + valid | `dist > max_merge_dist` → **dùng FRONT** (tránh average đọc sai) |
| fresh + valid | stale / thiếu | **dùng FRONT** |
| stale / thiếu | fresh + valid | **dùng BACK** ← trường hợp occlusion-fill |
| stale | stale | joint bị bỏ → **NaN** trong output |

**Freshness:** camera stale khi `now - last_msg_time > max_input_age_sec` (0.35 s). Camera stale → không đóng góp joint nào.

**Modes báo cáo trong fusion_status:**
- `BOTH` — cả 2 camera fresh, fuse bình thường
- `FILL_FROM_BACK` — front fresh nhưng thiếu ≥ 1 joint → back bù
- `FRONT_ONLY` — chỉ front fresh
- `BACK_ONLY` — chỉ back fresh
- `NO_INPUT` — cả 2 stale → publish skeleton rỗng (all-NaN)

**Cơ chế NO_INPUT:**
- Khi cả 2 camera stale, node publish PoseArray all-NaN (nếu `empty_publish_on_no_input=true`)
- Downstream obstacle builder nhận skeleton rỗng → removal timeout tự xóa collision object
- Không có ghost obstacle kẹt lại trong MoveIt scene

---

## Class: `DutyScheduler` (pure — không có ROS)

State machine điều phối tốc độ back camera dựa trên occlusion từ front.

### Nguyên lý

- Front camera luôn giữ HIGH (là primary)
- Back camera mặc định LOW (backup, không bao giờ tắt hoàn toàn)
- Khi front thiếu joints → boost back lên HIGH để fill occlusion
- Khi front phục hồi → giữ back HIGH thêm `boost_hold_sec` (hysteresis), rồi hạ về LOW

### State machine

```
[STARTUP]
  → command front = HIGH (1 lần)
  → command back = LOW (1 lần)

[STEADY: back=LOW]
  per tick: đếm front_missing = |tracked_set| - front_valid_fresh_count
  if front_missing >= miss_threshold (2)
     sustained confirm_frames (3) ticks liên tiếp:
         → command back = HIGH
         → trạng thái = BOOST

[BOOST: back=HIGH]
  per tick: kiểm tra front phục hồi (front_missing < miss_threshold)
  if front phục hồi:
     → bắt đầu giữ HIGH thêm boost_hold_sec (1.5 s)
  if đã giữ đủ boost_hold_sec:
     → command back = LOW
     → trạng thái = STEADY

Lệnh rate: Float32 — chỉ publish khi giá trị thay đổi (idempotent)
```

---

## Class: `DualKinectFusionController` (ROS node)

Node chính. Timer 20 Hz đọc message cache từ 2 callback, gọi Fuser + DutyScheduler, publish kết quả.

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load params, khởi tạo subscribers/publishers, Fuser, DutyScheduler, timer |
| `_front_cb(msg)` | Cache front PoseArray + thời gian nhận, thread-safe (mutex) |
| `_back_cb(msg)` | Cache back PoseArray + thời gian nhận, thread-safe (mutex) |
| `_timer_cb(event)` | Callback timer 20 Hz: đọc cache → decode → fuse → duty tick → publish; exception không kill timer |
| `spin()` | `rospy.spin()` |

**Thread safety:** message cache được bảo vệ bằng mutex tránh torn read khi callback và timer chạy đồng thời.

**Exception guard:** `_timer_cb` bọc logic trong try/except để 1 exception không silently kill timer loop.

---

## Params quan trọng

| Param | Mặc định | Mô tả |
|-------|---------|-------|
| `~target_frame` | `base_link` | Frame đầu ra |
| `~tracked_joint_ids` | `0,11,12,13,14,15,16,23,24` | 9 body joints theo dõi |
| `~front_skeleton_base_topic` | `/kinect_front/human_skeleton_base` | Topic input front |
| `~back_skeleton_base_topic` | `/kinect_back/human_skeleton_base` | Topic input back |
| `~output_skeleton_base_topic` | `/human_skeleton_base` | Topic output fused |
| `~fusion_status_topic` | `/human_skeleton_fusion_status` | Topic status |
| `~front_rate_cmd_topic` | `/kinect_front/tracker_rate_cmd` | Lệnh rate cho front tracker |
| `~back_rate_cmd_topic` | `/kinect_back/tracker_rate_cmd` | Lệnh rate cho back tracker |
| `~fusion_rate_hz` | `20` | Tần số timer fusion (Hz); phải > 0 |
| `~max_input_age_sec` | `0.35` | Tuổi tối đa message đầu vào để coi là fresh |
| `~max_merge_dist` | `0.20` | Khoảng cách tối đa (m) để average 2 joint; phải >= 0 |
| `~miss_threshold` | `2` | Số joint front thiếu để trigger boost back |
| `~confirm_frames` | `3` | Số tick liên tiếp thiếu trước khi boost |
| `~boost_hold_sec` | `1.5` | Giữ back HIGH sau khi front phục hồi (hysteresis) |
| `~front_high_rate` | `10` | Rate front ở chế độ HIGH (Hz) |
| `~back_high_rate` | `6` | Rate back ở chế độ HIGH (Hz) |
| `~back_low_rate` | `2` | Rate back ở chế độ LOW (Hz) |
| `~empty_publish_on_no_input` | `true` | Publish skeleton rỗng (NaN) khi cả 2 camera stale |

**Validation:** `fusion_rate_hz > 0` và `max_merge_dist >= 0` được kiểm tra khi khởi động.

---

## So sánh với node cũ (`multi_kinect_skeleton_controller`)

### Đã xóa (deliberately — đơn giản hóa + giảm CPU)

| Feature cũ | Lý do bỏ |
|-----------|---------|
| Kalman constant-velocity filter | Tăng CPU, phức tạp cold-start; downstream filter đủ |
| Hard/soft conflict map | Phức tạp; agree-or-prefer-front đơn giản hơn và đủ dùng |
| Side-swap detection + hysteresis | Camera front/back không đảo L/R trong cấu hình thực tế |
| Per-camera quality scoring | Không cần khi logic fusion chỉ là simple priority |
| Agreement gate (3 ngưỡng) | Thay bằng `max_merge_dist` đơn giản |
| Debug MarkerArray | Tiết kiệm CPU; fusion_status String đủ để debug |

### Đã thêm

| Feature mới | Mô tả |
|------------|-------|
| DutyScheduler | Điều phối tốc độ back camera theo occlusion — tiết kiệm CPU |
| Timer-driven architecture | Tách output cadence khỏi input rate bất đối xứng |
| Thread-safe cache | Mutex guard tránh torn read giữa callback và timer |
| Exception guard trong timer | 1 exception không kill timer loop |
| `fusion_rate_hz` + `max_merge_dist` validation | Validate params khi khởi động |

---

## Safety notes

- **Không bao giờ tạo joint giả:** chỉ forward observation thực từ camera; không extrapolate hay hold.
- **Both-stale → empty → removal timeout:** downstream obstacle builder sẽ tự xóa CollisionObject sau `timeout_remove_sec`.
- **Rate command clamped:** tracker nhận lệnh rate bị clamp vào `[rate_min, rate_max]` (trong `kinect_skeleton_tracker.py`) — không stall, không runaway.
- **Không thay đổi topic, node name, frame ID, message type, PoseArray order** — byte-compatible với downstream.
- **max_merge_dist guard:** ngăn average vị trí người khi 1 camera đọc sai (divergent reading).
- Launch file vẫn là protected; thay đổi `dual_kinect_system.launch` cần xác nhận riêng.

---

## Verification

Static:

```bash
python3 -m py_compile scripts/dual_kinect_fusion_controller.py
```

Runtime smoke:

```bash
rostopic echo /human_skeleton_fusion_status
rostopic hz /human_skeleton_base
rostopic echo /kinect_back/tracker_rate_cmd
```

Scenario checks:

- Che front camera → mode `FILL_FROM_BACK` hoặc `BACK_ONLY`, back nhận rate HIGH.
- Che cả 2 camera → mode `NO_INPUT`, publish skeleton rỗng, downstream obstacle bị remove.
- Phục hồi front → back giữ HIGH thêm `boost_hold_sec`, rồi về LOW.
- Single-cam launch (`system.launch`, `system_back.launch`) → node này không chạy, hành vi giống trước.
- Chạy `test/test_pose_array_schema.py` để verify schema decode/encode contract giữ nguyên.

---

## Migration Note

Node này thay thế `multi_kinect_skeleton_controller.py` (file `.py` đã bị xóa khỏi `scripts/`).

Khác biệt chính:
- Kiến trúc **timer-driven** thay vì event-driven callback.
- Fusion đơn giản hơn: không Kalman, không conflict map, không side-swap, không quality scoring.
- Thêm **duty-cycle** điều phối rate tracker back.
- Mode labels thay đổi: cũ dùng `FUSION_OK/FALLBACK_FRONT/FALLBACK_BACK/NO_INPUT`; mới dùng `BOTH/FILL_FROM_BACK/FRONT_ONLY/BACK_ONLY/NO_INPUT`.
- Topic `/human_skeleton_fusion_mode` (latched) đã bỏ; thay bằng `/human_skeleton_fusion_status` (String chi tiết).
- Topic `/human_skeleton_fusion_markers` (MarkerArray) đã bỏ.
- Output `/human_skeleton_base` vẫn **byte-compatible**.

*Xem thêm: [MODULE_A_kinect_skeleton_tracker.md](MODULE_A_kinect_skeleton_tracker.md) — nguồn dữ liệu đầu vào và runtime rate control.*
*Xem thêm: [MODULE_D_skeleton_obstacle_builder.md](MODULE_D_skeleton_obstacle_builder.md) — module tiêu thụ output.*
