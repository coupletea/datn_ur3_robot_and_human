# MODULE F — `planner_ab_replan_node.py`

> **Node name:** `planner_ab_replan_node` (alias: `ur3_fixed_joint_ab_node`)
> **Vai trò:** Lập kế hoạch + thực thi robot UR3 chạy chu kỳ A→B→A liên tục. Tích hợp ARA* guard, kiểm tra an toàn trajectory, emergency stop khi người quá gần.

---

## Tổng quan flow

```
spin() loop: A → B → A → B → ...
    │
    ▼
run_path(path_a_to_b)
    └─► run_waypoint(joint_map, idx)
              │
              ├─ run_detour_if_hand_blocks_waypoint()
              │    └─ hand_blocks_target_pose()?
              │         ├─ ARA* gate → bounded MoveIt detour retries
              │         └─ retries exhausted → HOLD/poll until clear
              │
              └─ REPLAN LOOP:
                    │
                    ├─ can_plan_to_waypoint()?
                    │    ├─ skeleton active?
                    │    └─ scene_ready_for_planning()?
                    │
                    ├─ astar_path_is_available(joint_map)    ← ARA* guard
                    │    ├─ active_human_voxels()
                    │    └─ AStarImproved3D.plan/replan
                    │
                    ├─ MoveIt group.plan()                   ← OMPL
                    │
                    ├─ trajectory_is_safe(plan)?             ← FK + distance check
                    │
                    └─ execute_plan(plan)
                         └─ monitor: distance < clearance? → group.stop()
```

---

## Class: `UR3FixedJointABNode`

### Nhóm: Khởi tạo & Setup

| Phương thức | Chức năng |
|-------------|-----------|
| `__init__()` | Load params, kết nối MoveIt (`create_move_group()`), khởi tạo `AStarImproved3D`, build waypoints A→B, tạo subscribers/publishers |
| `create_move_group() → MoveGroupCommander` | Kết nối MoveIt với retry timeout (thử lại mỗi 1s đến `moveit_connect_timeout_sec`) |
| `get_safety_link_names() → List[str]` | Lấy danh sách tên link robot cần kiểm tra khoảng cách an toàn |
| `build_path_a_to_b() → List[JointMap]` | Định nghĩa 7 waypoints khớp cứng cho đường A→B |
| `resample_joint_path(path, target_count) → List[JointMap]` | Nội suy lại path thành N waypoints đều nhau (linear interpolation trong joint space) |

### Nhóm: Nhận dữ liệu người

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `human_callback(msg)` | `PointStamped` | Nhận điểm người (legacy single-point mode) |
| `human_skeleton_callback(msg)` | `PoseArray` | Nhận skeleton đầy đủ, gọi `select_planner_human_points()` |
| `moveit_scene_status_callback(msg)` | `String` | Nhận trạng thái MoveIt scene, lưu `latest_scene_status` |
| `decode_skeleton_pose_array(msg) → Dict[int, Point]` | `PoseArray` | Decode canonical 51/9-pose schema → `{joint_id: Point}` |
| `select_planner_human_points(skeleton_by_id) → List[Point]` | dict joints | Chọn subset joints cần thiết cho ARA* và safety check (theo `tracked_joint_ids`) |

### Nhóm: ARA* Guard

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `astar_path_is_available(joint_map) → bool` | dict joints | Chạy ARA* từ vị trí TCP hiện tại đến vị trí TCP của `joint_map`; `True` nếu tìm thấy đường không bị chặn |
| `astar_path_is_available_to_pose(target_pose) → bool` | `PoseStamped` | Chạy cùng ARA* guard cho pose Cartesian, dùng trước detour MoveIt |
| `active_human_voxels() → Set[Voxel]` | — | Lấy set voxel bị người chiếm = voxel quanh khớp (điểm đã ổn định qua cache) **∪ voxel dọc bones** |
| `_bone_obstacle_voxels() → Set[Voxel]` | — | Inflate voxel dọc đoạn nối khớp (`bone_connection_pairs`): sample nửa-voxel → inflate `bone_inflate_radius` → A* thấy chi/thân, không chỉ khớp |
| `stable_human_points() → List[Point]` | — | Làm mượt điểm người qua **obstacle stability cache**: điểm lệch trong `obstacle_stability_threshold` thì giữ anchor cũ (cùng voxel); anchor mất hình giữ thêm `obstacle_cache_hold_sec` rồi mới drop; loại điểm invalid (NaN/inf/ngoài map) |
| `_is_valid_human_point(point) → bool` | `Point` | Kiểm tra điểm hợp lệ: không NaN/inf và nằm trong map (kèm margin = `voxel_size`) |
| `_point_distance(a, b) → float` | `Point×2` | Khoảng cách Euclid giữa 2 điểm (m) |
| `inflate_voxel(center, radius) → Set[Voxel]` | Voxel, int | Giãn nở 1 voxel: **cầu** (`obstacle_inflate_sphere=true`, giữ `dx²+dy²+dz²≤r²`) hoặc cube đặc |
| `world_to_voxel(x, y, z) → Voxel` | float×3 | Chuyển tọa độ thế giới (m) → voxel index `(ix, iy, iz)` |
| `voxel_to_world(voxel) → (x, y, z)` | Voxel | Chuyển ngược voxel → tọa độ tâm voxel (m) |
| `current_tcp_voxel() → Voxel` | — | Voxel hiện tại của TCP robot (từ FK) |
| `publish_astar_markers(start, goal, path, obstacles)` | voxels (start/goal có thể `None`) | Visualize obstacles + path/start/goal trong RViz qua MarkerArray |
| `store_plan_viz(start, goal, path)` | voxels | Lưu path/start/goal của plan gần nhất để renderer dùng (thread-safe) |
| `render_astar_markers()` | — | Build obstacle **mới mỗi lần gọi** + publish; overlay path/start/goal chỉ giữ `obstacle_path_viz_hold` giây sau plan |
| `_obstacle_timer_cb(event)` | Timer | Callback timer định kỳ gọi `render_astar_markers()` (luôn build obs, không drop) |

### Nhóm: Forward Kinematics (FK)

| Phương thức | Chức năng |
|-------------|-----------|
| `target_tcp_pose_for_joint_map(joint_map) → Optional[PoseStamped]` | FK cho waypoint đích: tính pose TCP tại trạng thái khớp `joint_map` |
| `fk_pose_for_joint_positions(names, positions) → Optional[PoseStamped]` | FK cho tập (tên khớp, góc khớp) bất kỳ, trả pose của end-effector |
| `fk_poses_for_joint_positions(names, positions, link_names) → List[PoseStamped]` | FK nhiều link cùng lúc cho 1 trạng thái khớp |
| `fk_poses_for_robot_state(robot_state, link_names) → List[PoseStamped]` | FK nhiều link cho `RobotState` object |

### Nhóm: Điểm người

| Phương thức | Chức năng |
|-------------|-----------|
| `filter_human_points_near_link_poses(points, link_poses, context, threshold) → List[Point]` | Helper false-positive filter giữ lại trong file, không còn được gọi bởi pipeline chính |
| `filter_human_points_near_plan_path(points, plan, sample_indexes) → List[Point]` | Helper false-positive filter giữ lại trong file, không còn được gọi bởi pipeline chính |
| `latest_human_points() → List[Point]` | Trả về điểm người raw từ skeleton hoặc legacy point, không lọc gần robot/current path |

### Nhóm: Kiểm tra an toàn trajectory

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `trajectory_is_safe(plan) → bool` | `MoveItPlan` | Duyệt từng sample trajectory, FK cho tất cả safety links, tính khoảng cách min đến điểm người. `False` nếu bất kỳ sample nào < `human_safety_distance` |
| `current_robot_hand_min_distance() → Optional[(float, str)]` | — | Khoảng cách tối thiểu hiện tại robot→người (dùng trong execute monitor) |
| `trajectory_speed_margin(prev_poses, curr_poses, dt) → float` | poses trước/sau, delta time | Tính thêm margin an toàn theo tốc độ robot (robot nhanh → margin lớn hơn) |
| `hand_blocks_target_pose(pose) → bool` | `PoseStamped` | Kiểm tra tay người có trong vùng bán kính `hand_block_radius` xung quanh waypoint đích không |
| `goal_state_in_collision(joint_map) → Optional[bool]` | dict | Gọi MoveIt `check_state_validity` cho goal joint state. `True`=goal in-collision (OMPL sẽ reject ~vài chục ms), `False`=hợp lệ, `None`=service không có (giữ hành vi cũ) |
| `waypoint_obstructed(joint_map, target_pose) → bool` | dict, pose | Cổng detour gộp: tay chặn pose **HOẶC** goal state in-collision. Khớp đúng cái OMPL reject (fix B+C) |
| `plan_and_execute_detour_with_retry(pose, label) → bool` | pose, str | ARA* gate + MoveIt detour tối đa `max_detour_attempts`, có backoff |
| `hold_until_waypoint_clear(joint_map, target_pose, index) → bool` | dict, pose, int | Sau khi hết detour attempts, đứng yên và poll `waypoint_obstructed`; chỉ release khi goal hợp lệ + tay rời |
| `run_detour_if_hand_blocks_waypoint(joint_map, index) → bool` | dict, int | Trigger detour khi `waypoint_obstructed`; chạy detour +Z; nếu hết attempts thì HOLD tới khi waypoint clear |

### Nhóm: Lập kế hoạch & Thực thi

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `plan_to_joint_map(joint_map) → Optional[Plan]` | dict | Plan đến 1 waypoint joint (gọi MoveIt `group.plan()`) |
| `plan_to_pose(pose_goal, planning_time=None) → Optional[Plan]` | `PoseStamped`, float | ARA* gate trước OMPL; dùng timeout tùy chọn rồi restore timeout planner chính |
| `execute_plan(plan) → bool` | `MoveItPlan` | Thực thi plan + monitor loop: nếu khoảng cách robot→người < `required_clearance` → `group.stop()` (emergency stop) |
| `retime_plan(plan) → Plan` | `MoveItPlan` | Retime trajectory theo `velocity_scaling` và `acceleration_scaling` |
| `run_waypoint(joint_map, index, total, direction) → bool` | | Chạy 1 waypoint: detour check → replan loop (ARA* + MoveIt + safety + execute). Plan fail + goal in-collision → escalate detour/hold (fix B) thay vì retry vô hạn |
| `run_path(path, direction) → bool` | list waypoints, `"A→B"/"B→A"` | Chạy toàn bộ path (gọi `run_waypoint` lần lượt) |
| `spin()` | — | Vòng lặp chính A→B→A |

### Nhóm: Điều kiện cho phép plan

| Phương thức | Trả về | Chức năng |
|-------------|--------|-----------|
| `can_plan_to_waypoint() → bool` | bool | Kiểm tra scene sẵn sàng; không còn chặn khi skeleton thiếu/stale |
| `moveit_scene_ready() → bool` | bool | Scene MoveIt có sẵn sàng theo status manager chưa? |
| `scene_ready_for_planning(human_active) → bool` | bool | Kết hợp: sync_enabled? scene_status OK? |
| `scene_status_is_fresh() → bool` | bool | Status nhận được có còn trong `scene_status_timeout_sec` không? |

---

## Cấu hình — ROS Parameters

### Guard planner selection

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~guard_planner_type` | `ara_star` | Chọn guard planner TCP voxel: `ara_star` = `AStarImproved3D` (mặc định, giữ nguyên), `lpa_star` = `LPAStar3D` (incremental repair, drop-in API). Factory ở `__init__`. Xem `PROJECT_STRUCTURE.md` §11b + spec `docs/superpowers/specs/2026-06-20-lpa-star-3d-guard-planner-design.md` |
| `~lpa_epsilon` | `1.0` | (LPA*) hệ số heuristic; 1.0 = tối ưu |
| `~lpa_start_reuse_radius_voxels` | `1` | (LPA*) reserved; v1 chỉ REPAIR khi start không đổi |
| `~lpa_max_changed_obstacles_for_repair` | `500` | (LPA*) obstacle diff > ngưỡng → RESET thay vì REPAIR |

> LPA* dùng chung ngân sách `~ara_max_time_ms` / `~ara_max_steps`. Khác ARA*: hết giờ → `TIMEOUT` không có path (ARA* anytime trả best-so-far). Mọi call site (`plan_with_info`/`replan_with_info`/`set_penalty_cells`) giữ nguyên — API drop-in.
>
> **Runtime:** code default `ara_star`, nhưng cả 3 launch (`system.launch`, `system_back.launch`, `dual_kinect_system.launch`) hiện set `guard_planner_type=lpa_star` (kèm `lpa_epsilon=1.0`, `lpa_start_reuse_radius_voxels=1`, `lpa_max_changed_obstacles_for_repair=500`). Đổi value về `ara_star` để quay lại ARA*.

### ARA* Guard

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~ara_epsilon_start` | `3.0` | Hệ số heuristic ban đầu ARA* |
| `~ara_epsilon_final` | `1.0` | Hệ số heuristic tối thiểu (1.0 = exact A*) |
| `~ara_epsilon_decay` | `0.5` | Giảm ε mỗi iteration |
| `~ara_max_time_ms` | `50.0` | Ngân sách thời gian ARA* (ms) — cũng dùng cho LPA* |
| `~ara_max_steps` | `50000` | Số bước expand tối đa — cũng dùng cho LPA* |
| `~voxel_size` | `0.05` | Kích thước 1 voxel (m) |
| `~human_inflate_radius` | `0` | Số voxel inflate xung quanh khớp người |
| `~obstacle_inflate_sphere` | `True` | Inflate hình **cầu** (bỏ góc cube ngoài clearance) → giảm ~3.8× số voxel. `false` = cube đặc cũ |
| `~enable_bone_obstacles` | `True` | Bật build obstacle dọc đoạn nối khớp (bones), không chỉ quanh khớp |
| `~bone_inflate_radius` | `1` | Bán kính (voxel) ống obstacle dọc bone. Tăng → dày/an toàn hơn nhưng nhiều voxel hơn (A* chậm hơn) |
| `~bone_connection_pairs` | tay+thân | Cặp khớp định nghĩa bone, dạng `"11-13,13-15,..."` |
| `~enable_obstacle_cache` | `True` | Bật cache ổn định obstacle (chống drop voxel A* do skeleton jitter) |
| `~obstacle_stability_threshold` | `0.04` | Ngưỡng (m, ~3–5cm): điểm người lệch trong ngưỡng thì giữ nguyên anchor/voxel cũ |
| `~obstacle_cache_hold_sec` | `0.5` | Grace hold (s): giữ anchor khi mất hình tạm thời, quá hạn mới drop |
| `~obstacle_publish_rate` | `10.0` | Hz timer luôn build + publish obstacle (độc lập chu kỳ plan & khoảng cách robot). `<=0` = chỉ build khi plan (legacy) |
| `~obstacle_path_viz_hold` | `1.0` | Thời gian (s) overlay path/start/goal còn hiển thị sau plan; obstacle luôn vẽ mới |
| `~enable_plan_log` | `True` | Ghi log sự kiện plan (success/blocked/unsafe/not-executed) ra file |
| `~plan_log_dir` | `<pkg>/logs` | Thư mục chứa file log (`plan_log_<timestamp>.csv` + `.log`) |
| `~enable_breadcrumb` | `True` | Bật cache breadcrumb (nhớ pose đã đi để warm-start khi blocked) |
| `~breadcrumb_record_period` | `0.5` | Chu kỳ (s) ghi pose vào cache khi robot di chuyển |
| `~breadcrumb_revisit_threshold` | `0.05` | Ngưỡng (m) coi là về lại node cũ (trim prefix) + giãn cách node |
| `~breadcrumb_max_nodes` | `200` | Cap số node cache |
| `~enable_breadcrumb_fallback` | `True` | Khi waypoint blocked, hop tới node cache còn-valid để warm-start |
| `~enable_region_preference` | `True` | A* (soft) ưu tiên voxel trong vùng làm việc (bbox quét bởi waypoint A→B) |
| `~region_penalty_weight` | `4.0` | Cost cộng thêm mỗi bước A* ra ngoài vùng (soft, càng cao càng bám vùng) |
| `~region_margin` | `0.10` | Nới bbox vùng (m) quanh các waypoint |
| `~enable_pan_limit` | `True` | **HARD**: ép `shoulder_pan` trong dải sweep qua MoveIt path constraint (chống đứt dây khí nén) |
| `~pan_limit_margin` | `0.10` | Nới dải pan (rad) quanh min/max waypoint |
| `~shoulder_pan_min` / `~shoulder_pan_max` | `None` | Override dải pan (rad); None = tự suy từ waypoint |
| `~enable_astar_guard` | `True` | Bật/tắt ARA* guard |
| `~astar_speed_padding_gain` | `0.0` | m padding / (m/s) trên deadband. `0` = tắt (padding luôn 0, hành vi như cũ) |
| `~astar_speed_padding_deadband_mps` | `0.05` | Tốc độ TCP (m/s) dưới ngưỡng này → padding 0 |
| `~astar_max_speed_padding_m` | `0.05` | Cap padding (m) |
| `~astar_speed_padding_smoothing_alpha` | `0.7` | EMA làm mượt tốc độ TCP đo được |
| `~astar_recheck_period_sec` | `0.2` | Chu kỳ A* re-check trong lúc execute. `<=0` = tắt re-check (vẫn inflate theo speed, không trigger reroute) |

### An toàn

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~human_safety_distance` | `0.12` | Khoảng cách tối thiểu robot→người trong trajectory (m) |
| `~required_clearance` | `0.10` | Khoảng cách tối thiểu khi đang thực thi (emergency stop) (m) |
| `~hand_block_radius` | `0.15` | Bán kính tay chặn waypoint (m) |
| `~robot_path_false_positive_distance` | `0.15` | Tham số cũ cho helper false-positive filter; pipeline chính không còn dùng |
| `~ignore_human_points_near_robot_path` | `True` | Tham số cũ cho helper false-positive filter; pipeline chính không còn dùng |

### Detour bounded retry / HOLD

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~max_detour_attempts` | `5` | Số lần tối đa gọi detour planner trước HOLD |
| `~detour_retry_period` | `0.5` | Backoff giữa các detour attempts (s) |
| `~detour_planning_time` | `0.75` | MoveIt timeout riêng cho mỗi detour attempt (s) |
| `~detour_hold_poll_period` | `0.5` | Chu kỳ kiểm tra waypoint trong HOLD (s) |
| `~detour_z_offset` | `0.07` | Offset +Z mặc định; launch single/dual đặt lần lượt `0.15m`/`0.20m` |
| `~state_validity_service` | `/check_state_validity` | Service MoveIt kiểm tra goal joint state in-collision (fix B+C). Không có → escalation tắt, giữ hành vi cũ |

### MoveIt & Execution

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `~velocity_scaling` | `0.3` | Scale tốc độ thực thi (0–1) |
| `~acceleration_scaling` | `0.3` | Scale gia tốc thực thi (0–1) |
| `~planning_time` | `10.0` | Budget OMPL mỗi `plan()` (s). Launch đặt `1.0` để fail nhanh. Chỉ cắn khi plan khó; planner feasible trả về ngay khi có path |
| `~planning_attempts` | `10` | Số lần solve-from-scratch mỗi `plan()`, trả shortest. Launch đặt `1` (bỏ best-of-N) → giảm ~N× plan time trên success. Guard A* + `trajectory_is_safe` vẫn gác an toàn |
| `~sync_scene_with_planner` | `True` | Yêu cầu scene confirmed trước khi plan |
| `~scene_status_timeout_sec` | `30.0` | Tuổi tối đa scene status còn hợp lệ |

## Speed-scaled A* Padding + In-Motion Reroute (padding theo tốc độ robot)

Khi robot chạy nhanh, inflate obstacle A* rộng hơn để planner né xa hơn. Mặc định
`~astar_speed_padding_gain=0.0` → padding luôn 0 → **hành vi y như cũ** cho tới khi
tune trên hardware (calibration knob).

- **Đo tốc độ TCP (live):** `_update_tcp_speed()` chạy mỗi vòng execution-monitor
  (`~execution_monitor_rate`, 20 Hz), tính tốc độ từ delta vị trí end-effector / dt,
  EMA làm mượt (`~astar_speed_padding_smoothing_alpha`).
- **Speed → padding:** hàm thuần `speed_padding_meters(speed, deadband, gain, max)` →
  `pad_m = clamp(gain*(speed-deadband), 0, max)`; `_speed_padding_voxels()` đổi sang
  số voxel (`ceil(pad_m/voxel_size)`). Kiểm thử nhanh: `python3 planner_ab_replan_node.py --selftest`.
- **Inflate A*:** `active_human_voxels()` cộng thêm `max(pad_live, pad_latched)` voxel
  vào bán kính inflate. Lúc plan robot đứng yên → speed≈0 → padding 0 (backward compatible).
- **Two-tier re-check trong `execute_plan` (chạy khi đang di chuyển):**
  - **Tier 2 — quá gần:** emergency-stop sẵn có (`base_clearance + execution_speed_safety_margin`),
    dừng ngay, return `False` (status `EXECUTION_STOPPED_HUMAN_TOO_CLOSE`). Là safety floor, không đổi.
  - **Tier 1 — route phía trước bị chặn:** mỗi `~astar_recheck_period_sec`, chạy A* fresh từ
    voxel TCP hiện tại → `_exec_goal_voxel` với obstacle đã inflate theo speed. Nếu bị chặn:
    **không dừng**; latch `_reroute_pad_voxels` (giá trị padding đo được lúc đang chạy),
    publish `ASTAR_EXEC_REPLAN`, để segment ngắn hiện tại chạy hết tới waypoint. Reroute áp
    dụng ở plan của **leg kế tiếp** (option A: swap at next waypoint). Tier 2 vẫn bảo vệ trong segment.
- **Latch lifecycle:** set khi Tier 1 phát hiện chặn; `active_human_voxels()` dùng tới khi
  plan của waypoint kế tiêu thụ; clear ở đầu `execute_plan` (khi segment mới bắt đầu chuyển động).
- **Goal voxel:** `run_waypoint()` tính từ `target_tcp_pose_for_joint_map(joint_map)` và truyền
  vào `execute_plan(plan, goal_voxel=...)`. `None` (FK không có) → tắt re-check cho leg đó.
- **Waypoint nhỏ hơn = phản ứng nhanh hơn:** tăng `~resampled_waypoint_count` để segment ngắn,
  độ trễ reroute và swap-stop nhỏ. Breadcrumb cache warm-start cho replan. Không thêm code.

Lưu ý: re-check dùng `self.astar.plan_with_info()` rồi reset `_astar_last_goal=None` +
`_first_plan_done=False` để guard plan-time sau đó plan lại sạch (tránh lẫn state incremental).

Giới hạn: MoveIt không blend trajectory; "plan while moving" = **quyết định** chạy khi di
chuyển, **swap** quỹ đạo vẫn cần dừng ngắn tại waypoint. Non-stop blend cần `moveit_servo` (ngoài scope).

## Ghi chú thay đổi blocker

- `can_plan_to_waypoint()` không còn gate `skeleton_data_is_active()`.
- `astar_path_is_available()` không còn chặn khi thiếu skeleton; voxel markers vẫn publish từ tập obstacle hiện có.
- Detour không còn dùng `max_replan_attempts=0` vô hạn. Sau `max_detour_attempts`, planner publish `DETOUR_HOLD`, không replan và chỉ resume khi `hand_blocks_target_pose()` trả false.
- Theo cấu hình hiện tại, skeleton thiếu/stale làm tập điểm người rỗng nên HOLD xem waypoint đã clear.
- `latest_human_points()` và `trajectory_is_safe()` không còn gọi filter điểm gần link robot hoặc gần planned path. Rủi ro: robot self-detect có thể được xem là điểm người thật, nhưng task này ưu tiên không chặn skeleton/voxel build.

## Goal-collision escalation (fix B+C — chống kẹt OMPL vô hạn)

**Triệu chứng (từ log):** A* báo `ASTAR/OK path=2` nhưng OMPL báo `MOVEIT/FAILED reason=moveit_no_plan` lặp hàng trăm lần, mỗi lần ~vài chục ms (≪ `planning_time` 1.0s). `plan_ms` nhỏ ⇒ OMPL **reject start/goal joint state in-collision ngay**, không phải search hết giờ. Với `max_replan_attempts=0`, `run_waypoint()` retry mãi ⇒ robot đứng yên tới khi người rời thì goal mới hợp lệ. Detour không kích hoạt vì `hand_blocks_target_pose()` (đo khoảng cách điểm) bất đồng với collision toàn-thân-arm của OMPL.

**Nguyên nhân gốc:** A* chỉ validate đường đi **điểm TCP** trên voxel grid; OMPL validate **toàn bộ joint config + thân arm** trên PlanningScene. Link arm chạm người dù ô TCP trống ⇒ A* greenlight goal mà OMPL từ chối.

**Fix:**
- **B —** `run_waypoint()`: khi plan fail và `goal_state_in_collision(joint_map) is True` → escalate sang `run_detour_if_hand_blocks_waypoint` thay vì busy-retry goal chắc chắn invalid. Log `DETOUR/ESCALATE reason=goal_state_in_collision`.
- **C —** Cổng detour và release của HOLD dùng `waypoint_obstructed()` = tay chặn **HOẶC** goal in-collision (khớp OMPL), không chỉ `hand_blocks_target_pose()`. HOLD chỉ release khi goal thật sự hợp lệ.
- Quyết định tách thành hàm thuần `should_escalate_to_detour(hand_blocks, goal_in_collision)` (unit-check trong `--selftest`).

**An toàn:** Chỉ **thêm** một escape sớm + làm cổng detour bảo thủ hơn (khớp OMPL). Không nới lỏng safety nào. Service `check_state_validity` thiếu → `goal_state_in_collision` trả `None` → `should_escalate_to_detour(..., None)=False` ⇒ về đúng hành vi cũ. Tắt bằng `~state_validity_service` trỏ service không tồn tại.

**Kiểm thử:** `python3 planner_ab_replan_node.py --selftest` (assert cho `should_escalate_to_detour`).

## Obstacle Stability Cache (chống drop voxel A*)

**Lý do:** Obstacle voxel của A* được rebuild mỗi chu kỳ từ `latest_human_points()`. Skeleton jitter làm voxel index lật qua biên cell, cộng frame stale tạm thời → obstacle biến mất 1 chu kỳ → A* lập path xuyên vùng người (sai, mất an toàn).

**Cách hoạt động:** `active_human_voxels()` nay dùng `stable_human_points()`:
- Điểm người mới lệch ≤ `obstacle_stability_threshold` (mặc định 0.04m, dải 3–5cm) so với anchor cache → **giữ vị trí anchor cũ** ⇒ cùng voxel, obstacle không nhấp nháy.
- Điểm lệch quá ngưỡng hoặc điểm mới → thêm anchor mới.
- Anchor không thấy trong chu kỳ → giữ thêm `obstacle_cache_hold_sec` (mặc định 0.5s) để vượt dropout tạm thời, quá hạn mới **drop**.
- Điểm invalid (NaN/inf/ngoài map ± `voxel_size`) bị loại ⇒ "không thấy voxel hoặc voxel không hợp lệ thì drop".

**Phạm vi:** Chỉ tác động obstacle của A* guard. Các check real-time (emergency-stop, `trajectory_is_safe`, detour) vẫn dùng điểm raw để không giữ phantom khi người đã rời.

**An toàn:** Thay đổi này **tăng** độ bảo thủ (obstacle giữ lâu hơn, không drop) ⇒ không làm yếu safety logic. Tắt bằng `~enable_obstacle_cache=false` để về hành vi cũ.

## Bone Obstacles (build dọc đoạn nối khớp)

**Lý do:** Trước đây obstacle A* chỉ inflate quanh **khớp** (joint spheres). Giữa 2 khớp có khe → A* có thể luồn path qua chi người. Yêu cầu: build obstacle quanh cả **đường nối khớp**.

**Cách hoạt động:** `_bone_obstacle_voxels()` với mỗi cặp trong `bone_connection_pairs` (mặc định: tay trái/phải shoulder→elbow→wrist, 2 vai, vai→hông, 2 hông):
- Lấy 2 đầu khớp từ `latest_skeleton_by_id` (cần joint id), bỏ nếu thiếu/invalid.
- Sample dọc đoạn ở bước nửa voxel → tập voxel đường (dedupe).
- Inflate mỗi voxel đường `bone_inflate_radius` (mặc định 1 = ống mỏng) → union vào `active_human_voxels()`.

**Chi phí:** Ống mỏng, dedupe bằng set. Bone 0.4m ≈ 5 voxel đường → ~63 voxel (r=1); 8 bone → vài trăm voxel. ARA* vẫn bị cap `ara_max_time_ms`.

**An toàn:** Tăng coverage (lấp khe giữa khớp) ⇒ **tăng** an toàn, không làm yếu. Tắt: `~enable_bone_obstacles=false`. Muốn dày hơn: tăng `~bone_inflate_radius` (đánh đổi tốc độ A*).

**Giới hạn:** Bones dùng `latest_skeleton_by_id` raw (không qua stability cache); khi skeleton dropout, joint vẫn được cache giữ nhưng bone tạm biến mất tới frame kế. Joint coverage vẫn còn.

## Region Preference (ưu tiên vùng shoulder_pan)

**Lý do:** Robot lắp thêm dây/cable nén lên → không di chuyển thoải mái, chỉ hoạt động trong vùng quét bởi 7 waypoint A→B (cung shoulder_pan, TCP ~ x[-0.27,0.32] y[-0.29,-0.10] z[0.09,0.42]). Muốn A* **ưu tiên cao** path nằm trong vùng này.

**Cách hoạt động:** `_setup_region_preference()` (gọi cuối `__init__`):
- FK 7 waypoint (`_joint_map_goal_xyz`) → bbox vùng + nới `region_margin`.
- Duyệt toàn grid → voxel **ngoài** bbox → tập penalty.
- `astar.set_penalty_cells(penalty, region_penalty_weight)`.

A* `cost()` cộng `region_penalty_weight` mỗi bước vào voxel ngoài vùng → tổng cost = quãng đường + Σ penalty. A* đổi `weight` cost lấy đường ngắn → **bám vùng trừ khi ra ngoài tiết kiệm nhiều hơn**.

**Soft, không hard-block:** vẫn cho ra ngoài vùng khi buộc (tránh người), không làm A* fail. Heuristic giữ Euclid (admissible). Tắt: `~enable_region_preference=false`. Bám chặt hơn: tăng `~region_penalty_weight`.

**An toàn:** Không đụng obstacle/safety. Chỉ thêm chi phí mềm định hướng. Nếu FK waypoint thất bại (fk_client None) → tự tắt + warn.

> **Lưu ý quan trọng:** region penalty này CHỈ tác động **A\* guard**, không ràng buộc quỹ đạo OMPL thực thi. Robot vẫn có thể vung ra ngoài vùng → xem **Pan Limit** bên dưới để chặn cứng.

## Pan Limit — HARD constraint (chống đứt dây khí nén)

**Lý do:** Region penalty (soft) chỉ ở A* guard; **đường robot chạy thật = MoveIt OMPL** không biết vùng → vung `shoulder_pan` ra ngoài → dễ đứt dây khí nén. Cần chặn cứng trên chính OMPL.

**Cách hoạt động:** `_setup_pan_limit()` (cuối `__init__`):
- Lấy `shoulder_pan` min/max của 7 waypoint A→B (hoặc override `~shoulder_pan_min/max`), nới `~pan_limit_margin`.
- Tạo `moveit_msgs/Constraints` với `JointConstraint(shoulder_pan_joint, position=center, tolerance_above/below)` phủ dải đó.
- `group.set_path_constraints(...)` → **mọi** MoveIt plan (kể cả detour) giữ pan trong dải suốt quỹ đạo.

Mặc định (từ waypoint hiện tại): `shoulder_pan ∈ [-2.405, 0.123] rad = [-137.8°, 7.1°]`.

**Khác region penalty:** đây là ràng buộc **cứng trên OMPL** → quỹ đạo thực thi **không** ra ngoài dải pan. Tắt: `~enable_pan_limit=false`.

**Đánh đổi / rủi ro:**
- Planning có path constraint **khó hơn** → có thể tăng `MOVEIT/FAILED`. Nên tăng `~planning_attempts` (1→3–5) và/hoặc `~planning_time`.
- **Start state phải nằm trong dải pan**, nếu robot khởi động ở pan ngoài dải → plan đầu fail. Đảm bảo home trong dải.
- Chỉ giới hạn `shoulder_pan`. Nếu dây căng theo khớp khác (shoulder_lift…) cần thêm JointConstraint tương tự.

## Breadcrumb Cache (warm-start khi gặp vật cản)

**Lý do:** Planner chạy A↔B lặp. Route đã đi (pose từng free) tái dùng được để vượt block nhanh hơn thay vì plan lại từ đầu.

**Class `BreadcrumbCache`:**
- **Ghi** `(joint_config, tcp_xyz, stamp)` mỗi `breadcrumb_record_period` (0.5s) trong vòng monitor của `execute_plan` (cùng thread execution → an toàn). Chỉ ghi khi robot dịch ≥ `breadcrumb_revisit_threshold` (tránh trùng khi đứng yên).
- **Trim loop-closure:** về lại gần node cũ → drop prefix trước node đó. Vd `A B C D E` rồi về gần B → `B C D E B` (bỏ A). Cap `breadcrumb_max_nodes`.
- **Query** `nearest_valid(goal, is_valid)`: trả joint config của node gần goal nhất mà TCP **còn collision-free** với obstacle hiện tại.

**Dùng khi blocked (`run_waypoint`):** Lần đầu `plan_to_joint_map` fail → `_try_breadcrumb_hop()`: hop tới node cache còn-valid gần goal (1 hop/waypoint), rồi retry target thật.

**An toàn (quan trọng):** Cache **chỉ cung cấp ứng viên waypoint**. Mọi chuyển động hop vẫn qua đủ cổng an toàn: `astar_path_is_available` (ARA*), `trajectory_is_safe`, emergency-stop trong `execute_plan`. Node cache stale được **re-validate** với `active_human_voxels()` hiện tại trước khi dùng. Không thay/yếu hóa safety gate. Tắt: `~enable_breadcrumb=false` hoặc `~enable_breadcrumb_fallback=false`.

**Giới hạn:** Cache chỉ giúp khi detour trùng route đã đi (đúng với A↔B); goal mới ở vùng chưa tới → fallback về planner thường. Log sự kiện `BREADCRUMB/HOP|HOP_PLAN_FAILED|HOP_EXEC_FAILED`.

## Plan Event Logging (ghi log ra file)

**Lý do:** Cần truy vết vì sao plan thất bại (vật cản chặn), hoặc plan xong mà không thực thi, kèm tọa độ thực.

**Cách hoạt động:** Class `PlanLogger` mở thư mục `~plan_log_dir` (mặc định `<package>/logs`), mỗi lần chạy tạo `plan_log_<timestamp>.csv` (máy đọc) + `.log` (người đọc). Flush mỗi sự kiện (sống sót khi crash). Đóng file qua `rospy.on_shutdown`.

**Sự kiện + hook:**
| event/status | Hook | Ý nghĩa |
|--------------|------|---------|
| `ASTAR/OK` | `astar_path_is_available_to_pose` | ARA* có đường |
| `ASTAR/BLOCKED` | nt | Không plan được vì vật cản (reason `no_path:*`) |
| `MOVEIT/FAILED` | `plan_to_joint_map` | A* OK nhưng OMPL fail |
| `PLAN/NOT_EXECUTED` | `plan_to_joint_map` / `execute_plan` | Plan xong không chạy (unsafe / auto_execute=false) |
| `TRAJECTORY/TOO_CLOSE` | `trajectory_is_safe` | Trajectory quá gần người (link + human xyz + dist) |
| `EXECUTE/STOPPED_HUMAN_TOO_CLOSE` | `execute_plan` | Emergency stop khi chạy |
| `EXECUTE/FAILED` | `execute_plan` | MoveIt execute false |
| `EXECUTE/SUCCESS` | `execute_plan` | Chạy xong (min_dist) |

**Tọa độ ghi (m, frame `base_link`):** `start_x/y/z` (TCP/link), `goal_x/y/z` (đích), `human_x/y/z` (điểm người gần nhất), `min_dist_m`, `n_obstacles`, `n_human_pts`, `reason`, `detail`.

**An toàn:** Chỉ ghi log, không đổi logic plan/safety. Tắt: `~enable_plan_log=false`.

## Sphere Inflation (giảm số voxel obstacle)

**Lý do:** `inflate_voxel` cũ fill **khối cube đặc** `(2r+1)³`. Góc cube nằm cách tâm tới `r·√3` voxel — **ngoài** bán kính clearance `r` → voxel thừa, obstacle quá dày, nhiều.

**Cách hoạt động:** `obstacle_inflate_sphere=true` (mặc định) chỉ giữ voxel có `dx²+dy²+dz² ≤ r²` → quả cầu clearance đúng bán kính. Áp cho cả khớp lẫn bones.

| radius | cube | sphere | giảm |
|--------|------|--------|------|
| 1 (bones) | 27 | 7 | 3.86× |
| 2 (khớp, voxel 0.05) | 125 | 33 | 3.79× |
| 3 | 343 | 123 | 2.79× |

**An toàn:** Voxel bị bỏ là **góc ngoài clearance** (xa hơn `r` voxel) ⇒ quả cầu clearance giữ nguyên, **không** thu nhỏ vùng an toàn. Tắt về cube: `~obstacle_inflate_sphere=false`.

## Continuous Obstacle Publisher (luôn build, không drop)

**Lý do:** Trước đây `active_human_voxels()` + `publish_astar_markers()` **chỉ chạy trong lúc planning** (`astar_path_is_available_to_pose`). Giữa các plan (robot execute/idle/HOLD) hoặc khi người ở xa nên planner ít gọi guard → obstacle không rebuild → markers stale/clear → trông như **drop**, và "voxel xa robot thì không build obs".

**Cách hoạt động:** `rospy.Timer` ở `obstacle_publish_rate` Hz (mặc định 10) gọi `render_astar_markers()`:
- Build obstacle **mới mỗi tick** từ `stable_human_points()` ⇒ luôn build khi có voxel hợp lệ, **bất kể robot xa/gần hay đang plan hay không**.
- Publish `/hrc_astar_voxel_markers` liên tục ⇒ obstacle không drop.
- Overlay path/start/goal lấy từ `store_plan_viz()` của plan gần nhất, chỉ hiển thị trong `obstacle_path_viz_hold` giây.
- `~obstacle_publish_rate <= 0` → tắt timer, về hành vi cũ (chỉ build khi plan).

**Thread-safety:** Timer chạy thread riêng. `_obstacle_lock` bảo vệ obstacle cache (`_obstacle_anchors`) và viz state (`_viz_*`) dùng chung giữa thread planner và thread timer. Các critical section ngắn, không lồng nhau ⇒ không deadlock.

**Phạm vi:** Chỉ ảnh hưởng build/visualize obstacle A*. Guard ARA* lúc plan vẫn tự tính obstacle như cũ (dùng chung cache nên nhất quán).

**An toàn:** Obstacle luôn hiện diện ⇒ A* không còn lập path xuyên vùng người do obstacle biến mất giữa chu kỳ. Tăng bảo thủ, không làm yếu safety.

## Validation

```bash
python3 -m py_compile scripts/planner_ab_replan_node.py
python3 test/test_planner_detour_retry.py
python3 test/test_collision_visualization.py
python3 test/test_pose_array_schema.py
```

Runtime: giữ tay tại waypoint ít nhất 60 giây; xác nhận attempt không vượt `max_detour_attempts`, planner không giữ 100% CPU, `/human_collision_object` không tụt và tay rời thì path tiếp tục.

### Safety implications và giới hạn

- HOLD không gọi MoveIt và không execute trajectory, nên robot giữ nguyên vị trí sau khi detour thất bại.
- ARA* gate chạy trước mỗi detour OMPL attempt để tránh timeout MoveIt khi voxel path đã blocked.
- Skeleton thiếu/stale được xem là clear theo cấu hình hiện tại; đây là giới hạn fail-open cần xác nhận trong runtime safety review.
- `detour_z_offset` cần tune theo workspace thật; dual launch tăng lên `0.20m` để vượt clearance `0.15m`, nhưng vẫn phải kiểm tra va chạm thực tế.

---

## ROS Topics & Services

| Tên | Type | Chiều | Mô tả |
|-----|------|-------|-------|
| `/human_skeleton_base` | `PoseArray` | Subscribe | Skeleton người |
| `/moveit_scene_status` | `String` | Subscribe | Trạng thái MoveIt scene |
| `/ur3_fixed_joint_path_status` | `String` | Publish | Trạng thái planner |
| `/hrc_path_text` | `String` | Publish | ARA* status: `ASTAR_OK ...` / `NO_ASTAR_PATH ...` |
| `/hrc_astar_voxel_markers` | `MarkerArray` | Publish | ARA* voxel visualization RViz |
| `/hrc_planning_time_ms` | `Float32` | Publish | Tổng thời gian plan (ms) |
| `/hrc_astar_planning_time_ms` | `Float32` | Publish | Thời gian ARA* riêng (ms) |
| `/hrc_moveit_planning_time_ms` | `Float32` | Publish | Thời gian MoveIt riêng (ms) |
| `/hrc_execution_time_ms` | `Float32` | Publish | Thời gian thực thi trajectory (ms) |
| `/compute_fk` | `moveit_msgs/GetPositionFK` | Service Client | Forward kinematics |

---

*Xem thêm: [MODULE_G_astar_improved_3d.md](MODULE_G_astar_improved_3d.md) — ARA* library được import bởi module này.*
*Xem thêm: [MODULE_E_moveit_scene_manager.md](MODULE_E_moveit_scene_manager.md) — cung cấp scene status.*

> ARA* voxel chỉ tồn tại nội bộ trong planner; không cung cấp topic voxel trung gian.
