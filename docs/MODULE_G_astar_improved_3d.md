# MODULE G — `astar_improved_3d.py`

> **Loại:** Thư viện thuần Python (không phải ROS node)
> **Vai trò:** Thuật toán **ARA\* (Anytime Repairing A\*)** trong không gian voxel 3D. Được import bởi `planner_ab_replan_node.py` dùng làm guard planner.

---

## Tổng quan ARA*

ARA* (Anytime Repairing A*) là biến thể của A* có thể:
1. Tìm ngay lời giải **chưa tối ưu** với `ε > 1` (nhanh)
2. Dần cải thiện lời giải bằng cách giảm `ε` cho đến khi hết time budget
3. Tái sử dụng (reuse) kết quả cũ khi obstacle thay đổi nhỏ (`replan`)

```
plan_with_info():
  ε = epsilon_start  (ví dụ 3.0)
  while ε ≥ epsilon_final AND còn time budget:
      _improve_path(ε)     ← weighted A* với heuristic × ε
      ε -= epsilon_decay   ← giảm ε → solution tốt hơn
      nếu path tìm được → rebuild, tiếp tục cải thiện
  trả về PlanResult với path tốt nhất + epsilon_satisfied

replan_with_info():
  Dùng lại INCONS từ lần plan trước
  Update chỉ voxel thay đổi (changed_obstacle_count)
  Chạy _ara_search() mới từ start mới
```

---

## Class: `AStarImproved3D`

### Khởi tạo

| Tham số `__init__` | Mặc định | Mô tả |
|---------------------|----------|-------|
| `size_x, size_y, size_z` | — | Kích thước lưới voxel (số voxel theo 3 trục) |
| `diagonal` | `True` | `True` = 26-connectivity (kể cả đường chéo), `False` = 6-connectivity |
| `epsilon_start` | `1.5` | Hệ số heuristic ban đầu (nhỏ → chính xác hơn, chậm hơn) |
| `epsilon_final` | `1.0` | Hệ số heuristic tối thiểu (1.0 = A* chính xác) |
| `epsilon_decay` | `0.2` | Giảm ε mỗi iteration (nhỏ → nhiều bước refinement hơn) |
| `max_time_ms` | `200.0` | Ngân sách thời gian tối đa (ms) |
| `max_steps` | `150000` | Số bước expand tối đa |
| `smooth` | `True` | Áp dụng string-pulling (line-of-sight) sau khi tìm đường |

### Hàm lập kế hoạch

| Phương thức | Tham số | Trả về | Chức năng |
|-------------|---------|--------|-----------|
| `plan_with_info(start, goal, obstacles)` | Voxel, Voxel, Set[Voxel] | `PlanResult` | Lập kế hoạch lần đầu (ARA*: nhiều iteration _improve_path với ε giảm dần) |
| `replan_with_info(new_start, obstacles)` | Voxel, Set[Voxel] | `PlanResult` | Tái lập kế hoạch: reuse INCONS, chỉ update voxel thay đổi |
| `plan(start, goal, obstacles)` | Voxel, Voxel, Set[Voxel] | `List[Voxel]` | Wrapper: gọi `plan_with_info`, trả về path list (rỗng nếu thất bại) |
| `replan(new_start, obstacles)` | Voxel, Set[Voxel] | `List[Voxel]` | Wrapper: gọi `replan_with_info`, trả về path list |

### Hàm hỗ trợ

| Phương thức | Tham số | Chức năng |
|-------------|---------|-----------|
| `neighbors(s) → List[Voxel]` | Voxel | Lấy voxel kề: 6 hướng (`diagonal=False`) hoặc 26 hướng (`diagonal=True`) |
| `set_penalty_cells(cells, weight) → None` | `Set[Voxel]`, float | Soft region preference: bước vào voxel trong `cells` cộng thêm `weight` cost (không hard-block). `weight<=0` = tắt |
| `cost(a, b) → float` | Voxel, Voxel | Chi phí: `1.0` thẳng, `√2` chéo 2D, `√3` chéo 3D; **+`weight`** nếu `b` ∈ penalty cells |
| `heuristic(a, b) → float` | Voxel, Voxel | Ước lượng Euclidean: `√(dx²+dy²+dz²)` |
| `path_cost(path) → float` | `List[Voxel]` | Tổng chi phí path |
| `line_of_sight(a, b) → bool` | Voxel, Voxel | 3D Bresenham: True nếu không có obstacle giữa a và b |
| `smooth_path(path) → List[Voxel]` | `List[Voxel]` | String-pulling: loại waypoint trung gian có thể bỏ qua (line-of-sight), giảm zigzag → MoveIt dễ phối hợp khớp hơn |
| `filter_obstacles(obstacles) → (Set, valid_count, invalid_count)` | `Set[Voxel]` | Lọc voxel obstacle: loại out-of-bounds, trả về set hợp lệ + thống kê |

---

## Dataclass: `PlanResult`

```python
@dataclass
class PlanResult:           # Cũng export là DStarResult để tương thích
    path: List[Voxel]       # Danh sách voxel từ start đến goal
    success: bool           # True nếu tìm được đường
    reason: str             # Lý do kết quả (xem bảng reason bên dưới)
    metrics: Dict[str, object]
    # metrics chứa:
    #   expanded_steps: int         — số voxel đã expand
    #   epsilon_satisfied: float    — ε tốt nhất đạt được
    #   elapsed_ms: float           — thời gian thực thi (ms)
    #   changed_obstacle_count: int — số voxel thay đổi (chỉ trong replan)
    #   iterations: int             — số ARA* iteration
```

---

## Reason Codes

| Reason | Mô tả |
|--------|-------|
| `OK` | Tìm được đường đi |
| `NO_PATH` | Không tìm được đường (đã expand hết không gian khả năng) |
| `GOAL_BLOCKED` | Voxel goal bị obstacle chiếm |
| `START_BLOCKED` | Voxel start bị obstacle chiếm |
| `GOAL_OUT_OF_BOUNDS` | Goal nằm ngoài biên lưới voxel |
| `START_OUT_OF_BOUNDS` | Start nằm ngoài biên lưới voxel |
| `MAX_STEPS_REACHED` | Vượt quá `max_steps` — hết budget expand |
| `TIMEOUT` | Vượt quá `max_time_ms` |
| `PATH_EXTRACTION_FAILED` | Không reconstruct được path từ came_from map |
| `REPLAN_NOT_INITIALIZED` | Gọi `replan` trước khi gọi `plan` lần đầu |
| `EMPTY_GRID` | Lưới voxel kích thước 0 |
| `INVALID_START` | Start không phải tuple/list 3 phần tử |
| `INVALID_GOAL` | Goal không phải tuple/list 3 phần tử |

---

## Cách dùng (trong planner)

```python
from astar_improved_3d import AStarImproved3D, PlanResult

# Khởi tạo 1 lần
planner = AStarImproved3D(
    size_x=40, size_y=40, size_z=20,
    diagonal=True,
    epsilon_start=3.0,
    epsilon_final=1.0,
    epsilon_decay=0.5,
    max_time_ms=50.0,
    max_steps=50000
)

# Lần đầu: plan
obstacles = {(5,5,3), (5,6,3), (6,5,3)}  # set voxel bị người chiếm
result: PlanResult = planner.plan_with_info(
    start=(0,0,5),
    goal=(30,20,8),
    obstacles=obstacles
)
if result.success:
    path = result.path  # List[Voxel]

# Khi obstacle thay đổi nhỏ: replan (nhanh hơn plan lại từ đầu)
new_obstacles = {(5,5,3), (5,6,3), (7,5,3)}  # 1 voxel thay đổi
result = planner.replan_with_info(
    new_start=(3,2,5),
    obstacles=new_obstacles
)
```

---

## Lưu ý triển khai

- **Voxel** = `tuple(int, int, int)` — index trong lưới 3D
- **Tọa độ thế giới → Voxel:** thực hiện trong `planner_ab_replan_node.py` (không trong module này)
- `DStarResult` là alias của `PlanResult` (tương thích ngược với code cũ dùng D*Lite)
- Khi `diagonal=True`: chi phí không đồng nhất (1.0 / √2 / √3) → path tự nhiên hơn, ít bậc thang
- `epsilon_start=3.0` nghĩa là heuristic có thể phóng đại 3× → plan nhanh nhưng path dài hơn tối ưu ≤3×

---

*Xem thêm: [MODULE_F_planner_ab_replan_node.md](MODULE_F_planner_ab_replan_node.md) — module sử dụng ARA* này.*
