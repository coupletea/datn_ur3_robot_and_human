# UR3 HRC Planner — Module Documentation Index

> Mỗi file mô tả 1 module: cấu hình (ROS params), hàm, topics/services.

---

## Danh sách module

| Module | File | Loại | Vai trò |
|--------|------|------|---------|
| **A** — `kinect_skeleton_tracker.py` | [MODULE_A_kinect_skeleton_tracker.md](MODULE_A_kinect_skeleton_tracker.md) | ROS Node | Đọc Kinect, nhận diện skeleton (YOLO + MediaPipe) |
| **B** — `data_skeleton.py` | [MODULE_B_data_skeleton.md](MODULE_B_data_skeleton.md) | Library | Tiện ích camera, depth, YOLO, MediaPipe, skeleton |
| **C** — `dual_kinect_fusion_controller.py` | [MODULE_C_dual_kinect_fusion_controller.md](MODULE_C_dual_kinect_fusion_controller.md) | ROS Node | Fuse 2 Kinect → 1 skeleton + điều phối duty-cycle rate tracker (Dual Kinect only) |
| **D** — `skeleton_obstacle_builder.py` | [MODULE_D_skeleton_obstacle_builder.md](MODULE_D_skeleton_obstacle_builder.md) | ROS Node | Skeleton → CollisionObject + marker đỏ cùng geometry |
| **E** — `moveit_scene_manager.py` | [MODULE_E_moveit_scene_manager.md](MODULE_E_moveit_scene_manager.md) | ROS Node | Apply CollisionObject vào MoveIt planning scene |
| **F** — `planner_ab_replan_node.py` | [MODULE_F_planner_ab_replan_node.md](MODULE_F_planner_ab_replan_node.md) | ROS Node | Lập kế hoạch + thực thi UR3 (ARA* guard + MoveIt) |
| **G** — `astar_improved_3d.py` | [MODULE_G_astar_improved_3d.md](MODULE_G_astar_improved_3d.md) | Library | ARA* (Anytime Repairing A*) voxel 3D guard planner |

---

## Luồng dữ liệu

```
Single Kinect:
  [A] kinect_skeleton_tracker
         ↓ /human_skeleton_base (PoseArray)
         ├──→ [D] skeleton_obstacle_builder
         │          ↓ /human_collision_object (CollisionObject)
         │          └──→ [E] moveit_scene_manager
         │                     ↓ /moveit_scene_status
         ├──→ [F] planner_ab_replan_node  ← (cũng đọc /moveit_scene_status)
         │          ↑ [G] AStarImproved3D (import nội bộ)

Dual Kinect:
  [A] kinect_skeleton_tracker (front)  →  /kinect_front/human_skeleton_base
  [A] kinect_skeleton_tracker (back)   →  /kinect_back/human_skeleton_base
         ↓
  [C] dual_kinect_fusion_controller
         ↓ /human_skeleton_base (fused)
         → [D] → [E] → [F] (giống Single Kinect)
```

---

## Dependencies

| Module | Import từ |
|--------|-----------|
| A (`kinect_skeleton_tracker`) | B (`data_skeleton`) |
| F (`planner_ab_replan_node`) | G (`astar_improved_3d`) |
| C, D, E, F | ROS, MoveIt (external) |

---

*Tài liệu tổng quan đầy đủ: [../PROJECT_STRUCTURE.md](../PROJECT_STRUCTURE.md)*
