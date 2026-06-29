# UR3 HRC Start Checklist

Checklist nay dung cho pipeline hien tai sau update dual Kinect. Mac dinh luon chay `auto_execute:=false` truoc. Chi bat `auto_execute:=true` khi robot, TF, Kinect, skeleton, collision object, PlanningScene va planner deu dung.

## 0. Cau Hinh Hien Tai

| Muc | Gia tri |
| --- | --- |
| Ubuntu IP | `192.168.100.2` |
| Robot IP | `192.168.100.100` |
| Teach Pendant External Control Port | `50002` |
| ROS `reverse_port` | `50001` |
| ROS `script_sender_port` | `50002` |
| Workspace | `~/catkin_ws` |
| Package chinh | `ur3_hrc_planner` |
| Launch 1 Kinect front | `ur3_hrc_planner system.launch` |
| Launch 1 Kinect back | `ur3_hrc_planner system_back.launch` |
| Launch 2 Kinect | `ur3_hrc_planner dual_kinect_system.launch` |
| Launch back perception test | `ur3_hrc_planner back_kinect_perception_test.launch` |
| Planner node | `planner_ab_replan_node.py` |
| Perception node | `kinect_skeleton_tracker.py` |
| Fusion node | `dual_kinect_fusion_controller.py` |
| Obstacle node | `skeleton_obstacle_builder.py` |
| Scene node | `moveit_scene_manager.py` |

## 1. Kiem Tra Source Va Build

Workspace nay dang build bang `catkin_make`. Neu chay `catkin build` se bao build space da duoc tao bang `catkin_make`.

```bash
cd ~/catkin_ws
source ~/env_ur3.sh

python3 -m py_compile src/ur3_hrc_planner/scripts/*.py
xmllint --noout src/ur3_hrc_planner/package.xml src/ur3_hrc_planner/launch/*.launch
catkin_make --pkg ur3_hrc_planner
source devel/setup.bash
```

Kiem tra launch 1 Kinect:

```bash
roslaunch --nodes ur3_hrc_planner system.launch
```

Can thay:

```text
/base_to_kinect_link
/base_to_kinect_ir_optical
/kinect_skeleton_tracker
/skeleton_obstacle_builder
/moveit_scene_manager
/planner_ab_replan_node
```

Kiem tra launch 2 Kinect:

```bash
roslaunch --nodes ur3_hrc_planner dual_kinect_system.launch
```

Can thay:

```text
/base_to_kinect_front_link
/base_to_kinect_front_ir_optical
/base_to_kinect_back_link
/base_to_kinect_back_ir_optical
/kinect_front/kinect_skeleton_tracker
/kinect_back/kinect_skeleton_tracker
/dual_kinect_fusion_controller
/skeleton_obstacle_builder
/moveit_scene_manager
/planner_ab_replan_node
```

Kiem tra Python runtime:

```bash
python3 - <<'PY'
import ultralytics, mediapipe, pylibfreenect2
print("ultralytics OK", ultralytics.__version__)
print("mediapipe OK", mediapipe.__version__)
print("pylibfreenect2 OK")
PY
```

## 2. Luu Y Kinect Va `kinect2_bridge`

`kinect_skeleton_tracker.py` tu mo Kinect bang `pylibfreenect2`. Khong chay `kinect2_bridge` song song voi tracker tren cung Kinect, vi se tranh thiet bi.

Chi dung `kinect2_bridge` de smoke test phan cung:

```bash
source ~/env_ur3.sh
rospack find kinect2_bridge
roslaunch --nodes kinect2_bridge kinect2_bridge.launch
roslaunch kinect2_bridge kinect2_bridge.launch
```

Kiem tra topic khi smoke test:

```bash
rostopic list | grep -E "kinect2|image_color|image_depth|camera_info|points"
rosrun image_view image_view image:=/kinect2/qhd/image_color
rosrun image_view image_view image:=/kinect2/qhd/image_depth_rect
```

Stop bridge truoc khi chay HRC tracker:

```bash
pkill -f kinect2_bridge
```

## 3. Cau Hinh Teach Pendant

Trong program moi chi de 1 node:

```text
External Control
```

Cau hinh External Control:

```text
Host IP   : 192.168.100.2
Host Name : 192.168.100.2
Port      : 50002
```

Sau do:

```text
Save Program
Stop Program
Chua bam Play
```

Neu co EtherNet/IP:

```text
Installation -> Fieldbus / EtherNet/IP -> Disabled
Save Installation
Reboot controller neu vua doi
```

## 4. Don Sach ROS Cu

```bash
pkill -9 -f ur_robot_driver
pkill -9 -f ur_hardware_interface
pkill -9 -f roslaunch
pkill -9 -f roscore
pkill -9 -f rosmaster
pkill -9 -f move_group
pkill -f kinect2_bridge
```

Kiem tra:

```bash
ps aux | grep -E "ur_robot_driver|ur_hardware_interface|roslaunch|roscore|rosmaster|move_group|kinect2_bridge" | grep -v grep
sudo ss -ltnp | grep 11311
```

Neu khong hien gi la sach.

## 5. Load Moi Truong

```bash
source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

echo $ROS_MASTER_URI
echo $ROS_IP
echo $ROS_HOSTNAME
```

Ket qua dung:

```text
ROS_MASTER_URI=http://192.168.100.2:11311
ROS_IP=192.168.100.2
ROS_HOSTNAME rong
```

## 6. Kiem Tra Ket Noi Robot

```bash
source ~/env_ur3.sh

ip route get 192.168.100.100
ping 192.168.100.100

nc -vz 192.168.100.100 29999
nc -vz 192.168.100.100 30004
```

Kiem tra trang thai robot:

```bash
echo "PolyscopeVersion" | nc 192.168.100.100 29999
echo "robotmode" | nc 192.168.100.100 29999
echo "programState" | nc 192.168.100.100 29999
echo "running" | nc 192.168.100.100 29999
```

Neu robot dang chay program cu:

```bash
echo "stop" | nc 192.168.100.100 29999
```

## 7. Terminal 1, Chay ROS Core

```bash
source ~/env_ur3.sh
roscore
```

De terminal nay chay nguyen.

## 8. Terminal 2, Chay UR Driver

```bash
source ~/env_ur3.sh

roslaunch ur_robot_driver ur3_bringup.launch \
  robot_ip:=192.168.100.100 \
  reverse_ip:=192.168.100.2 \
  reverse_port:=50001 \
  script_sender_port:=50002
```

Doi driver len on, chua bam Play voi.

## 9. Terminal 3, Kiem Tra Port Driver Roi Bam Play

```bash
source ~/env_ur3.sh
sudo ss -ltnp | grep -E "50001|50002"
```

Can thay LISTEN o port `50001` va `50002`. Sau do moi bam Play tren pendant.

Kiem tra sau khi bam Play:

```bash
rostopic echo /ur_hardware_interface/robot_program_running
rostopic hz /joint_states
```

Ket qua dung:

```text
data: True
```

## 10. Terminal 4, Chay MoveIt

```bash
source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

roslaunch ur3_moveit_config move_group.launch
```

Kiem tra:

```bash
rosnode list | grep move_group
rosservice list | grep -E "apply_planning_scene|get_planning_scene|compute_fk"
```

## 11. Terminal 5A, Chay HRC 1 Kinect Plan-Only

Dung mode nay neu chi co 1 Kinect. Hai launch nay deu publish topic non-namespaced (`/human_skeleton_base`, `/human_skeleton_status`, `/kinect_raw/image_raw`, `/kinect_skeleton/image_raw`).

Kinect truoc (front, serial `000441552747`, frame `kinect_front_ir_optical_frame`):

```bash
source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

roslaunch ur3_hrc_planner system.launch auto_execute:=false
```

Kinect sau (back, serial `501294742442`, frame `kinect_back_ir_optical_frame`):

```bash
source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

roslaunch ur3_hrc_planner system_back.launch auto_execute:=false
```

Kiem tra node (chung cho ca front va back):

```bash
rosnode list | grep -E "kinect_skeleton_tracker|skeleton_obstacle_builder|moveit_scene_manager|planner_ab_replan_node"
```

Kiem tra perception:

```bash
rostopic echo /human_skeleton_status
rostopic hz /human_skeleton_base
rostopic echo /human_skeleton_base -n 1
rqt_image_view /kinect_raw/image_raw
rqt_image_view /kinect_skeleton/image_raw
```

Kiem tra TF (dung frame theo launch dang chay):

```bash
# system.launch (front)
rosrun tf tf_echo base_link kinect_front_ir_optical_frame
# system_back.launch (back)
rosrun tf tf_echo base_link kinect_back_ir_optical_frame
rosrun tf tf_echo base_link tool0
```

## 12. Terminal 5B, Chay HRC 2 Kinect Plan-Only

Dung mode nay neu co 2 Kinect. Khuyen nghi dung serial de tranh doi index giua cac lan cam USB.

```bash
source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

roslaunch ur3_hrc_planner dual_kinect_system.launch auto_execute:=false
```

Neu biet serial:

```bash
roslaunch ur3_hrc_planner dual_kinect_system.launch \
  front_camera_serial:=SERIAL_FRONT \
  back_camera_serial:=SERIAL_BACK \
  auto_execute:=false
```

Kiem tra node:

```bash
rosnode list | grep -E "kinect_front|kinect_back|dual_kinect_fusion_controller|skeleton_obstacle_builder|moveit_scene_manager|planner_ab_replan_node"
```

Kiem tra status va output:

```bash
rostopic echo /kinect_front/human_skeleton_status
rostopic echo /kinect_back/human_skeleton_status
rostopic echo /human_skeleton_fusion_status

rostopic hz /kinect_front/human_skeleton_base
rostopic hz /kinect_back/human_skeleton_base
rostopic hz /human_skeleton_base

rostopic hz /kinect_front/kinect_raw/image_raw
rostopic hz /kinect_back/kinect_raw/image_raw

rqt_image_view /kinect_front/kinect_raw/image_raw
rqt_image_view /kinect_back/kinect_raw/image_raw
```

Mo anh raw ca 2 Kinect cung luc:

```bash
source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

rosrun image_view image_view image:=/kinect_front/kinect_raw/image_raw __name:=front_kinect_raw_view &

source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

rosrun image_view image_view image:=/kinect_back/kinect_raw/image_raw __name:=back_kinect_raw_view &
```

Anh debug co skeleton/overlay neu can:

```bash
rqt_image_view /kinect_front/kinect_skeleton/image_raw
rqt_image_view /kinect_back/kinect_skeleton/image_raw
```

Luu y: topic raw duoc `dual_kinect_system.launch` tao san qua `kinect_skeleton_tracker.py`. Khong chay them `kinect2_bridge` song song, vi tracker dang tu mo Kinect bang `pylibfreenect2`. Neu `/kinect_back/kinect_raw/image_raw` khong co Hz, check `/kinect_back/human_skeleton_status` truoc; thuong la back Kinect dang `CAMERA_ERROR` hoac `FRAME_TIMEOUT`. Neu nghi 2 Kinect mo cung luc bi loi USB/libfreenect2, chay lai voi `back_camera_startup_delay_sec:=2.0`; neu nghi index bi dao, chay bang serial.

Kiem tra TF:

```bash
rosrun tf tf_echo base_link kinect_front_ir_optical_frame
rosrun tf tf_echo base_link kinect_back_ir_optical_frame
rosrun tf tf_echo base_link tool0
```

Luu y: TF front/back trong launch hien la placeholder. Phai thay bang eye-to-hand calibration that truoc khi robot execute that.

## 13. Kiem Tra Obstacle Va MoveIt Scene

Ap dung cho ca 1 Kinect va 2 Kinect, vi downstream deu dung `/human_skeleton_base`.

```bash
source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

rostopic echo /human_obstacle_status
rostopic echo /human_collision_object -n 1
rostopic echo /moveit_scene_status
```

Ky vong:

| Topic | Ket qua mong muon |
| --- | --- |
| `/human_obstacle_status` | `OBSTACLE_OK ... primitives=...` khi skeleton hop le; timeout thi `OBSTACLE_TIMEOUT_REMOVE`. |
| `/human_collision_object` | `CollisionObject` id `human_skeleton`, frame `base_link`, operation `ADD` hoac `REMOVE`. |
| `/moveit_scene_status` | `SCENE_OBJECT_APPLIED` va tot nhat co `SCENE_OBJECT_CONFIRMED`. |

## 14. Terminal RViz

```bash
source ~/env_ur3.sh
source ~/catkin_ws/devel/setup.bash

rviz -d ~/catkin_ws/src/rviz/ur3_hrc.rviz
```

Nen them/kiem tra display:

| Display | Topic/Frame |
| --- | --- |
| RobotModel | `base_link` |
| TF | bat |
| MarkerArray | `/hrc_astar_voxel_markers` |
| Image single raw | `/kinect_raw/image_raw` |
| Image single debug | `/kinect_skeleton/image_raw` |
| Image front/back raw | `/kinect_front/kinect_raw/image_raw`, `/kinect_back/kinect_raw/image_raw` |
| Image front/back debug | `/kinect_front/kinect_skeleton/image_raw`, `/kinect_back/kinect_skeleton/image_raw` |
| PlanningScene | MoveIt scene |

## 15. Kiem Tra Planner Plan-Only

Khi `system.launch` hoac `dual_kinect_system.launch` dang chay voi `auto_execute:=false`:

```bash
rostopic echo /ur3_fixed_joint_path_status
rostopic echo /hrc_path_text
rostopic hz /hrc_astar_voxel_markers
rostopic echo /hrc_planning_time_ms
rostopic echo /hrc_astar_planning_time_ms
rostopic echo /hrc_moveit_planning_time_ms
```

Ky vong:

| Status | Y nghia |
| --- | --- |
| `PLAN_ONLY_SUCCESS` | Planner tim duoc plan nhung khong execute. |
| `ASTAR_OK ...` | ARA* guard co duong voxel tu TCP toi target (kem `epsilon`, `expanded_steps`, `time_ms`). |
| `ASTAR_PATH_BLOCKED ...` | ARA* khong tim duoc duong, kem `reason` va `obstacles`. |
| `NO_ASTAR_PATH ...` | Publish tren `/hrc_path_text` khi ARA* fail. |
| `MOVEIT_PLAN_OK ...` | MoveIt plan thanh cong. |
| `TRAJECTORY_SAFE ...` | Trajectory du xa human points. |
| `WAITING_FOR_MOVEIT_SCENE` | Planner dang cho scene manager neu `require_moveit_scene_ready=true`. |
| `WAITING_FOR_HUMAN_SKELETON` | Skeleton bat buoc nhung chua co fresh data. |

## 16. Chay Robot That Voi `auto_execute=true`

Chi chay khi da pass tat ca muc tren va vung lam viec an toan.

Single Kinect front:

```bash
roslaunch ur3_hrc_planner system.launch auto_execute:=true
```

Single Kinect back:

```bash
roslaunch ur3_hrc_planner system_back.launch auto_execute:=true
```

Dual Kinect:

```bash
roslaunch ur3_hrc_planner dual_kinect_system.launch auto_execute:=true
```

Theo doi trong luc chay:

```bash
rostopic echo /ur3_fixed_joint_path_status
rostopic echo /hrc_execution_time_ms
rostopic echo /human_skeleton_status
rostopic echo /human_skeleton_fusion_status
rostopic echo /moveit_scene_status
```

Robot phai dung/khong execute neu:

| Status | Y nghia |
| --- | --- |
| `EXECUTION_STOPPED_HUMAN_TOO_CLOSE` | Human qua gan robot trong luc execute. |
| `EXECUTION_BLOCKED_BY_HUMAN` | Trajectory khong con an toan truoc execute. |
| `HUMAN_BLOCKS_WAYPOINT` | Human gan waypoint target, planner detour/replan. |
| `TRAJECTORY_TOO_CLOSE_TO_HUMAN` | Planned trajectory vi pham clearance. |

## 17. Lenh Kiem Tra Nhanh Khi Loi

Robot:

```bash
echo "PolyscopeVersion" | nc 192.168.100.100 29999
echo "robotmode" | nc 192.168.100.100 29999
echo "programState" | nc 192.168.100.100 29999
echo "running" | nc 192.168.100.100 29999

sudo ss -ltnp | grep -E "11311|50001|50002"
sudo ss -tnp | grep 192.168.100.100
```

ROS graph:

```bash
rosnode list
rostopic list | grep -E "human|hrc|moveit_scene|kinect_skeleton|kinect_front|kinect_back"
rosservice list | grep -E "apply_planning_scene|get_planning_scene|compute_fk"
```

Kinect/perception single:

```bash
rostopic echo /human_skeleton_status
rostopic hz /human_skeleton_base
rqt_image_view /kinect_raw/image_raw
rqt_image_view /kinect_skeleton/image_raw
```

Kinect/perception dual:

```bash
rostopic echo /kinect_front/human_skeleton_status
rostopic echo /kinect_back/human_skeleton_status
rostopic echo /human_skeleton_fusion_status
rostopic hz /human_skeleton_base
rostopic hz /kinect_front/kinect_raw/image_raw
rostopic hz /kinect_back/kinect_raw/image_raw
rqt_image_view /kinect_front/kinect_raw/image_raw
rqt_image_view /kinect_back/kinect_raw/image_raw
```

Mo nhanh 2 cua so anh raw:

```bash
rosrun image_view image_view image:=/kinect_front/kinect_raw/image_raw __name:=front_kinect_raw_view &
rosrun image_view image_view image:=/kinect_back/kinect_raw/image_raw __name:=back_kinect_raw_view &
```

Obstacle/scene:

```bash
rostopic echo /human_obstacle_status
rostopic echo /human_collision_object -n 1
rostopic echo /moveit_scene_status
```

TF:

```bash
rosrun tf tf_echo base_link kinect_front_ir_optical_frame
rosrun tf tf_echo base_link kinect_back_ir_optical_frame
rosrun tf tf_echo base_link shoulder_link
rosrun tf tf_echo base_link tool0
```

Build/static:

```bash
cd ~/catkin_ws
source ~/env_ur3.sh
python3 -m py_compile src/ur3_hrc_planner/scripts/*.py
xmllint --noout src/ur3_hrc_planner/package.xml src/ur3_hrc_planner/launch/*.launch
catkin_make --pkg ur3_hrc_planner
```

## 18. Trang Thai Hay Gap

| Status | Module | Y nghia |
| --- | --- | --- |
| `CAMERA_ERROR ...` | tracker | Khong mo duoc Kinect/pylibfreenect2. |
| `FRAME_TIMEOUT ...` | tracker | Kinect khong tra frame trong timeout. |
| `YOLO_ERROR ...` | tracker | Thieu `ultralytics`, model, hoac YOLO predict loi. |
| `NO_PERSON_MASK ...` | tracker | YOLO khong thay mask nguoi du lon. |
| `BODY_TOO_FEW_CORE_POINTS ...` | tracker | Holistic khong du body landmarks hop le. |
| `ROBOT_FALSE_POSITIVE_REJECTED ...` | tracker | Skeleton bi coi la robot false positive. |
| `TEMPORAL_CONFIRMING ...` | tracker | Skeleton dang doi du frame confirm. |
| `FUSION_OK` | fusion | Ca 2 camera hop le, output da merge. |
| `FUSION_FRONT_ONLY` | fusion | Chi front hop le, fallback front. |
| `FUSION_BACK_ONLY` | fusion | Chi back hop le, fallback back. |
| `FUSION_NO_VALID_INPUT` | fusion | Khong co input fresh. |
| `FUSION_TOO_FEW_JOINTS` | fusion | Output qua it joint hop le. |
| `FUSION_SIDE_SWAP_DETECTED` | fusion | Phat hien/sua nham trai-phai. |
| `FUSION_CONFLICT` | fusion | Front/back lech joint qua nguong, da chon source tot hon. |
| `OBSTACLE_OK ...` | obstacle builder | Da publish collision object human. |
| `OBSTACLE_TIMEOUT_REMOVE ...` | obstacle builder | Mat skeleton, da remove object. |
| `SCENE_OBJECT_APPLIED ...` | scene manager | Da apply object vao MoveIt scene. |
| `SCENE_OBJECT_CONFIRMED ...` | scene manager | Da confirm object co trong PlanningScene. |
| `SCENE_OBJECT_REMOVED ...` | scene manager | Da remove object khoi scene. |
| `ASTAR_OK ...` | planner | ARA* guard co duong voxel (kem epsilon, expanded_steps, time_ms). |
| `ASTAR_PATH_BLOCKED ...` | planner | ARA* khong tim duoc duong, kem reason va so obstacle. |
| `NO_ASTAR_PATH ...` | planner | Publish tren `/hrc_path_text` khi ARA* fail. |
| `MOVEIT_PLAN_OK ...` | planner | MoveIt plan thanh cong. |
| `PLAN_ONLY_SUCCESS` | planner | Plan thanh cong, khong execute. |
| `EXECUTE_SUCCESS` | planner | Execute xong. |
| `ASTAR_EXEC_REPLAN ...` | planner | In-motion A* re-check thay route bi chan o toc do cao; latch padding theo speed, reroute o waypoint ke (khong dung). Chi xuat hien khi `astar_speed_padding_gain>0`. |
