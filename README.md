# grasp_pose_estimator

> Lightweight grasp pose estimator for cobot arms — bounding box + depth image → reliable 3D grasp pose with quality score. No training required.

Built with **ROS2 Jazzy** + **Gazebo Harmonic** on a 6-DOF cobot arm with a fixed eye-to-hand RGBD camera.

---

## What it does

Takes a depth image and a 2D bounding box from YOLOv8 and outputs a reliable 3D grasp pose for any robot arm — handling depth noise, surface orientation, and grasp quality scoring. The output is a `PoseStamped` in `base_link` frame, ready for any motion planner.

```
RGB image    →  yolo_detector    →  /detected_object/bounding_box
Depth image  →  depth_processor  →  /grasp_pose   (PoseStamped, base_link frame)
Camera info  →                   →  /grasp_score  (Float32, 0.0 – 1.0)
                                 →  /grasp_debug  (Image, colourised depth)
```

**Example output at 10Hz:**
```
[depth_processor]: Grasp pose in base_link: (0.532, 0.015, 0.169)m  score=1.00
```

---

## Why this exists

No lightweight ROS2 package exists that does **all** of:

- Bounding box + depth → 3D grasp pose
- Depth noise filtering at object edges (bilateral filter)
- Surface normal estimation for approach orientation (PCA)
- Grasp quality score before motion attempt
- No MoveIt dependency — works with any arm or planner
- No custom training — uses YOLOv8 pretrained on COCO (80 classes)

Heavy alternatives (GraspIt, GPD) require training data and GPU inference. This runs on CPU and works out of the box.

---

## System

| Component | Version |
|-----------|---------|
| ROS2 | Jazzy |
| Gazebo | Harmonic |
| Python | 3.12 |
| YOLOv8 | ultralytics 8.x |
| Camera | D435-style RGBD (simulated, eye-to-hand) |
| Arm | 6-DOF cobot |


---

## Pipeline — step by step

```
Step 1   ROI extraction       crop depth image to bounding box region
Step 2   Bilateral filter     remove depth noise, preserve object edges
Step 3   3D centroid          deproject filtered depth using camera intrinsics
Step 4   Surface normal       PCA on point cloud → orientation of object surface
Step 5   Grasp orientation    align gripper Z with surface normal → quaternion
Step 6   Grasp score          ratio of valid depth pixels in ROI (0.0 – 1.0)
Step 7   TF transform         convert pose from camera frame to base_link
```

**Why bilateral filter and not mean depth:**
Mean depth inside a bounding box is skewed by mixed pixels at object edges — pixels that simultaneously see part object and part background. Bilateral filter preserves depth discontinuities while smoothing within-object noise, giving a stable and accurate centroid.

**Why PCA for surface normal:**
The third principal component of the point cloud (smallest eigenvalue direction) is perpendicular to the dominant surface. This gives a grasp approach direction that adapts to object orientation — a tilted object gets a tilted approach, not always top-down.

**Why eye-to-hand camera:**
Camera mounted on a fixed pole, not on the end effector. The arm never occludes the camera view during motion. TF2 handles the coordinate transform from camera frame to base_link automatically via the static transform chain.

---

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/camera_head/image` | `sensor_msgs/Image` | RGB image for YOLO detection |
| `/camera_head/depth_image` | `sensor_msgs/Image` | Depth image in metres (32FC1) |
| `/camera_head/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics read at runtime |
| `/detected_object/bounding_box` | `vision_msgs/Detection2DArray` | YOLO bounding boxes |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/detected_object/bounding_box` | `vision_msgs/Detection2DArray` | Detected objects with class and confidence |
| `/detected_object/debug_image` | `sensor_msgs/Image` | RGB with bounding boxes drawn |
| `/grasp_pose` | `geometry_msgs/PoseStamped` | 3D grasp pose in `base_link` frame |
| `/grasp_score` | `std_msgs/Float32` | Grasp quality score 0.0 – 1.0 |
| `/grasp_debug` | `sensor_msgs/Image` | Colourised depth with centroid marker |

---

## Installation

```bash
# clone into your workspace
cd ~/your_ws/src
git clone https://github.com/bineeshajabi/grasp_pose_estimator.git

# install Python dependencies
pip install ultralytics --break-system-packages

# install ROS2 dependencies
sudo apt install ros-jazzy-tf2-geometry-msgs

# build
cd ~/your_ws
colcon build --packages-select grasp_pose_estimator
source install/setup.bash
```

> **Note:** If you see a NumPy version conflict with cv_bridge, run:
> `pip install "numpy<2" --break-system-packages`

---

## Usage

```bash
# launch full pipeline (both nodes)
ros2 launch grasp_pose_estimator grasp_estimator.launch.py

# with custom confidence threshold
ros2 launch grasp_pose_estimator grasp_estimator.launch.py confidence:=0.3

# with a larger model for better accuracy
ros2 launch grasp_pose_estimator grasp_estimator.launch.py model:=yolov8s.pt

# verify grasp pose output
ros2 topic echo /grasp_pose --once

# check detection rate
ros2 topic hz /detected_object/bounding_box
```

---

## Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `confidence` | `0.5` | YOLO detection confidence threshold |
| `model` | `yolov8n.pt` | YOLOv8 model weights |
| `gripper_width` | `0.08` | Gripper width in metres for scoring |

---

## Grasp score interpretation

| Score | Meaning | Recommended action |
|-------|---------|-------------------|
| 0.8 – 1.0 | Excellent — full depth coverage | Execute grasp |
| 0.5 – 0.8 | Good — minor occlusion | Execute grasp |
| 0.4 – 0.5 | Borderline | Reposition recommended |
| < 0.4 | Poor — too much missing depth | Do not attempt grasp |


## TF transform chain

```
camera_head_depth_optical_frame
        ↓
camera_head_depth_frame
        ↓
camera_head_link
        ↓  (0, 0.01, 0.50) + 25° tilt + 90° rotation
torso_link
        ↓  (0.22, -0.30, 0)
base_link
```

TF2 walks this chain automatically. The static transforms are defined in the robot URDF and broadcast by `robot_state_publisher`.

---

## MoveIt2 integration

`grasp_pose_estimator` has no MoveIt dependency. It outputs a standard `PoseStamped` in `base_link` frame which any motion planner can consume directly.

An example MoveIt2 integration is provided in `examples/arm_task_node.py`:

- Subscribes to `/grasp_pose` and `/grasp_score`
- Gates on score threshold before attempting motion
- Executes pre-grasp → grasp → gripper close sequence
- Uses `executing` flag to prevent overlapping motion commands

```python
# the estimator output feeds directly into MoveIt
arm.set_goal_state(pose_stamped_msg=grasp_pose_in_base, pose_link='link_flange')
plan = arm.plan()
moveit.execute(plan.trajectory)
```

---

## Transferability to mobile robots

The `yolo_detector` node is completely decoupled from the robot type. On a mobile robot:

- **Person following** — detection pixel offset → angular velocity → `/cmd_vel`
- **Object navigation** — detection + depth → 3D position → Nav2 goal → robot navigates to object
- **Aerial camera annotation** — project robot pose into image coordinates using the same intrinsics formula

The detector node requires zero changes. Only the subscriber logic changes.

---

## Dependencies

```
rclpy                   sensor_msgs
vision_msgs             cv_bridge
geometry_msgs           std_msgs
tf2_ros                 tf2_geometry_msgs
ultralytics (pip)       opencv-python
numpy < 2.0
```

---

## Known limitations

- YOLOv8n (nano) detection range drops beyond ~1m for small objects on CPU
- Plain untextured Gazebo primitives may not reach 0.5 confidence — use textured models
- Depth noise increases at object edges even after bilateral filtering — score threshold of 0.4 handles this

---

## Author

**Bineesha Jabi**

[github.com/bineeshajabi](https://github.com/bineeshajabi)
