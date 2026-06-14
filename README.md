# CSCI5551 / EE5271 — TurtleBot3 Navigation

Final project for CSCI5551 / EE5271. Implements ArUco marker following and LiDAR-based obstacle detection/avoidance on a TurtleBot3 using ROS 2.

---

## Overview

The system is composed of three ROS 2 nodes and one launch file:

| File | Node | Role |
|---|---|---|
| `obstacle_detector.py` | `obstacle_detector` | Reads `/scan`, publishes `Bool` on `/obstacle` |
| `obstacle_avoidance.py` | `obstacle_avoidance` | Subscribes to `/obstacle`, commands `cmd_vel` to avoid |
| `aruco_follower.py` | `aruco_follower_lidar` | Follows an ArUco marker via TF2 + LiDAR distance control |
| `turtlebot3_aruco_tracker.launch.py` | — | Launches the ArUco tracker + camera static TF |

---

## Node Details

### `obstacle_detector.py`

Detects obstacles in the robot's forward LiDAR cone and publishes a binary flag.

- **Subscribes:** `/scan` (`sensor_msgs/LaserScan`)
- **Publishes:** `/obstacle` (`std_msgs/Bool`)
- **Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `distance_threshold` | `0.36` m | Minimum range before obstacle is flagged |
| `front_angle_deg` | `36.0` ° | Half-angle of the forward detection cone (±36°) |

---

### `obstacle_avoidance.py`

Reacts to the `/obstacle` flag by backing up and turning in place at 30 Hz.

- **Subscribes:** `/obstacle` (`std_msgs/Bool`)
- **Publishes:** `cmd_vel` (`geometry_msgs/Twist`)
- **Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `turn_speed` | `-0.6` rad/s | Angular velocity when avoiding (negative = right turn) |

When an obstacle is detected the robot moves at −0.03 m/s linearly while turning.

https://github.com/user-attachments/assets/ea69c3eb-adb6-4360-bffa-69687fe9462c

---

### `aruco_follower.py`

Follows an ArUco marker by reading its pose from the TF tree (published by the ArUco tracker) and using the front LiDAR range for distance control. Runs at 20 Hz.

- **Subscribes:** `/scan` (`sensor_msgs/LaserScan`)
- **Publishes:** `cmd_vel` (`geometry_msgs/Twist`)
- **TF:** looks up `ar_marker_<id>` → `base_link`



https://github.com/user-attachments/assets/f42848a3-3814-481d-a26d-fa46979428c8


  
- **Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `marker_id` | `0` | ID of the ArUco marker to follow |
| `target_distance` | `0.5` m | Desired following distance |
| `min_distance` | `0.2` m | Safety distance — triggers gentle backup |
| `linear_kp` | `0.7` | Proportional gain for linear speed |
| `angular_kp` | `2.0` | Proportional gain for angular speed |
| `max_linear_speed` | `0.2` m/s | Speed clamp |
| `max_angular_speed` | `1.0` rad/s | Angular speed clamp |
| `lookup_timeout_sec` | `0.05` s | TF lookup timeout |
| `max_tf_age_sec` | `0.3` s | Maximum age of marker TF before it is considered stale |
| `front_angle_deg` | `15.0` ° | Half-angle of the LiDAR cone used for distance measurement |

---

### `turtlebot3_aruco_tracker.launch.py`

Launches the ArUco tracker and a static transform for the camera mount.

- **Camera TF:** `base_link` → `camera_rgb_frame` at (x=0.01 m, z=0.10 m)
- **Launch argument:**

| Argument | Default | Description |
|---|---|---|
| `marker_size` | `0.04` m | Physical size of the printed ArUco marker |

---

## Dependencies

- ROS 2 (tested on Humble)
- `turtlebot3_aruco_tracker` package
- `tf2_ros`
- `sensor_msgs`, `geometry_msgs`, `std_msgs`

---

## Running

### 1. Launch the ArUco tracker

```bash
ros2 launch . turtlebot3_aruco_tracker.launch.py marker_size:=0.04
```

### 2. Run obstacle detection

```bash
ros2 run <your_package> obstacle_detector
```

### 3. Run obstacle avoidance

```bash
ros2 run <your_package> obstacle_avoidance
```

### 4. Run ArUco follower

```bash
ros2 run <your_package> aruco_follower
```

> **Note:** `obstacle_avoidance` and `aruco_follower` both publish to `cmd_vel`. Run only one at a time, or implement a mux/priority arbitration layer before deployment.

---

## Authors

Azamat Turganbayev
