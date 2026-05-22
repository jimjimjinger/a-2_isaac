# Isaac Sim ROS2 Topic Summary

Date: 2026-05-22

## Deliverable Files

Saved scene USD with Action Graphs:

```text
/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/rover_v2/rover_m0609_localization.usd
```

Standalone copy of the joint state splitter node:

```text
/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/rover_v2/joint_state_splitter.py
```

## Topic Table

| Data | Topic | Message Type | Source | Consumer / Purpose | Status |
|---|---|---|---|---|---|
| Rover command | `/cmd_vel` | `geometry_msgs/msg/Twist` | External ROS2 publisher | Rover drive Action Graph | OK |
| IMU | `/imu/data` | `sensor_msgs/msg/Imu` | `/World/Vehicle/rover/.../Body/Imu_Sensor` | Localization / EKF | OK, about 102 Hz |
| Rover camera RGB | `/camera/rover/image_raw` | `sensor_msgs/msg/Image` | Rover body camera | Vision / terrain / mineral detection | OK, about 60 Hz |
| Rover camera depth | `/camera/rover/depth` | `sensor_msgs/msg/Image` | Rover body camera | Vision / obstacle response | OK, about 55-68 Hz |
| Rover camera info | `/camera/rover/camera_info` | `sensor_msgs/msg/CameraInfo` | Rover body camera render product | Deprojection / camera model | OK, about 100 Hz |
| Wrist camera RGB | `/camera/wrist/image_raw` | `sensor_msgs/msg/Image` | D455 `Camera_OmniVision_OV9782_Color` | Manipulation vision | OK, about 26-29 Hz |
| Wrist camera depth | `/camera/wrist/depth` | `sensor_msgs/msg/Image` | D455 `Camera_Pseudo_Depth` | Manipulation depth | OK, about 25-31 Hz |
| Wrist camera info | `/camera/wrist/camera_info` | `sensor_msgs/msg/CameraInfo` | D455 depth render product | Wrist depth deprojection | OK, about 59 Hz |
| Raw joint states | `/joint_states_raw` | `sensor_msgs/msg/JointState` | Isaac articulation publisher | Raw full robot state | OK |
| Rover wheel states | `/rover/wheel_states` | `sensor_msgs/msg/JointState` | `joint_state_splitter_node` | Future wheel odometry / localization | OK, about 103 Hz |
| Arm joint states | `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_splitter_node` | M0609 + gripper state | OK, about 101 Hz |

## Intentionally Excluded

| Topic | Reason |
|---|---|
| `/clock` | Not needed for the current workflow. |
| `/odom` | Isaac-generated odometry is not suitable as the real-world interface target. Wheel odometry will be derived later from `/rover/wheel_states`. |
| `/tf`, `/tf_static` | Not required for the current milestone. Removed from the immediate scope. |

## Action Graphs

The current scene creates three main Action Graphs.

### `/ActionGraph/LocalizationSensors`

Purpose:

```text
Publish sensor data from Isaac Sim to ROS2.
```

Published topics:

```text
/imu/data
/camera/rover/image_raw
/camera/rover/depth
/camera/rover/camera_info
/camera/wrist/image_raw
/camera/wrist/depth
/camera/wrist/camera_info
```

This graph contains the IMU reader and ROS2 camera helper nodes.

### `/ActionGraph/RoverStatePublishers`

Purpose:

```text
Publish the full articulation joint state from Isaac Sim.
```

Published topic:

```text
/joint_states_raw
```

This is intentionally kept as the raw full robot state. It includes rover wheel joints, rover steering joints, suspension joints, M0609 joints, and gripper joints.

Filtering is done outside Isaac Sim by `joint_state_splitter_node`.

### `/ActionGraph/RoverAckermannDrive`

Purpose:

```text
Receive /cmd_vel and convert it into rover steering and wheel velocity commands.
```

Subscribed topic:

```text
/cmd_vel
```

Behavior:

```text
linear.x  -> forward/backward wheel velocity
angular.z -> steering direction and turning radius
```

The graph computes Ackermann-style steering angles and wheel velocities, then sends:

```text
steer joint position commands
drive wheel velocity commands
```

This is the graph that actually moves the rover from ROS2 command velocity input.

## Data Flow

Command flow:

```text
ROS2 /cmd_vel
  -> /ActionGraph/RoverAckermannDrive
  -> rover steer joints + drive wheel joints
```

Sensor flow:

```text
Isaac IMU + cameras
  -> /ActionGraph/LocalizationSensors
  -> /imu/data
  -> /camera/rover/...
  -> /camera/wrist/...
```

Joint state flow:

```text
Isaac articulation
  -> /ActionGraph/RoverStatePublishers
  -> /joint_states_raw
  -> joint_state_splitter_node
  -> /rover/wheel_states
  -> /joint_states
```

## Joint State Splitter

The node that splits Isaac's full joint state is here:

```text
/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_localization/isaac_localization/sensors/joint_state_splitter.py
```

It is registered in:

```text
/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_localization/setup.py
```

Run command:

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/dev_ws/rover_ws/install/setup.bash
ros2 run isaac_localization joint_state_splitter_node
```

Input:

```text
/joint_states_raw
```

Outputs:

```text
/rover/wheel_states
/joint_states
```

## Split Rules

`/rover/wheel_states` contains:

```text
FL_Drive_Continuous
FR_Drive_Continuous
CL_Drive_Continuous
CR_Drive_Continuous
RL_Drive_Continuous
RR_Drive_Continuous
FL_Steer_Revolute
FR_Steer_Revolute
RL_Steer_Revolute
RR_Steer_Revolute
```

`/joint_states` contains:

```text
joint_1
joint_2
joint_3
joint_4
joint_5
joint_6
finger_joint
left_inner_knuckle_joint
left_outer_knuckle_joint
right_inner_knuckle_joint
right_inner_finger_joint
left_inner_finger_joint
```

## Main Isaac Scene Script

The Isaac Sim scene and Action Graph setup is here:

```text
/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/build_rover_m0609_scene.py
```

Main run command:

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/dev_ws/rover_ws/install/setup.bash
cd /home/rokey/dev_ws/rover_ws/src/a2_isaac
isaac-python isaac_manipulation/scripts/build_rover_m0609_scene.py --no-drive --auto-play
```
