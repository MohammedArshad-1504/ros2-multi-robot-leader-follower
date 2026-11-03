
`README.md` Leader-Follower 

````markdown
# Multi-Robot Leader–Follower Navigation — ROS 2

ROS 2 Jazzy + Gazebo Sim project for multi-robot autonomous navigation.

The **leader** receives navigation goals through Nav2. **Followers** maintain formation behind the leader and use TF2, LiDAR, dynamic obstacle handling, dynamic A* planning, and recovery/rejoin behavior.

## Stack

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim
- RViz2
- Nav2
- AMCL
- TF2
- LiDAR
- Dynamic A*
- Python / ROS 2
- ros_gz_bridge

## Build

```bash
cd ~/leader_follower_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select leader_follower

source install/setup.bash
````

## Launch

Main multi-robot launch:

```bash
ros2 launch leader_follower navigation.launch.py
```

## Spawn N Robots

`robot_count` means the **total number of robots, including the leader**.

### 2 Robots

```bash
ros2 launch leader_follower navigation.launch.py robot_count:=2
```

Spawns:

```text
Leader
Follower 1
```

### 4 Robots

```bash
ros2 launch leader_follower navigation.launch.py robot_count:=4
```

Spawns:

```text
Leader
Follower 1
Follower 2
Follower 3
```

### 6 Robots

```bash
ros2 launch leader_follower navigation.launch.py robot_count:=6
```

Spawns:

```text
Leader
Follower 1
Follower 2
Follower 3
Follower 4
Follower 5
```

## Formation Spacing

Formation spacing can be configured using `follower_spacing`.

Example:

```bash
ros2 launch leader_follower navigation.launch.py \
  robot_count:=4 \
  follower_spacing:=1.2
```

## Short Architecture

### Leader

The leader uses Nav2 for goal-based navigation.

```text
RViz Goal
   ↓
Nav2
   ↓
Leader cmd_vel
   ↓
Leader Safety Controller
   ↓
Gazebo
```

AMCL provides localization on the saved map.

### Followers

Followers use a custom controller for formation following.

```text
Formation Target + TF2
          +
        LiDAR
          ↓
Dynamic Obstacle Grid
          ↓
A* Path Planning
          ↓
Follower Controller
          ↓
       cmd_vel
```

Followers can:

* Maintain formation spacing
* Catch up when separated
* Rejoin their formation position
* Detect dynamic obstacles using LiDAR
* Replan when paths become blocked
* Detect stuck conditions
* Perform recovery behavior

## Saved Map

The navigation system uses the previously generated map:

```text
maps/
├── two_rooms_map.pgm
└── two_rooms_map.yaml
```

SLAM is not required during normal multi-robot navigation.

## TF / Localization

Each robot has its own odometry and base frames.

Example:

```text
map
├── leader/odom
│   └── leader_base_link
│
├── follower_1/odom
│   └── follower_1_base_link
│
├── follower_2/odom
│   └── follower_2_base_link
│
└── ...
```

This allows the robots to maintain independent localization.

## Basic Test

Launch the system:

```bash
ros2 launch leader_follower navigation.launch.py \
  robot_count:=4 \
  follower_spacing:=1.2
```

Then:

1. Wait for Gazebo, RViz, AMCL and Nav2 to become active.
2. Set the leader's initial pose in RViz when required.
3. Use **Nav2 Goal / Goal Pose** in RViz.
4. Send a goal to the leader.
5. Observe the followers maintaining formation and responding to obstacles.

## Useful Commands

Check nodes:

```bash
ros2 node list
```

Check topics:

```bash
ros2 topic list
```

Check leader velocity:

```bash
ros2 topic echo /leader/cmd_vel
```

Check leader LiDAR:

```bash
ros2 topic echo /leader/front_lidar/scan
```

Check follower velocity:

```bash
ros2 topic echo /follower_1/cmd_vel
```

View TF:

```bash
ros2 run tf2_tools view_frames
```

## Project Structure

```text
leader_follower_ws/
├── maps/
│   ├── two_rooms_map.pgm
│   └── two_rooms_map.yaml
│
└── src/
    └── leader_follower/
        ├── config/
        │   ├── nav2_params.yaml
        │   └── slam_mapping.yaml
        │
        ├── launch/
        │   ├── gazebo.launch.py
        │   ├── multi_robot.launch.py
        │   ├── navigation.launch.py
        │   └── single_robot.launch.py
        │
        ├── leader_follower/
        │   ├── follower_controller.py
        │   ├── formation_manager.py
        │   ├── leader_safety_controller.py
        │   └── obstacle_avoidance.py
        │
        ├── urdf/
        │   └── *.xacro
        │
        └── worlds/
            └── two_rooms.sdf
```

## Project Status

Core multi-robot functionality was implemented and tested with repeated navigation goals.

The system demonstrated:

* Leader Nav2 navigation
* Dynamic spawning of multiple robots
* Independent follower localization
* Formation following
* LiDAR-based dynamic obstacle handling
* Dynamic A* replanning
* Catch-up / rejoin behavior
* Stuck recovery

Known simulation and host-network edge cases may still occur.

```

This version is intentionally **README-sized** rather than turning the GitHub README into the full technical documentation.
```

