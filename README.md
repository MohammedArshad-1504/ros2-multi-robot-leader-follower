# ROS 2 Multi-Robot Leader–Follower

A ROS 2 multi-robot coordination system where a **leader robot autonomously navigates using Nav2**, while multiple follower robots maintain formation, avoid obstacles, dynamically replan their paths, and recover when they become stuck.

Built and tested with **ROS 2 Jazzy + Gazebo Sim**.

---

## 🚀 Project Overview

This project implements a multi-robot coordination system using a **leader–follower architecture**.

The leader receives navigation goals and performs global navigation using the ROS 2 Navigation2 stack. Followers continuously track their assigned formation positions relative to the leader while independently handling:

- Formation control
- LiDAR-based obstacle detection
- Dynamic path planning
- Obstacle avoidance
- Catch-up behavior
- Formation rejoining
- Stuck detection and recovery

The number of robots and follower spacing can be configured without modifying the core controller.

---

## 🧠 System Architecture

```text
                         Navigation Goal
                               │
                               ▼
                       ┌─────────────┐
                       │    Nav2     │
                       │   Leader    │
                       └──────┬──────┘
                              │
                              ▼
                       ┌─────────────┐
                       │   Leader    │
                       │    Robot    │
                       └──────┬──────┘
                              │
                       TF2 / Leader Pose
                              │
              ┌───────────────┼───────────────┐
              │               │               │
              ▼               ▼               ▼
        ┌──────────┐    ┌──────────┐    ┌──────────┐
        │ Follower │    │ Follower │    │ Follower │
        │    1     │    │    2     │    │    N     │
        └────┬─────┘    └────┬─────┘    └────┬─────┘
             │               │               │
             ▼               ▼               ▼
        LiDAR + TF2     LiDAR + TF2     LiDAR + TF2
             │               │               │
             └───────────────┼───────────────┘
                             ▼
                    Dynamic A* Replanning
                             │
                             ▼
                    Formation Controller
```

---

## ✨ Key Features

### Multi-Robot System

- Dynamically spawn multiple robots
- Configurable number of robots
- Leader + N followers architecture
- Configurable follower spacing
- Independent robot namespaces and TF trees

### Leader Navigation

The leader uses **ROS 2 Nav2** for autonomous navigation.

The leader can:

- Receive navigation goals
- Plan a global path
- Avoid static obstacles
- Navigate through the environment
- Provide its pose as the reference for followers

### Formation Following

Followers maintain a desired position relative to the leader.

The controller continuously calculates:

```text
Desired Follower Position
          ↓
Current Follower Position
          ↓
Position / Heading Error
          ↓
Velocity Command
```

Formation spacing can be adjusted through the launch configuration.

### Dynamic Obstacle Avoidance

Followers use LiDAR data to detect obstacles in their local environment.

Instead of blindly following the leader, each follower considers its local environment and can replan when an obstacle interferes with its path.

### Dynamic A* Path Planning

A grid-based **A\*** planner is used by followers to generate collision-free local paths around detected obstacles.

The general process is:

```text
1. Detect obstacle
       ↓
2. Update local obstacle grid
       ↓
3. Recalculate path
       ↓
4. Continue toward formation position
```

### Formation State Machine

Followers operate through different behavioral states:

```text
FOLLOW
  │
  ├── obstacle / distance increase
  ▼
CATCH_UP
  │
  ├── formation position recovered
  ▼
REJOIN
  │
  └──────────────► FOLLOW


FOLLOW
  │
  └── robot becomes blocked
             ▼
           STUCK
             │
             ▼
         RECOVERY
             │
             ▼
           FOLLOW
```

Implemented states include:

- `FOLLOW`
- `CATCH_UP`
- `REJOIN`
- `HOLD`
- `STUCK`

This allows followers to respond to temporary separation or blockage instead of simply stopping.

### Stuck Recovery

Followers monitor their movement and detect situations where they are unable to make meaningful progress.

When a follower becomes stuck, the controller can transition into recovery behavior before attempting to return to formation.

---

## 🛠️ Technology Stack

| Component | Technology |
|---|---|
| Middleware | ROS 2 Jazzy |
| Simulation | Gazebo Sim |
| Navigation | Nav2 |
| Localization | AMCL |
| Robot Communication | ROS 2 Topics / TF2 |
| Perception | 2D LiDAR |
| Path Planning | Dynamic Grid-based A* |
| Robot Description | URDF / Xacro |
| Visualization | RViz2 |
| Programming | Python / ROS 2 |
| Operating System | Ubuntu 24.04 |

---

## 📁 Project Structure

```text
ros2-multi-robot-leader-follower/
├── src/
│   └── leader_follower/
│       ├── config/
│       │   ├── nav2_params.yaml
│       │   └── slam_mapping.yaml
│       │
│       ├── launch/
│       │   ├── gazebo.launch.py
│       │   ├── single_robot.launch.py
│       │   ├── multi_robot.launch.py
│       │   └── navigation.launch.py
│       │
│       ├── leader_follower/
│       │   ├── follower_controller.py
│       │   ├── formation_manager.py
│       │   ├── leader_safety_controller.py
│       │   └── obstacle_avoidance.py
│       │
│       ├── urdf/
│       │   └── *.xacro
│       │
│       └── worlds/
│           └── two_rooms.sdf
│
├── README.md
└── .gitignore
```

---

# 🏗️ Software Architecture

## Why the Project Uses Multiple Launch Files

The project is divided into multiple launch files because different stages of the robotics system have different responsibilities.

This makes the system easier to **test, debug, and reuse** instead of requiring the complete multi-robot stack to be launched every time.

### `single_robot.launch.py`

Used to test the robot and navigation stack with a single robot.

This provides a simpler environment for validating:

- Robot spawning
- Sensors and LiDAR
- TF tree
- Odometry
- Localization
- Nav2
- Basic autonomous navigation

Single-robot testing makes it possible to isolate navigation problems before introducing multi-robot communication and formation control.

### `gazebo.launch.py`

Responsible for starting the Gazebo simulation environment and the required simulation components.

Keeping simulation startup separate allows the Gazebo environment and robot model to be tested independently from the navigation stack.

### `multi_robot.launch.py`

Responsible for spawning and configuring multiple robots.

This makes it possible to test:

- Robot namespaces
- Independent TF trees
- Robot-specific topics
- Multiple LiDAR streams
- Multi-robot spawning
- Robot-to-robot relationships

### `navigation.launch.py`

This is the main integration launch file.

It combines the required simulation, robot spawning, localization, Nav2, formation controllers, and other components into the complete leader–follower system.

The separation allows individual components to be tested independently while still providing a single launch command for the complete demonstration.

---

# 🧩 Why the Controller Logic Is Split Into Multiple Programs

The project does not place all robot behavior into one large Python node.

Different responsibilities are separated into individual programs so that each part can be developed, tested, and debugged independently.

### `formation_manager.py`

Responsible for determining where each follower should be relative to the leader and maintaining the desired formation spacing.

Conceptually:

```text
Leader Pose
    ↓
Formation Manager
    ↓
Desired Follower Position
```

This separates **formation geometry** from the low-level motion controller.

### `follower_controller.py`

Responsible for converting the follower's desired position and current state into velocity commands.

Conceptually:

```text
Desired Formation Position
          +
Current Follower Position
          ↓
Follower Controller
          ↓
       cmd_vel
```

Separating this logic allows the motion-control behavior to be modified without rewriting the formation-management logic.

### `obstacle_avoidance.py`

Responsible for processing LiDAR information and handling obstacles that may interfere with follower motion.

The follower therefore does not blindly follow the leader. It considers the local environment before generating motion.

```text
LiDAR
  ↓
Obstacle Detection
  ↓
Safe Motion / Replanning
```

### `leader_safety_controller.py`

The leader normally receives velocity commands from Nav2.

The safety controller provides an additional layer between navigation commands and the simulated robot:

```text
Nav2
  ↓
Leader Safety Controller
  ↓
Gazebo
```

This allows safety and obstacle checks to be applied before the final velocity command reaches the robot.

---

# 🎯 Why These Components Are Separate

The architecture follows a basic robotics software principle:

> **Each component should have a clear responsibility.**

Instead of creating one large node responsible for navigation, formation, obstacle avoidance, and recovery, the project separates these behaviors so they can be:

- Tested independently
- Debugged independently
- Modified without affecting unrelated components
- Reused in different configurations
- Extended with additional controllers or planners

This separation also makes the system easier to scale from two robots to larger formations.

---

# 🔄 Development and Testing Strategy

The system was developed incrementally rather than starting with the complete multi-robot stack.

The general development progression was:

```text
Single Robot
     ↓
Robot + Sensors
     ↓
Localization + Nav2
     ↓
Multiple Robots
     ↓
Leader Navigation
     ↓
Follower Formation Control
     ↓
LiDAR Obstacle Handling
     ↓
Dynamic A* Replanning
     ↓
Recovery / Rejoin Behavior
```

This approach allowed individual failures to be isolated before integrating the complete system.

For example, navigation can be tested with `single_robot.launch.py` before introducing follower formation control, while multi-robot spawning can be tested through `multi_robot.launch.py` before adding the complete navigation stack.

---

# ⚙️ Configuration

The system supports configurable multi-robot deployment.

## Number of Robots

The `robot_count` parameter controls the **total number of robots**, including the leader.

For example:

```text
robot_count = 4
```

results in:

```text
1 Leader
3 Followers
```

## Formation Spacing

Follower spacing can be adjusted using:

```text
follower_spacing
```

This allows the same controller to be tested with different formation sizes.

---

# 🗺️ Navigation and Localization

The navigation system uses a previously generated map for autonomous navigation.

The robots use independent namespaces and TF frames so that each robot can maintain its own odometry and robot-specific state while operating in the same environment.

Conceptually:

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

This allows multiple robots to operate simultaneously without mixing their robot-specific topics and transforms.

---

# ▶️ Running the Simulation

## 1. Clone the Repository

```bash
git clone https://github.com/MohammedArshad-1504/ros2-multi-robot-leader-follower.git
cd ros2-multi-robot-leader-follower
```

## 2. Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

## 3. Build

```bash
colcon build
```

## 4. Source the Workspace

```bash
source install/setup.bash
```

## 5. Launch the System

```bash
ros2 launch leader_follower navigation.launch.py
```

The launch system starts the required simulation, robots, localization, navigation and multi-robot control components.

---

# 🤖 Spawn Multiple Robots

The `robot_count` parameter controls the total number of robots.

### 2 Robots

```bash
ros2 launch leader_follower navigation.launch.py robot_count:=2
```

```text
Leader
Follower 1
```

### 4 Robots

```bash
ros2 launch leader_follower navigation.launch.py robot_count:=4
```

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

```text
Leader
Follower 1
Follower 2
Follower 3
Follower 4
Follower 5
```

---

# 📏 Formation Spacing

Formation spacing can be configured at launch.

Example:

```bash
ros2 launch leader_follower navigation.launch.py \
  robot_count:=4 \
  follower_spacing:=1.2
```

This launches one leader and three followers with a configured formation spacing of `1.2` meters.

---

# 🎮 Navigation Workflow

After launching the simulation:

```text
Gazebo
   ↓
Spawn Leader + Followers
   ↓
Localization
   ↓
Nav2
   ↓
Leader receives goal
   ↓
Leader navigates
   ↓
Followers track leader
   ↓
LiDAR detects obstacles
   ↓
Followers replan using A*
   ↓
Formation maintained
```

The leader acts as the primary navigation reference while followers continuously calculate their formation targets and respond to their local environment.

---

# 🔍 Useful ROS 2 Commands

### Check Nodes

```bash
ros2 node list
```

### Check Topics

```bash
ros2 topic list
```

### Check Leader Velocity

```bash
ros2 topic echo /leader/cmd_vel
```

### Check Leader LiDAR

```bash
ros2 topic echo /leader/front_lidar/scan
```

### Check Follower Velocity

```bash
ros2 topic echo /follower_1/cmd_vel
```

### Inspect TF

```bash
ros2 run tf2_tools view_frames
```

---

# 🧪 Testing

The core system was tested in simulation with repeated navigation goals and multiple robots.

Testing focused on:

- Leader autonomous navigation
- Multi-robot spawning
- Formation maintenance
- Dynamic obstacle avoidance
- A* replanning
- Catch-up behavior
- Formation rejoining
- Stuck detection
- Recovery behavior
- TF2 and localization

The system was tested using **ROS 2 Jazzy and Gazebo Sim**.

---

# 📊 Example Behaviors

## Normal Following

```text
Leader ───────────────►

        F1
        F2
        F3
```

Followers continuously adjust their velocity to maintain their assigned formation positions.

## Obstacle Encounter

```text
Leader ─────────►

          █████
      F1  █   █
      F2  █   █
      F3  █████

          ↓
     LiDAR detects
        obstacle
          ↓
       A* replans
          ↓
     Followers avoid
        obstacle
```

## Temporary Separation

```text
FOLLOW
  ↓
distance increases
  ↓
CATCH_UP
  ↓
formation recovered
  ↓
REJOIN
  ↓
FOLLOW
```

---

# 🧠 What This Project Demonstrates

This project was developed to explore practical problems that occur when multiple autonomous robots operate together.

### Robotics

- Differential-drive robot control
- Multi-robot coordination
- Formation control
- Obstacle avoidance
- Recovery behavior

### ROS 2

- Nodes
- Topics
- Parameters
- Launch systems
- TF2
- Namespaces
- Nav2
- AMCL
- RViz2

### Autonomous Navigation

- Global navigation
- Local obstacle handling
- LiDAR processing
- Dynamic replanning
- A* path planning
- Localization

### Software Architecture

The project separates:

```text
Navigation
    +
Perception
    +
Planning
    +
Formation Control
    +
Recovery
```

rather than implementing the entire behavior as a single control loop.

---

# 📌 Project Status

**Core multi-robot leader–follower system: Implemented and tested in simulation.**

The project demonstrates autonomous navigation, multi-robot coordination, formation control, LiDAR-based obstacle handling, dynamic path planning, and recovery behavior using ROS 2.

---

# 🔮 Future Improvements

Planned extensions include:

- More advanced formation geometries
- Improved velocity and trajectory control
- Improved multi-robot collision avoidance
- Communication-aware coordination
- Formation switching
- Real-robot deployment
- Performance benchmarking
- Comparison of different multi-robot planning strategies

---

# 👨‍💻 Author

**Mohammed Arshad S.**

B.E. Mechatronics Engineering  
Robotics | ROS 2 | Autonomous Navigation

GitHub: [MohammedArshad-1504](https://github.com/MohammedArshad-1504)

---

# 📜 License

This project is provided for educational and research purposes.
