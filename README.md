# ROS 2 Multi-Robot Leader–Follower

A ROS 2 multi-robot coordination system where a **leader robot autonomously navigates using Nav2**, while multiple follower robots maintain formation, avoid obstacles, dynamically replan their paths, and recover when they become stuck.

Built and tested with **ROS 2 Jazzy + Gazebo Sim**.

---

## 🚀 Project Overview

This project explores multi-robot coordination using a leader–follower architecture.

The leader receives navigation goals and is responsible for global navigation using the ROS 2 Navigation2 stack. Followers continuously track their assigned formation positions relative to the leader while independently handling:

- Formation control
- LiDAR-based obstacle detection
- Dynamic path planning
- Obstacle avoidance
- Catch-up behavior
- Formation rejoining
- Stuck detection and recovery

The number of robots and formation spacing can be configured without modifying the core controller.

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
                         │   Robot     │
                         └──────┬──────┘
                                │
                         TF2 / Leader Pose
                                │
              ┌─────────────────┼─────────────────┐
              │                 │                 │
              ▼                 ▼                 ▼
        ┌──────────┐      ┌──────────┐      ┌──────────┐
        │ Follower │      │ Follower │      │ Follower │
        │    1     │      │    2     │      │    N     │
        └────┬─────┘      └────┬─────┘      └────┬─────┘
             │                 │                 │
             ▼                 ▼                 ▼
        LiDAR + TF2       LiDAR + TF2       LiDAR + TF2
             │                 │                 │
             └─────────────────┼─────────────────┘
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

The formation can be adjusted using the configurable follower spacing parameter.

### Dynamic Obstacle Avoidance

Followers use LiDAR data to detect obstacles in their local environment.

Instead of blindly following the leader's path, each follower builds a local obstacle representation and replans when necessary.

### Dynamic A* Path Planning

A grid-based **A\*** planner is used by followers to generate collision-free local paths around detected obstacles.

This allows a follower to:

1. Detect an obstacle
2. Update its local obstacle grid
3. Recalculate a path
4. Continue toward its formation position

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

This prevents followers from simply stopping when they temporarily lose formation.

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
│       ├── launch/
│       ├── maps/
│       ├── params/
│       ├── rviz/
│       ├── urdf/
│       ├── worlds/
│       ├── leader_controller.py
│       ├── follower_controller.py
│       ├── ...
│       ├── package.xml
│       └── setup.py
│
├── README.md
└── .gitignore
```

---

## ⚙️ Configuration

The system supports configurable multi-robot deployment.

### Number of Robots

The `robot_count` parameter controls the total number of robots, including the leader.

For example:

```text
robot_count = 4
```

results in:

```text
1 Leader
3 Followers
```

### Formation Spacing

Follower spacing can be adjusted using:

```text
follower_spacing
```

This allows the same controller to be tested with different formation sizes.

---

## ▶️ Running the Simulation

### 1. Clone the repository

```bash
git clone https://github.com/MohammedArshad-1504/ros2-multi-robot-leader-follower.git
cd ros2-multi-robot-leader-follower
```

### 2. Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

### 3. Build

```bash
colcon build
```

### 4. Source the workspace

```bash
source install/setup.bash
```

### 5. Launch the system

```bash
ros2 launch leader_follower navigation.launch.py
```

The launch system starts the simulation, robots, navigation stack, localization, RViz and multi-robot control system.

---

## 🎮 Navigation Workflow

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

---

## 🔍 What This Project Demonstrates

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
- Local obstacle avoidance
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

## 🧪 Testing

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

The system was tested using ROS 2 Jazzy and Gazebo Sim.

---

## 📊 Example Behavior

### Normal Following

```text
Leader ───────────────►

        F1
        F2
        F3
```

Followers continuously adjust their velocity to maintain their formation positions.

### Obstacle Encounter

```text
Leader ─────────►

          █████
     F1   █   █
     F2   █   █
     F3   █████

          ↓
      LiDAR detects
        obstacle
          ↓
       A* replans
          ↓
    Followers avoid
       obstacle
```

### Temporary Separation

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

## 🎯 Future Improvements

Planned extensions include:

- More advanced formation geometries
- Better velocity/trajectory control
- Improved multi-robot collision avoidance
- Communication-aware coordination
- Formation switching
- Real-robot deployment
- Performance benchmarking
- Comparison of different multi-robot planning strategies

---

## 📌 Project Status

**Core multi-robot leader–follower system: Implemented and tested in simulation.**

The project is intended as a robotics software engineering project demonstrating autonomous navigation, multi-robot coordination, perception-driven planning, and recovery behavior using ROS 2.

---

## 👨‍💻 Author

**Mohammed Arshad S.**

B.E. Mechatronics Engineering  
Robotics / ROS 2 / Autonomous Navigation

GitHub: [MohammedArshad-1504](https://github.com/MohammedArshad-1504)

---

## 📜 License

This project is provided for educational and research purposes.
