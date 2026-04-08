# 🤖 Autonomous Obstacle-Avoiding Robot using ROS2 + Gazebo

A fully autonomous differential-drive robot that navigates and avoids obstacles in real time using a **Vector Field Histogram (VFH)** algorithm, built on **ROS2 Humble** and simulated in **Gazebo Classic**. Features a state machine brain, live SLAM mapping, and an RViz2 visualization dashboard.

> Built as a spring break robotics project. Every component mirrors real-world autonomous robot stacks used in research and industry.

---

## 📸 What it looks like

```
Terminal 1: Robot navigating + state machine logging
[INFO] State: EXPLORING | Stuck counter: 0
[INFO] State: AVOIDING  | Stuck counter: 0
[INFO] State: EXPLORING | Stuck counter: 0

Terminal 2: Live velocity stream
linear:  x: 0.3
angular: z: 0.0

Terminal 3: SLAM building a map in RViz2
```

---

## 🧠 Architecture

```
┌─────────────────────────────────────────┐
│           Gazebo Simulation              │
│  Physics · Diff Drive · LIDAR Model     │
└────────┬──────────────┬─────────────────┘
         │              │
    /scan topic     /odom topic
         │              │
┌────────▼──────────────▼─────────────────┐
│         Obstacle Avoidance Node          │
│   VFH Algorithm + State Machine          │
│   States: EXPLORING / AVOIDING /         │
│           RECOVERING                     │
└────────────────┬────────────────────────┘
                 │
           /cmd_vel topic
                 │
┌────────────────▼────────────────────────┐
│         Robot Actuators                  │
│         (Differential Drive)             │
└─────────────────────────────────────────┘
         │              │
    SLAM Toolbox     RViz2
    (live mapping)   (visualization)
```

---

## ✨ Features

- **360° LIDAR** 360 laser beams per scan, 8m range, Gaussian noise model
- **VFH obstacle avoidance** polar histogram danger mapping, finds widest open valley
- **3-state machine** `EXPLORING` → `AVOIDING` → `RECOVERING` with stuck detection
- **SLAM mapping** builds a live occupancy grid map using SLAM Toolbox
- **RViz2 dashboard** visualize robot model, laser scan, and live map
- **Fully tunable** all parameters adjustable via CLI without restarting
- **Bag recording ready** record and replay any run for analysis

---

## 🗂️ Project Structure

```
obstacle_avoider_ros2/
├── obstacle_avoider/
│   ├── obstacle_avoider/
│   │   ├── __init__.py
│   │   └── avoider_node.py       # VFH + state machine brain
│   ├── launch/
│   │   └── bringup.launch.py     # Full system launch
│   ├── urdf/
│   │   └── robot.urdf            # Robot physical description
│   ├── worlds/
│   │   └── obstacles.world       # Gazebo obstacle course
│   ├── config/
│   │   └── slam.yaml             # SLAM Toolbox config
│   ├── rviz/
│   │   └── robot.rviz            # RViz2 config
│   ├── package.xml
│   └── setup.py
├── scripts/
│   └── install.sh                # One-command setup script
├── docs/
│   └── algorithm.md              # VFH algorithm explained
└── README.md
```

---

## ⚡ Quick Start

### Prerequisites
- Ubuntu 22.04 (or WSL2 on Windows)
- ROS2 Humble
- Gazebo Classic 11

### 1. Install ROS2 Humble
```bash
sudo apt install software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install ros-humble-desktop -y
```

### 2. Install dependencies
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions -y
```

### 3. Clone and build
```bash
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone https://github.com/YOUR_USERNAME/obstacle_avoider_ros2.git
cp -r obstacle_avoider_ros2/obstacle_avoider .
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch
```bash
ros2 launch obstacle_avoider bringup.launch.py
```

---

## 🎮 Usage

### Watch the robot's state in real time
```bash
ros2 topic echo /robot_state
```

### Stream live velocity commands
```bash
ros2 topic echo /cmd_vel
```

### Open RViz2 visualization
```bash
rviz2
# Add displays: LaserScan (/scan), Map (/map), RobotModel
```

### Tune parameters on the fly
```bash
ros2 param set /obstacle_avoider safe_distance 0.8
ros2 param set /obstacle_avoider linear_speed 0.5
ros2 param set /obstacle_avoider angular_speed 1.2
```

### Record a run
```bash
ros2 bag record /scan /cmd_vel /odom /robot_state -o my_run
```

### Replay a run
```bash
ros2 bag play my_run
```

### Drive manually
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🔬 The Algorithm: Vector Field Histogram (VFH)

VFH works in three steps:

1. **Build a polar histogram** divide the 360° scan into 36 bins of 10° each. For every laser reading closer than the warning distance, increase the danger value in that bin.

2. **Find open valleys** threshold the histogram to find consecutive bins below the danger threshold. These are the safe directions to travel.

3. **Steer toward the best valley** pick the open valley closest to the forward direction and compute the angular correction needed to steer into it.

The state machine adds:
- **EXPLORING** path is clear, drive forward at full speed
- **AVOIDING**  obstacle detected, slow down and steer toward best valley
- **RECOVERING** completely stuck, reverse and spin to escape

See [`docs/algorithm.md`](docs/algorithm.md) for the full mathematical breakdown.

---

## 📊 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 360° LIDAR readings |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to wheels |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/robot_state` | `std_msgs/String` | Current state machine state |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM-built map |
| `/robot_description` | `std_msgs/String` | URDF robot model |

---

## ⚙️ Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_speed` | `0.3` | Forward speed (m/s) |
| `angular_speed` | `0.8` | Rotation speed (rad/s) |
| `safe_distance` | `0.5` | Emergency stop distance (m) |
| `warning_distance` | `1.2` | Obstacle detection range (m) |
| `recovery_time` | `2.0` | Duration of recovery maneuver (s) |

---

## 🚀 Possible Extensions

- [ ] Add Nav2 for goal-based navigation
- [ ] Integrate a depth camera for 3D obstacle detection
- [ ] Add an IMU + EKF for better localization
- [ ] Deploy on a real TurtleBot3 or Raspberry Pi robot
- [ ] Add a web dashboard using rosbridge + React
- [ ] Implement frontier exploration for autonomous mapping

---

## 🛠️ Tech Stack

| Tool | Purpose |
|------|---------|
| ROS2 Humble | Robot middleware and communication |
| Gazebo Classic 11 | Physics simulation |
| SLAM Toolbox | Real-time mapping |
| Python 3 + NumPy | Algorithm implementation |
| RViz2 | Visualization |
| URDF | Robot physical model |

---

## 📄 License

MIT License free to use, modify, and build on.

---

## 👩‍💻 Author

Built by Fatima — spring break 2026.
