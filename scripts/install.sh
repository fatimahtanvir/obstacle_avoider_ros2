#!/bin/bash
# ============================================================
# Autonomous Obstacle Avoider — One-Command Install Script
# Works on Ubuntu 22.04 or WSL2
# Usage: bash scripts/install.sh
# ============================================================

set -e  # Exit on any error

echo ""
echo "🤖 Autonomous Obstacle Avoider — Setup Script"
echo "=============================================="
echo ""

# ── Step 1: System update ──────────────────────────────────
echo "📦 Step 1/5: Updating system packages..."
sudo apt update -y

# ── Step 2: Install ROS2 Humble if not present ────────────
if ! command -v ros2 &> /dev/null; then
    echo "🌐 Step 2/5: Installing ROS2 Humble..."
    sudo apt install software-properties-common curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list
    sudo apt update
    sudo apt install ros-humble-desktop -y
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble installed!"
else
    echo "✅ Step 2/5: ROS2 already installed — skipping."
    source /opt/ros/humble/setup.bash
fi

# ── Step 3: Install robotics packages ─────────────────────
echo "🔧 Step 3/5: Installing robotics packages..."
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-localization \
    python3-colcon-common-extensions \
    xvfb mesa-utils
echo "✅ Packages installed!"

# ── Step 4: Build workspace ────────────────────────────────
echo "🔨 Step 4/5: Building workspace..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE="$HOME/robot_ws"

mkdir -p "$WORKSPACE/src"
cp -r "$PROJECT_DIR/obstacle_avoider" "$WORKSPACE/src/"

cd "$WORKSPACE"
colcon build --symlink-install
echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc
source "$WORKSPACE/install/setup.bash"
echo "✅ Workspace built!"

# ── Step 5: Verify ────────────────────────────────────────
echo "🔍 Step 5/5: Verifying installation..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

if ros2 pkg list | grep -q obstacle_avoider; then
    echo ""
    echo "=============================================="
    echo "✅ Installation complete! Ready to launch."
    echo "=============================================="
    echo ""
    echo "Run the robot:"
    echo "  ros2 launch obstacle_avoider bringup.launch.py"
    echo ""
    echo "Watch robot state:"
    echo "  ros2 topic echo /robot_state"
    echo ""
    echo "Live velocity:"
    echo "  ros2 topic echo /cmd_vel"
    echo ""
else
    echo "❌ Something went wrong. Check the output above for errors."
    exit 1
fi
