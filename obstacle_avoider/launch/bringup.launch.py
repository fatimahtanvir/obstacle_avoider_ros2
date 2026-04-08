import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("obstacle_avoider")
    urdf_file = os.path.join(pkg, "urdf", "robot.urdf")
    world_file = os.path.join(pkg, "worlds", "obstacles.world")
    slam_config = os.path.join(pkg, "config", "slam.yaml")

    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([

        # Gazebo physics server (headless — no GUI required)
        ExecuteProcess(
            cmd=["gzserver", "--verbose", world_file,
                 "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so"],
            output="screen"
        ),

        # Publish robot URDF to /robot_description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}]
        ),

        # Spawn robot into Gazebo world
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", "robot_description",
                "-entity", "avoider",
                "-x", "0.0", "-y", "0.0", "-z", "0.1"
            ],
            output="screen"
        ),

        # VFH obstacle avoidance brain
        Node(
            package="obstacle_avoider",
            executable="avoider_node",
            name="obstacle_avoider",
            output="screen",
            parameters=[{
                "linear_speed": 0.3,
                "angular_speed": 0.8,
                "safe_distance": 0.5,
                "warning_distance": 1.2,
                "recovery_time": 2.0,
                "vfh_threshold": 0.3,
                "n_histogram_bins": 36,
            }]
        ),

        # SLAM Toolbox — builds live occupancy grid map
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config],
        ),
    ])
