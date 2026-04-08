#!/usr/bin/env python3
"""
Autonomous Obstacle Avoidance Node
===================================
Algorithm : Vector Field Histogram (VFH)
States    : EXPLORING → AVOIDING → RECOVERING
Inputs    : /scan  (sensor_msgs/LaserScan)
Outputs   : /cmd_vel (geometry_msgs/Twist)
            /robot_state (std_msgs/String)

Author    : Fatima
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from enum import Enum


class State(Enum):
    EXPLORING = "EXPLORING"
    AVOIDING = "AVOIDING"
    RECOVERING = "RECOVERING"


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__("obstacle_avoider")

        # --- Declare tunable parameters ---
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.8)
        self.declare_parameter("safe_distance", 0.5)
        self.declare_parameter("warning_distance", 1.2)
        self.declare_parameter("recovery_time", 2.0)
        self.declare_parameter("vfh_threshold", 0.3)
        self.declare_parameter("n_histogram_bins", 36)

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.safe_dist = self.get_parameter("safe_distance").value
        self.warn_dist = self.get_parameter("warning_distance").value
        self.recovery_time = self.get_parameter("recovery_time").value
        self.vfh_threshold = self.get_parameter("vfh_threshold").value
        self.n_bins = self.get_parameter("n_histogram_bins").value

        # --- State machine ---
        self.state = State.EXPLORING
        self.recovery_start = None
        self.stuck_counter = 0
        self.last_twist = Twist()

        # --- ROS2 interfaces ---
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "/robot_state", 10)

        # --- Diagnostics timer ---
        self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info("=" * 45)
        self.get_logger().info("  Obstacle Avoider with State Machine online!")
        self.get_logger().info(f"  Safe dist   : {self.safe_dist}m")
        self.get_logger().info(f"  Warn dist   : {self.warn_dist}m")
        self.get_logger().info(f"  Linear speed: {self.linear_speed} m/s")
        self.get_logger().info(f"  VFH bins    : {self.n_bins}")
        self.get_logger().info("=" * 45)

    # ------------------------------------------------------------------
    def publish_diagnostics(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)
        self.get_logger().info(
            f"State: {self.state.value:12s} | "
            f"Stuck: {self.stuck_counter} | "
            f"v={self.last_twist.linear.x:.2f} m/s  "
            f"w={self.last_twist.angular.z:.2f} rad/s"
        )

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)

        # Sector minimums
        third = len(ranges) // 3
        min_front = float(np.min(ranges[third: 2 * third]))
        min_left = float(np.min(ranges[2 * third:]))
        min_right = float(np.min(ranges[:third]))
        min_all = float(np.min(ranges))

        # Build VFH polar histogram
        hist = self._build_histogram(ranges, angles)
        open_bins = hist < self.vfh_threshold
        forward_bin = self.n_bins // 2
        best_bin = self._best_valley(open_bins, forward_bin)

        twist = Twist()
        now = self.get_clock().now().nanoseconds / 1e9

        # ── State machine ──────────────────────────────────────────────
        if self.state == State.RECOVERING:
            elapsed = now - self.recovery_start
            if elapsed < self.recovery_time:
                # Reverse and spin to escape
                twist.linear.x = -0.1
                twist.angular.z = self.angular_speed * 1.5
            else:
                self.get_logger().info("Recovery complete — resuming EXPLORING")
                self.state = State.EXPLORING
                self.stuck_counter = 0

        elif min_all < self.safe_dist * 0.6:
            # Dangerously close — potential stuck situation
            self.stuck_counter += 1
            if self.stuck_counter > 3:
                self.get_logger().warn("Stuck detected! Entering RECOVERY mode.")
                self.state = State.RECOVERING
                self.recovery_start = now
            else:
                self.state = State.AVOIDING
                twist.linear.x = 0.0
                turn_dir = 1.0 if min_left > min_right else -1.0
                twist.angular.z = self.angular_speed * turn_dir

        elif min_front < self.safe_dist:
            # Obstacle ahead — steer away
            self.state = State.AVOIDING
            self.stuck_counter = max(0, self.stuck_counter - 1)
            turn_dir = 1.0 if min_left > min_right else -1.0
            twist.linear.x = 0.05
            twist.angular.z = self.angular_speed * turn_dir

        elif best_bin == forward_bin:
            # Clear path ahead — full speed
            self.state = State.EXPLORING
            self.stuck_counter = max(0, self.stuck_counter - 1)
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        else:
            # Steer toward best open valley
            self.state = State.AVOIDING
            self.stuck_counter = max(0, self.stuck_counter - 1)
            angle_error = ((best_bin - forward_bin) / self.n_bins) * 2 * np.pi
            twist.linear.x = self.linear_speed * 0.6
            twist.angular.z = float(
                np.clip(
                    -angle_error * 1.5,
                    -self.angular_speed,
                    self.angular_speed
                )
            )

        self.last_twist = twist
        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    def _build_histogram(self, ranges, angles):
        """Build a polar obstacle density histogram using VFH."""
        hist = np.zeros(self.n_bins)
        bin_width = 2 * np.pi / self.n_bins
        for r, a in zip(ranges, angles):
            bin_idx = int((a + np.pi) / bin_width) % self.n_bins
            if r < self.warn_dist:
                danger = (self.warn_dist - r) / self.warn_dist
                hist[bin_idx] = max(hist[bin_idx], danger)
        return hist

    # ------------------------------------------------------------------
    def _best_valley(self, open_bins, preferred):
        """Return the open bin closest to the preferred direction."""
        if open_bins[preferred]:
            return preferred
        for offset in range(1, self.n_bins // 2):
            if open_bins[(preferred + offset) % self.n_bins]:
                return (preferred + offset) % self.n_bins
            if open_bins[(preferred - offset) % self.n_bins]:
                return (preferred - offset) % self.n_bins
        return preferred  # fallback


# ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Obstacle Avoider.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
