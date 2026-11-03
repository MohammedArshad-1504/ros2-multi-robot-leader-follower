#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LeaderSafetyController(Node):

    def __init__(self):
        super().__init__('leader_safety_controller')

        # =========================================================
        # Parameters
        # =========================================================

        self.declare_parameter('obstacle_stop_distance', 0.60)
        self.declare_parameter('obstacle_slow_distance', 1.00)

        self.declare_parameter('front_sector_degrees', 30.0)

        self.declare_parameter('command_timeout', 0.50)
        self.declare_parameter('scan_timeout', 0.50)
        self.declare_parameter('blocked_recovery_time', 1.00)
        self.declare_parameter('blocked_recovery_turn_speed', 0.70)
        self.declare_parameter('blocked_recovery_front_distance', 0.65)

        # =========================================================
        # Read parameters
        # =========================================================

        self.obstacle_stop_distance = float(
            self.get_parameter(
                'obstacle_stop_distance'
            ).value
        )

        self.obstacle_slow_distance = float(
            self.get_parameter(
                'obstacle_slow_distance'
            ).value
        )

        self.front_sector_degrees = float(
            self.get_parameter(
                'front_sector_degrees'
            ).value
        )

        self.command_timeout = float(
            self.get_parameter(
                'command_timeout'
            ).value
        )

        self.scan_timeout = float(
            self.get_parameter(
                'scan_timeout'
            ).value
        )

        self.blocked_recovery_time = float(
            self.get_parameter('blocked_recovery_time').value
        )
        self.blocked_recovery_turn_speed = float(
            self.get_parameter('blocked_recovery_turn_speed').value
        )
        self.blocked_recovery_front_distance = float(
            self.get_parameter('blocked_recovery_front_distance').value
        )

        self.front_sector_radians = math.radians(
            self.front_sector_degrees
        )

        # =========================================================
        # State
        # =========================================================

        self.latest_command = Twist()

        self.front_obstacle_distance = float('inf')
        self.front_left_distance = float('inf')
        self.front_right_distance = float('inf')
        self.blocked_since = None

        self.last_command_time = None
        self.last_scan_time = None

        self.last_output_linear = 0.0
        self.last_output_angular = 0.0
        self.last_safety_stop = False

        # =========================================================
        # Subscribers
        # =========================================================

        self.command_subscriber = self.create_subscription(
            Twist,
            '/leader/cmd_vel',
            self.command_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/leader/front_lidar/scan',
            self.scan_callback,
            10
        )

        # =========================================================
        # Publisher
        # =========================================================

        self.safe_command_publisher = self.create_publisher(
            Twist,
            '/leader/cmd_vel_safe',
            10
        )

        # =========================================================
        # Control timer
        # =========================================================

        self.control_rate = 20.0

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        # =========================================================
        # Debug timer
        # =========================================================

        self.debug_timer = self.create_timer(
            0.5,
            self.debug_status
        )

        # =========================================================
        # Startup messages
        # =========================================================

        self.get_logger().info(
            'Leader safety controller started.'
        )

        self.get_logger().info(
            f'Obstacle STOP distance: '
            f'{self.obstacle_stop_distance:.2f} m'
        )

        self.get_logger().info(
            f'Obstacle SLOW distance: '
            f'{self.obstacle_slow_distance:.2f} m'
        )

        self.get_logger().info(
            f'LiDAR front sector: '
            f'-{self.front_sector_degrees:.0f}° '
            f'to +{self.front_sector_degrees:.0f}°'
        )

        self.get_logger().info(
            f'Command timeout: '
            f'{self.command_timeout:.2f} s'
        )

    # =============================================================
    # Leader command callback
    # =============================================================

    def command_callback(self, msg):

        self.latest_command = msg
        self.last_command_time = self.get_clock().now()

    # =============================================================
    # Leader LiDAR callback
    # =============================================================

    def scan_callback(self, msg):

        self.last_scan_time = self.get_clock().now()

        minimum_front = float('inf')
        minimum_front_left = float('inf')
        minimum_front_right = float('inf')

        front_angle_min = -self.front_sector_radians
        front_angle_max = self.front_sector_radians

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + index * msg.angle_increment

            if front_angle_min <= angle <= front_angle_max:
                minimum_front = min(minimum_front, distance)
            elif math.radians(30.0) < angle <= math.radians(90.0):
                minimum_front_left = min(minimum_front_left, distance)
            elif -math.radians(90.0) <= angle < -math.radians(30.0):
                minimum_front_right = min(minimum_front_right, distance)

        self.front_obstacle_distance = minimum_front
        self.front_left_distance = minimum_front_left
        self.front_right_distance = minimum_front_right

    # =============================================================
    # Main safety loop
    # =============================================================

    def control_loop(self):

        now = self.get_clock().now()

        # ---------------------------------------------------------
        # No command received yet
        # ---------------------------------------------------------

        if self.last_command_time is None:
            self.publish_stop()
            return

        # ---------------------------------------------------------
        # Command timeout
        #
        # If the user stops publishing commands, stop the robot.
        # ---------------------------------------------------------

        command_age = (
            now - self.last_command_time
        ).nanoseconds / 1e9

        if command_age > self.command_timeout:
            self.publish_stop()
            return

        # ---------------------------------------------------------
        # LiDAR timeout
        #
        # If safety sensor data disappears, stop.
        # ---------------------------------------------------------

        if self.last_scan_time is None:
            self.publish_stop()
            return

        scan_age = (
            now - self.last_scan_time
        ).nanoseconds / 1e9

        if scan_age > self.scan_timeout:
            self.publish_stop()
            return

        # ---------------------------------------------------------
        # Start with user's requested command
        # ---------------------------------------------------------

        output = Twist()

        output.linear.x = self.latest_command.linear.x
        output.linear.y = self.latest_command.linear.y
        output.linear.z = self.latest_command.linear.z

        output.angular.x = self.latest_command.angular.x
        output.angular.y = self.latest_command.angular.y
        output.angular.z = self.latest_command.angular.z

        requested_linear = output.linear.x

        safety_stop = False

        if (
            requested_linear > 0.05
            and self.front_obstacle_distance < self.blocked_recovery_front_distance
        ):
            if self.blocked_since is None:
                self.blocked_since = now
        else:
            self.blocked_since = None

        # ---------------------------------------------------------
        # Only apply forward obstacle protection when moving
        # forward.
        # ---------------------------------------------------------

        if requested_linear > 0.0:

            obstacle_distance = self.front_obstacle_distance

            # -----------------------------------------------------
            # Emergency stop
            # -----------------------------------------------------

            if obstacle_distance < self.obstacle_stop_distance:

                output.linear.x = 0.0
                safety_stop = True

            # -----------------------------------------------------
            # Slow down
            # -----------------------------------------------------

            elif (
                obstacle_distance
                < self.obstacle_slow_distance
            ):

                scale = (
                    obstacle_distance
                    - self.obstacle_stop_distance
                ) / (
                    self.obstacle_slow_distance
                    - self.obstacle_stop_distance
                )

                scale = max(
                    0.0,
                    min(scale, 1.0)
                )

                output.linear.x = (
                    requested_linear * scale
                )

        if self.blocked_since is not None:
            blocked_age = (
                now - self.blocked_since
            ).nanoseconds / 1e9

            if blocked_age >= self.blocked_recovery_time:
                output.linear.x = 0.0
                if self.front_left_distance > self.front_right_distance:
                    output.angular.z = abs(self.blocked_recovery_turn_speed)
                else:
                    output.angular.z = -abs(self.blocked_recovery_turn_speed)
                safety_stop = True

        # ---------------------------------------------------------
        # Save debug values
        # ---------------------------------------------------------

        self.last_output_linear = output.linear.x
        self.last_output_angular = output.angular.z
        self.last_safety_stop = safety_stop

        # ---------------------------------------------------------
        # Publish safe command
        # ---------------------------------------------------------

        self.safe_command_publisher.publish(output)

    # =============================================================
    # Publish stop
    # =============================================================

    def publish_stop(self):

        output = Twist()

        output.linear.x = 0.0
        output.linear.y = 0.0
        output.linear.z = 0.0

        output.angular.x = 0.0
        output.angular.y = 0.0
        output.angular.z = 0.0

        self.last_output_linear = 0.0
        self.last_output_angular = 0.0
        self.last_safety_stop = True

        self.safe_command_publisher.publish(output)

    # =============================================================
    # Debug information
    # =============================================================

    def debug_status(self):

        if self.last_scan_time is None:
            obstacle_text = 'waiting for LiDAR'
        elif math.isinf(self.front_obstacle_distance):
            obstacle_text = 'no valid obstacle'
        else:
            obstacle_text = (
                f'{self.front_obstacle_distance:.2f} m'
            )

        if self.last_safety_stop:
            status = 'STOP'
        else:
            status = 'PASS'

        self.get_logger().info(
            f'[LEADER {status}] '
            f'Front obstacle: {obstacle_text} | '
            f'Sides: L={self.front_left_distance:.2f} '
            f'R={self.front_right_distance:.2f} | '
            f'Output: '
            f'linear={self.last_output_linear:.3f} m/s '
            f'angular={self.last_output_angular:.3f} rad/s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = LeaderSafetyController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

