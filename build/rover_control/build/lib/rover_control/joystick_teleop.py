#!/usr/bin/env python3
# Copyright 2026 User
# SPDX-License-Identifier: MIT

"""
Joystick Teleop Node for Rover Control
Subscribes to /joy topic and publishes Twist messages to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickTeleop(Node):
    """
    Node that converts joystick input to velocity commands.
    
    Subscribes to:
        /joy (sensor_msgs/Joy): Joystick input from joy_node
    
    Publishes to:
        /cmd_vel (geometry_msgs/Twist): Velocity commands for the rover
    """

    def __init__(self):
        super().__init__('joystick_teleop')
        
        # Declare parameters with defaults
        self.declare_parameter('linear_axis', 1)  # Left stick Y-axis
        self.declare_parameter('angular_axis', 0)  # Left stick X-axis
        self.declare_parameter('linear_scale', 1.0)  # Max linear speed (m/s)
        self.declare_parameter('angular_scale', 1.0)  # Max angular speed (rad/s)
        self.declare_parameter('deadzone', 0.1)  # Joystick deadzone
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        # Get parameters
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Current twist command
        self.twist = Twist()
        self.joy_received = False
        
        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Create timer for publishing at fixed rate
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Joystick Teleop Node started')
        self.get_logger().info(f'  Linear axis: {self.linear_axis}, scale: {self.linear_scale}')
        self.get_logger().info(f'  Angular axis: {self.angular_axis}, scale: {self.angular_scale}')
        self.get_logger().info(f'  Deadzone: {self.deadzone}')

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def joy_callback(self, msg: Joy):
        """Process joystick input and update twist command."""
        if len(msg.axes) <= max(self.linear_axis, self.angular_axis):
            self.get_logger().warn(
                f'Joystick has {len(msg.axes)} axes, but needs at least '
                f'{max(self.linear_axis, self.angular_axis) + 1}'
            )
            return
        
        # Get axes values with deadzone
        linear = self.apply_deadzone(msg.axes[self.linear_axis])
        angular = self.apply_deadzone(msg.axes[self.angular_axis])
        
        # Apply scaling
        self.twist.linear.x = linear * self.linear_scale
        self.twist.angular.z = angular * self.angular_scale
        
        self.joy_received = True

    def timer_callback(self):
        """Publish current twist command at fixed rate."""
        if not self.joy_received:
            # Don't publish until we receive joystick data
            return
        
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = JoystickTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero velocity on shutdown
        node.twist = Twist()
        node.cmd_vel_pub.publish(node.twist)
        node.get_logger().info('Shutting down, sent zero velocity')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
