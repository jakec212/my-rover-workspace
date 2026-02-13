#!/usr/bin/env python3
"""
joy_to_twist.py - ROS2 Node for converting joystick input to Twist messages

This node subscribes to the /joy topic (from joy_node) and publishes Twist messages
to /cmd_vel for robot velocity control.

Key Concepts:
1. ROS2 Node: A process that performs computation (inherits from Node class)
2. Subscriber: Receives messages from a topic
3. Publisher: Sends messages to a topic
4. Parameters: Configurable values that can be set from YAML files or command line
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToTwist(Node):
    """
    ROS2 Node that converts joystick input to velocity commands.
    
    In ROS2, nodes are classes that inherit from rclpy.node.Node.
    This provides all the ROS functionality (publishers, subscribers, parameters, etc.)
    """
    
    def __init__(self):
        # Initialize the parent Node class with the node name
        super().__init__('joy_to_twist')
        
        # ============ Declare Parameters ============
        # Parameters allow configuration without changing code
        # They can be set from YAML files or command line
        
        self.declare_parameter('axis_linear', 7)  # Which joystick axis controls forward/backward
        self.declare_parameter('axis_angular', 6)  # Which joystick axis controls turning
        self.declare_parameter('linear_speed_scale', 0.2)  # Max linear speed (m/s) ------------------
        self.declare_parameter('angular_speed_scale', 0.5)  # Max angular speed (rad/s)
        self.declare_parameter('publish_rate', 10.0)  # How often to publish (Hz)
        self.declare_parameter('deadzone', 0.05)  # Ignore small joystick movements
        
        # Get parameter values
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.linear_speed_scale = self.get_parameter('linear_speed_scale').value
        self.angular_speed_scale = self.get_parameter('angular_speed_scale').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # ============ Create Publisher ============
        # Publisher sends Twist messages to /cmd_vel topic
        # Queue size of 10 means it can buffer up to 10 messages
        self.twist_publisher = self.create_publisher(
            Twist,      # Message type
            '/cmd_vel',  # Topic name
            10          # Queue size
        )
        
        # ============ Create Subscriber ============
        # Subscriber receives Joy messages from /joy topic
        # joy_callback is called every time a message arrives
        self.joy_subscriber = self.create_subscription(
            Joy,                # Message type
            '/joy',             # Topic name
            self.joy_callback,  # Callback function
            10                  # Queue size
        )
        
        # ============ Create Timer ============
        # Timer calls publish_twist at a fixed rate (publish_rate Hz)
        # This ensures we publish velocity commands regularly
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_twist)
        
        # ============ State Variables ============
        # Store the latest joystick values
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Log that the node started successfully
        self.get_logger().info(f'Joy to Twist node started!')
        self.get_logger().info(f'  Linear axis: {self.axis_linear}')
        self.get_logger().info(f'  Angular axis: {self.axis_angular}')
        self.get_logger().info(f'  Linear scale: {self.linear_speed_scale} m/s')
        self.get_logger().info(f'  Angular scale: {self.angular_speed_scale} rad/s')
        
    def joy_callback(self, joy_msg):
        """
        Callback function called when a Joy message is received.
        
        Args:
            joy_msg (Joy): Joystick message containing axes and button values
        
        Key Concept: Callbacks in ROS2
        - This function is called automatically when a message arrives
        - It should be fast and non-blocking
        - Heavy computation should be done elsewhere
        """
        
        # Check if the joystick has the axes we need
        if len(joy_msg.axes) <= max(self.axis_linear, self.axis_angular):
            self.get_logger().warn('Joystick does not have enough axes!')
            return
        
        # Get the raw joystick values (-1.0 to 1.0)
        raw_linear = joy_msg.axes[self.axis_linear]
        raw_angular = joy_msg.axes[self.axis_angular]
        
        # Apply deadzone - ignore small movements to prevent drift
        # If the absolute value is less than deadzone, treat it as zero
        if abs(raw_linear) < self.deadzone:
            raw_linear = 0.0
        if abs(raw_angular) < self.deadzone:
            raw_angular = 0.0
        
        # Scale the values to actual velocities
        self.current_linear = raw_linear * self.linear_speed_scale
        self.current_angular = raw_angular * self.angular_speed_scale
        
    def publish_twist(self):
        """
        Timer callback that publishes Twist messages at a fixed rate.
        
        Key Concept: Twist Messages
        - Twist is used for velocity commands in ROS
        - Linear: [x, y, z] velocities in m/s
        - Angular: [x, y, z] velocities in rad/s
        - For differential drive robots, we only use:
          - linear.x (forward/backward)
          - angular.z (turning)
        """
        
        # Create a new Twist message
        twist_msg = Twist()
        
        # Set the velocities
        twist_msg.linear.x = self.current_linear   # Forward/backward (m/s)
        twist_msg.linear.y = 0.0                    # Left/right (not used for diff drive)
        twist_msg.linear.z = 0.0                    # Up/down (not used for diff drive)
        
        twist_msg.angular.x = 0.0                   # Roll (not used for diff drive)
        twist_msg.angular.y = 0.0                   # Pitch (not used for diff drive)
        twist_msg.angular.z = self.current_angular  # Yaw/turning (rad/s)
        
        # Publish the message
        self.twist_publisher.publish(twist_msg)
        
        # Optional: Log when we're sending non-zero commands (for debugging)
        if abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01:
            self.get_logger().debug(
                f'Publishing: linear={self.current_linear:.2f} m/s, '
                f'angular={self.current_angular:.2f} rad/s'
            )


def main(args=None):
    """
    Main entry point for the node.
    
    This function is called when you run: ros2 run rover_teleop joy_to_twist
    """
    
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our node
    joy_to_twist_node = JoyToTwist()
    
    try:
        # Keep the node running and processing callbacks
        # This blocks until the node is shut down (Ctrl+C)
        rclpy.spin(joy_to_twist_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up
        joy_to_twist_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()