#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TankDriveNode(Node):
    def __init__(self):
        super().__init__('tank_drive_node')
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def joy_callback(self, msg):
        # Logitech F710 Axes: 1 is Left Stick Y, 5 is Right Stick Y
        left_y = msg.axes[4] * -1  # Invert Left Stick Y to match typical forward positive convention
        right_y = msg.axes[1]   

        twist = Twist()
        
        # Convert Tank to Twist
        # Linear X is the average of both sticks
        twist.linear.x = (left_y + right_y) / 2.0
        
        # Angular Z is the difference between them
        # (If Left is up and Right is down, the robot spins right)
        twist.angular.z = (left_y - right_y) / 2.0

        self.pub.publish(twist)

def main():
    rclpy.init()
    node = TankDriveNode()
    rclpy.spin(node)
    rclpy.shutdown()