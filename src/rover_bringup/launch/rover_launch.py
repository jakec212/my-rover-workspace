from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. The Joystick Driver Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.5}]
        ),

        # 2. Your custom Teleop Node
        Node(
            package='rover_teleop',
            executable='joy_to_twist',
            name='joy_to_twist'
        ),

        # 3. The Micro-ROS Agent (Serial connection)
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0']
        )
    ])