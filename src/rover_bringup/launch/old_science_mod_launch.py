from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ========================================
        # DRIVE CONTROLLER (js0)
        # ========================================
        Node(
            package='joy',
            executable='joy_node',
            name='joy_drive',
            parameters=[{
                'device_id': 1,  # /dev/input/js0
                'deadzone': 0.5,
                'autorepeat_rate': 20.0
            }],
            remappings=[
                ('joy', 'joy_drive')
            ]
        ),
        
        # Drive teleop node
        Node(
            package='rover_teleop',
            executable='joy_to_twist',
            name='joy_to_twist_drive',
            remappings=[
                ('joy', 'joy_drive'),
                ('cmd_vel', 'drive/cmd_vel')
            ]
        ),
        
        # ========================================
        # SCIENCE CONTROLLER (js1)
        # ========================================
        Node(
            package='joy',
            executable='joy_node',
            name='joy_science',
            parameters=[{
                'device_id': 0,  # /dev/input/js1
                'deadzone': 0.5,
                'autorepeat_rate': 20.0
            }],
            remappings=[
                ('joy', 'joy_science')
            ]
        ),
        
        # Science teleop node
        Node(
            package='rover_teleop',
            executable='old_science_mod_node',
            name='joy_to_twist_science',
            remappings=[
                ('joy', 'joy_science'),
                ('cmd_vel', 'science/cmd_vel')
            ]
        ),
        
        # ========================================
        # MICRO-ROS AGENT
        # ========================================
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0']
        )
    ])