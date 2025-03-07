import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare any arguments that can be passed in via command line
        DeclareLaunchArgument(
            'robot_name', 
            default_value='0', 
            description='Waypoint Count'
        ),
        
        # Log a message when launching
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('robot_name', 'robot_1'),
            msg="Starting Mode Switcher"
        ),
        
        # Launch the first node
        Node(
            package='waypoint_counter',  # Replace with your package name
            executable='tracker',  # Replace with your executable name
            name='robot_node',  # Node name
            output='screen',
            parameters=[{
            }],
        ),
    ])
