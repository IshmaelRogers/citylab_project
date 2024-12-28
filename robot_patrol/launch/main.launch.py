from launch import LaunchDescription
import os
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([os.environ['HOME'],'ros2_ws/src/citylab_project/robot_patrol/config/patrol_robot.rviz']),
        description='Path to config'
        ),

        Node(
            package='robot_patrol',
            executable = 'direction_service',
            name='direction_service_node',
            output='screen'
        ),
        Node(
            package='robot_patrol',
            executable='robot_patrol_client_node',
            name='robot_patrol_client_node',
            output='screen',
            emulate_tty=True
        )
    ])