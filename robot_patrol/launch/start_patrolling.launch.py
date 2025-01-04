
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value = PathJoinSubstitution([os.environ['HOME'], 'ros2_ws/src/citylab_project/robot_patrol/config/patrol_robot.rviz']),
            description = "Path to rviz config" 
        ),


        Node(
            package='robot_patrol',
            executable='patrol',
            name='patrol',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'rviz2', 
                '-d', '/home/user/ros2_ws/src/robot_patrol/config/patrol_config.rviz'
            ],
            output='screen'
        )
    ])
