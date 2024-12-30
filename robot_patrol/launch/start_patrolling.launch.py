
import os
from launch import LaunchDescription
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
            output='screen',
        ),

        Node(

            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        
        ),
    ])
