from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
