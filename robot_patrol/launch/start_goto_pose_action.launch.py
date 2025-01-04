from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_file = '/home/user/ros2_ws/src/robot_patrol/config/patrol_config.rviz'
    
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='go_to_pose_action',  # The name of your compiled node
            name='go_to_pose_action_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
