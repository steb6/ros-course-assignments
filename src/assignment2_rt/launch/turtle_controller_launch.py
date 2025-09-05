from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        # Launch turtle controller using Node (proper ROS 2 way)
        Node(
            package='assignment2_rt',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        )
    ])
