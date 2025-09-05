from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the executable
    assignment2_rt_path = get_package_share_directory('assignment2_rt')
    executable_path = os.path.join(assignment2_rt_path, '..', '..', 'bin', 'turtle_controller')
    
    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        # Launch turtle controller using ExecuteProcess
        ExecuteProcess(
            cmd=[executable_path],
            name='turtle_controller',
            output='screen'
        )
    ])
