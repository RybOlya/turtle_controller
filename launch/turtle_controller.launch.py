import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('turtle_controller')
    default_params_file = os.path.join(package_dir, 'params', 'default.yaml')

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtle_controller',
            executable='turtle_controller',
            name='turtle_controller',
            parameters=[default_params_file]
        )
    ])
