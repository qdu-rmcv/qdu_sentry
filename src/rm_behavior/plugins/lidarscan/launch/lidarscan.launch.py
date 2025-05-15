
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('lidarscan')

    params_file = os.path.join(pkg_dir, 'config', 'lidarscan.yaml')

    
    lidarscan_cmd = Node(
            package='lidarscan',
            executable='lidarscan_node',
            name='lidarscan_node',
            output='screen',
            parameters=[params_file],
    )

    ld = LaunchDescription()

    ld.add_action(lidarscan_cmd)

    return ld