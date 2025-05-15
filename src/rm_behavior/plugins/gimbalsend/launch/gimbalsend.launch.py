import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('gimbalsend')
    
    params_file = os.path.join(pkg_dir, 'config', 'gimbalsend_params.yaml')

    gimbal_control_cmd = Node(
        name='gimbalsend',
        output='screen',
        executable='gimbalsend_node',
        package='gimbalsend',
        parameters=[params_file]
    ) 
    
    ld = LaunchDescription()
    
    ld.add_action(gimbal_control_cmd)

    return ld