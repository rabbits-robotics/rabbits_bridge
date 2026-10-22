import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('rabbits_bridge'), 'config', 'beta.yaml')

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='robot_bridge のパラメータ YAML（config/beta.yaml, config/gamma.yaml）')

    robot_bridge_node = Node(
        package='rabbits_bridge',
        executable='robot_bridge_node',
        name='robot_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, robot_bridge_node])
