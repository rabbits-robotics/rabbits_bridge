from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    transceiver_module_bridge_node = Node(
        package='rabbits_bridge',
        executable='transceiver_module_bridge_node',
        name='transceiver_module_bridge_node',
        output='screen',
    )

    ld.add_action(transceiver_module_bridge_node)

    return ld
