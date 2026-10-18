from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    stm32_bridge_node = Node(
        package='rabbits_bridge',
        executable='stm32_bridge_node',
        name='stm32_bridge_node',
        output='screen',
    )

    ld.add_action(stm32_bridge_node)

    return ld
