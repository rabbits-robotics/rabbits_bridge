import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    gamepad_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rabbits_interface'), 'launch'),
            '/gamepad.launch.py'])
    )

    # game_client_interface_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('rabbits_interface'), 'launch'),
    #         '/game_client.launch.py'])
    # )

    stm32_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rabbits_bridge'), 'launch'),
            '/stm32_bridge.launch.py'])
    )

    transceiver_module_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rabbits_bridge'), 'launch'),
            '/transceiver_module_bridge.launch.py'])
    )

    ld.add_action(gamepad_interface_launch)
    # ld.add_action(game_client_interface_launch)
    ld.add_action(stm32_bridge_launch)
    ld.add_action(transceiver_module_bridge_launch)

    return ld
