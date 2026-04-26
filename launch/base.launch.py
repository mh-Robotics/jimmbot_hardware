from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_interface_launch = PathJoinSubstitution(
        [FindPackageShare('jimmbot_hardware'), 'launch', 'hardware_interface.launch.py']
    )
    controller_launch = PathJoinSubstitution(
        [FindPackageShare('jimmbot_controller'), 'launch', 'controller.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(hardware_interface_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch)),
    ])