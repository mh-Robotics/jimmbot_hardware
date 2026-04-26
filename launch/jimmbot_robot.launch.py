from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_bridge_launch = PathJoinSubstitution(
        [FindPackageShare('jimmbot_hardware'), 'launch', 'jimmbot_hw_bridge.launch.py']
    )
    sensors_launch = PathJoinSubstitution(
        [FindPackageShare('jimmbot_hardware'), 'launch', 'jimmbot_sensors.launch.py']
    )
    base_launch = PathJoinSubstitution(
        [FindPackageShare('jimmbot_hardware'), 'launch', 'base.launch.py']
    )
    audiofx_launch = PathJoinSubstitution(
        [FindPackageShare('jimmbot_hardware'), 'launch', 'audiofx.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(hw_bridge_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sensors_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(audiofx_launch)),
    ])