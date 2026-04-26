from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    teleop_launch = PathJoinSubstitution(
        [FindPackageShare('jimmbot_controller'), 'launch', 'teleop_joy.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(teleop_launch)),
        Node(
            package='jimmbot_hardware',
            executable='extended_joy',
            name='extended_joy_node',
            remappings=[
                ('camera_tilt_front', 'jimmbot_controller/kinect_front_position_controller/command'),
                ('camera_tilt_back', 'jimmbot_controller/kinect_back_position_controller/command'),
                ('extn_data', '/jimmbot_controller/jimmbot_extn_data_controller/extn_data'),
            ],
        ),
    ])