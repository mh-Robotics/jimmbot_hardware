from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_config = PathJoinSubstitution(
        [FindPackageShare('jimmbot_base'), 'config', 'hardware_interface.yaml']
    )

    return LaunchDescription([
        Node(
            package='jimmbot_base',
            executable='hardware_interface',
            name='hardware_interface_node',
            parameters=[hardware_config],
            remappings=[
                ('command/can_msg', 'esp32/command/can_msg'),
                ('feedback/can_msg', 'esp32/feedback/can_msg'),
                ('extn_data', '/jimmbot_controller/jimmbot_extn_data_controller/extn_data'),
            ],
        ),
    ])