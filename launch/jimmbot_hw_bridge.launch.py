from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosserial_python',
            executable='serial_node.py',
            name='serial_node',
            respawn=True,
            parameters=[
                {
                    'port': '/dev/esp32_ros_node',
                    'baud': 115200,
                }
            ],
        ),
    ])