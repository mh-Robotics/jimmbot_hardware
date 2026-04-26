from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    audiofx_config = PathJoinSubstitution(
        [FindPackageShare('jimmbot_hardware'), 'config', 'audiofx.yaml']
    )

    return LaunchDescription([
        Node(
            package='sound_play',
            executable='soundplay_node.py',
            name='soundplay_node',
            parameters=[audiofx_config],
        ),
        Node(
            package='audio_play',
            executable='audio_play',
            name='audio_play_fx',
            parameters=[audiofx_config],
            remappings=[('/audio', '/jimmbot_audiofx/buffer_play')],
        ),
        Node(
            package='jimmbot_hardware',
            executable='audiofx_loader',
            name='audiofx_loader_node',
            parameters=[audiofx_config],
            remappings=[
                ('/audio', '/jimmbot_audiofx/buffer_play'),
                ('/extn_data', '/jimmbot_controller/jimmbot_extn_data_controller/extn_data'),
            ],
        ),
    ])