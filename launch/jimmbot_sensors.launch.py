from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    razor_config_file = LaunchConfiguration('razor_config_file')

    openni_launch = PathJoinSubstitution(
        [FindPackageShare('openni_launch'), 'launch', 'openni.launch']
    )
    default_razor_config = PathJoinSubstitution(
        [FindPackageShare('razor_imu_9dof'), 'config', 'razor.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'razor_config_file',
            default_value=default_razor_config,
            description='Path to the Razor IMU configuration file.',
        ),
        Node(
            package='ydlidar_ros',
            executable='ydlidar_node',
            name='ydlidar_node',
            output='screen',
            respawn=False,
            parameters=[
                {
                    'port': '/dev/ydlidar',
                    'baudrate': 230400,
                    'frame_id': 'laser_frame',
                    'resolution_fixed': True,
                    'auto_reconnect': True,
                    'reversion': True,
                    'angle_min': -180.0,
                    'angle_max': 180.0,
                    'range_min': 0.1,
                    'range_max': 16.0,
                    'ignore_array': '',
                    'frequency': 10.0,
                    'isTOFLidar': False,
                }
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_laser_frame',
            arguments=['0.2245', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_footprint', 'laser_frame'],
        ),
        Node(
            package='razor_imu_9dof',
            executable='imu_node.py',
            name='imu_node',
            output='screen',
            parameters=[razor_config_file],
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(openni_launch),
            launch_arguments={
                'camera': 'kinect_front',
                'device_id': 'B00367726232107B',
                'publish_tf': 'true',
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='physical_kinect_to_model_front',
            arguments=['-0.18', '0.0', '0.1', '0.0', '0.0', '0.0', 'kinect_front_link', 'kinect_front_depth_frame'],
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(openni_launch),
            launch_arguments={
                'camera': 'kinect_back',
                'device_id': 'A00365927030043A',
                'publish_tf': 'true',
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='physical_kinect_to_model_back',
            arguments=['-0.18', '0.0', '0.1', '0.0', '0.0', '0.0', 'kinect_back_link', 'kinect_back_depth_frame'],
        ),
    ])