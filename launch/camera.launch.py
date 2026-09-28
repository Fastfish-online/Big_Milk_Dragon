from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('ip_address', default_value=''),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_frame'),
        DeclareLaunchArgument('exposure_time', default_value='5000.0'),
        DeclareLaunchArgument('gain', default_value='0.0'),
        DeclareLaunchArgument('frame_rate', default_value='30.0'),
        DeclareLaunchArgument('pixel_format', default_value='BGR8'),
    ]
    return LaunchDescription(arguments + [
        Node(
            package='hikrobot_camera', executable='hikrobot_camera_node',
            name='hikrobot_camera', output='screen',
            parameters=[{
                'serial_number': LaunchConfiguration('serial_number'),
                'ip_address': LaunchConfiguration('ip_address'),
                'image_topic': LaunchConfiguration('image_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'exposure_time': LaunchConfiguration('exposure_time'),
                'gain': LaunchConfiguration('gain'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'pixel_format': LaunchConfiguration('pixel_format'),
            }],
        )
    ])
