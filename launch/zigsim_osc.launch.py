from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='192.168.1.11',
        description='UDP host address to bind to'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3333',
        description='UDP port to listen on'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='iphone',
        description='Frame ID for sensor data'
    )

    # Create node
    zigsim_node = Node(
        package='teleop_iphone',
        executable='osc_node',
        name='osc_node',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        frame_id_arg,
        zigsim_node
    ])
