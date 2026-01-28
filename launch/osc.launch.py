from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare("teleop_iphone")

    # Config file path
    config_file = PathJoinSubstitution([pkg_share, "config", "osc_params.yaml"])

    # Declare launch arguments for overriding config
    host_arg = DeclareLaunchArgument(
        "host",
        default_value="",
        description="UDP host address to bind to (overrides config file)",
    )

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="",
        description="UDP port to listen on (overrides config file)",
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="",
        description="Frame ID for sensor data (overrides config file)",
    )

    # Create node with config file
    osc_node = Node(
        package="teleop_iphone",
        executable="osc_node",
        name="osc_node",
        output="screen",
        parameters=[config_file],
        emulate_tty=True,
    )

    tf_static = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=["0", "0", "0", "0.707", "0.0", "-0.707", "0.0", "iphone", "iphone_ros"],
        )
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([pkg_share, "config", "default.rviz"])],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            host_arg,
            port_arg,
            frame_id_arg,
            osc_node,
            rviz_node,
            *tf_static,
        ]
    )
