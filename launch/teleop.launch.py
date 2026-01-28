from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("teleop_iphone")

    osc_config = PathJoinSubstitution([pkg_share, "config", "osc_params.yaml"])
    teleop_config = PathJoinSubstitution([pkg_share, "config", "teleop_iphone.yaml"])

    osc_node = Node(
        package="teleop_iphone",
        executable="osc_node",
        name="osc_node",
        output="screen",
        parameters=[osc_config],
        emulate_tty=True,
    )

    teleop_node = Node(
        package="teleop_iphone",
        executable="teleop_iphone",
        name="teleop_iphone",
        output="screen",
        parameters=[teleop_config],
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
            osc_node,
            teleop_node,
            rviz_node,
            *tf_static,
        ]
    )
