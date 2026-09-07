from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("teleop_iphone")

    teleop_config = PathJoinSubstitution([pkg_share, "config", "teleop_iphone_zmq.yaml"])

    teleop_node = Node(
        package="teleop_iphone",
        executable="teleop_iphone_zmq.py",
        name="teleop_iphone_zmq",
        output="screen",
        parameters=[teleop_config],
        emulate_tty=True,
    )

    # Static transform for iPhone frame
    tf_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0.707", "0.0", "-0.707", "0.0", "iphone", "iphone_ros"],
    )

    return LaunchDescription([
        teleop_node,
        tf_static,
    ])
