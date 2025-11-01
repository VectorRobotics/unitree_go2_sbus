"""Launch file for Unitree Go2 control node.

This launch file starts the Go2 control node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for Go2 control node.

    Returns:
        LaunchDescription containing the Go2 control node configuration
    """
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for Arduino connection",
    )

    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="Serial communication baud rate",
    )

    use_stamped_arg = DeclareLaunchArgument(
        "use_stamped",
        default_value="false",
        description="Use TwistStamped instead of Twist messages",
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="Topic name for velocity commands",
    )

    auto_unlock_arg = DeclareLaunchArgument(
        "auto_unlock",
        default_value="true",
        description="Automatically unlock robot on startup",
    )

    # Create the node
    go2_control_node = Node(
        package="unitree_go2_ros",
        executable="go2_control_node",
        name="go2_control_node",
        output="screen",
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "use_stamped": LaunchConfiguration("use_stamped"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "auto_unlock": LaunchConfiguration("auto_unlock"),
            }
        ],
    )

    return LaunchDescription(
        [
            serial_port_arg,
            baudrate_arg,
            use_stamped_arg,
            cmd_vel_topic_arg,
            auto_unlock_arg,
            go2_control_node,
        ]
    )
