"""ROS 2 node for controlling Unitree Go2 robot via SBUS.

This node subscribes to Twist or TwistStamped messages and translates them
to SBUS commands for the Go2 robot.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial

from unitree_go2_ros.go2_control import Go2Control


class Go2ControlNode(Node):
    """ROS 2 node for Go2 robot control via velocity commands.

    This node subscribes to velocity commands (Twist or TwistStamped) and
    translates them to appropriate SBUS commands for the robot hardware.
    """

    def __init__(self) -> None:
        """Initialize the Go2 control node."""
        super().__init__("go2_control_node")

        # Declare parameters
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("use_stamped", False)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("auto_unlock", True)

        # Get parameters
        serial_port = self.get_parameter("serial_port").value
        baudrate = self.get_parameter("baudrate").value
        use_stamped = self.get_parameter("use_stamped").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._auto_unlock = self.get_parameter("auto_unlock").value

        # Initialize robot control
        try:
            self._robot = Go2Control(port=serial_port, baudrate=baudrate)
            self.get_logger().info(
                f"Connected to robot on {serial_port} at {baudrate} baud"
            )

            # Auto-unlock if enabled
            if self._auto_unlock:
                self._robot.unlock()
                self.get_logger().info("Robot unlocked")

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to robot: {e}")
            raise

        # Create subscriber based on message type
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        if use_stamped:
            self._cmd_vel_sub = self.create_subscription(
                TwistStamped, cmd_vel_topic, self._twist_stamped_callback, qos
            )
            self.get_logger().info(
                f"Subscribed to {cmd_vel_topic} (TwistStamped)"
            )
        else:
            self._cmd_vel_sub = self.create_subscription(
                Twist, cmd_vel_topic, self._twist_callback, qos
            )
            self.get_logger().info(f"Subscribed to {cmd_vel_topic} (Twist)")

        # Watchdog timer to stop robot if no commands received
        self._watchdog_timeout = 0.5  # seconds
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info("Go2 control node initialized successfully")

    def _twist_callback(self, msg: Twist) -> None:
        """Handle Twist messages.

        Args:
            msg: Twist message containing velocity commands
        """
        self._process_velocity_command(msg)

    def _twist_stamped_callback(self, msg: TwistStamped) -> None:
        """Handle TwistStamped messages.

        Args:
            msg: TwistStamped message containing velocity commands
        """
        self._process_velocity_command(msg.twist)

    def _process_velocity_command(self, twist: Twist) -> None:
        """Process velocity command and send to robot.

        Args:
            twist: Twist message containing linear and angular velocities
        """
        # Update last command time
        self._last_cmd_time = self.get_clock().now()

        # Extract velocities directly from twist message
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z

        # Send command to robot
        try:
            self._robot.move(x=x, y=y, z=z)
        except Exception as e:
            self.get_logger().error(f"Failed to send command to robot: {e}")

    def _watchdog_callback(self) -> None:
        """Watchdog timer callback to stop robot if no commands received."""
        time_since_last_cmd = (
            self.get_clock().now() - self._last_cmd_time
        ).nanoseconds / 1e9

        if time_since_last_cmd > self._watchdog_timeout:
            try:
                self._robot.stop()
            except Exception as e:
                self.get_logger().error(f"Failed to stop robot: {e}")

    def destroy_node(self) -> None:
        """Clean up resources when node is destroyed."""
        self.get_logger().info("Shutting down Go2 control node")
        try:
            self._robot.close()
        except Exception as e:
            self.get_logger().error(f"Error closing robot connection: {e}")
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the Go2 control node.

    Args:
        args: Command-line arguments
    """
    rclpy.init(args=args)

    try:
        node = Go2ControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Go2 control node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
