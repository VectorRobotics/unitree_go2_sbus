"""Unitree Go2 Robot Control Interface via SBUS Protocol.

This module provides a clean, type-safe interface for controlling a Unitree Go2
robot through SBUS commands sent via serial communication.
"""

from __future__ import annotations

import time
from typing import Final

import serial


class Go2Control:
    """Control interface for Unitree Go2 robot via SBUS protocol.

    This class provides methods to control robot movement and execute various
    commands through serial communication with an Arduino-based SBUS controller.

    Attributes:
        LOW: Minimum SBUS channel value (192)
        MID: Middle/neutral SBUS channel value (992)
        HIGH: Maximum SBUS channel value (1792)
        DEAD_ZONE: Dead zone threshold for inputs (100)
    """

    # SBUS channel value constants
    LOW: Final[int] = 192
    MID: Final[int] = 992
    HIGH: Final[int] = 1792
    DEAD_ZONE: Final[int] = 100

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        timeout: float = 1.0,
    ) -> None:
        """Initialize robot control interface.

        Args:
            port: Serial port path for Arduino connection
            baudrate: Serial communication baud rate
            timeout: Serial read timeout in seconds

        Raises:
            serial.SerialException: If serial port cannot be opened
        """
        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for Arduino to reset after connection

    def send_command(self, cmd: str) -> None:
        """Send a command string to the robot.

        Args:
            cmd: Command string to send (without newline)
        """
        self._serial.write(f"{cmd}\n".encode("utf-8"))
        time.sleep(0.02)  # Small delay to ensure command is processed

    def move(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
    ) -> None:
        """Send movement command to the robot.

        Args:
            x: Forward/Backward motion (-1.0 to 1.0, negative is backward)
            y: Strafe left/right motion (-1.0 to 1.0, negative is left)
            z: Yaw rotation (-1.0 to 1.0, negative is counter-clockwise)
            pitch: Pitch up/down motion (-1.0 to 1.0)

        Note:
            All values are clamped to the range [-1.0, 1.0]
        """
        # Convert normalized values to SBUS range
        strafe_val = self._normalize_to_sbus(y)
        forward_val = self._normalize_to_sbus(x)
        yaw_val = self._normalize_to_sbus(z)

        self.send_command(f"move:{strafe_val},{forward_val},{yaw_val}")

    def _normalize_to_sbus(self, value: float) -> int:
        """Convert normalized value (-1.0 to 1.0) to SBUS range.

        Args:
            value: Normalized input value (-1.0 to 1.0)

        Returns:
            SBUS value constrained to valid range (LOW to HIGH)
        """
        # Clamp input to valid range
        clamped = max(-1.0, min(1.0, value))

        # Convert to SBUS range
        sbus_value = int(self.MID + (clamped * (self.HIGH - self.MID)))

        # Constrain to SBUS limits
        return max(self.LOW, min(self.HIGH, sbus_value))

    def unlock(self) -> None:
        """Unlock the robot (Mode 1, Channel 7).

        This must typically be called before the robot will accept movement commands.
        """
        self.send_command("unlock")

    def toggle_damping(self) -> None:
        """Toggle damping mode (Mode 1, Channel 6).

        Damping mode makes the robot compliant and allows manual manipulation.
        """
        self.send_command("damping")

    def recover(self) -> None:
        """Execute fall recovery sequence (Mode 2, Channel 7).

        The robot will attempt to stand up from a fallen position.
        """
        self.send_command("recover")

    def toggle_continuous_movement(self) -> None:
        """Toggle continuous movement mode (Mode 2, Channel 6)."""
        self.send_command("continuous")

    def toggle_stand_height(self) -> None:
        """Toggle between high and low stand heights (Mode 1, Channel 8)."""
        self.send_command("stand")

    def toggle_obstacle_avoidance(self) -> None:
        """Toggle obstacle avoidance mode (Mode 2, Channel 8)."""
        self.send_command("obstacle")

    def switch_gait(self) -> None:
        """Switch walking gait pattern (Mode 3, Channel 8 high)."""
        self.send_command("gait")

    def toggle_light(self) -> None:
        """Toggle LED light on/off (Mode 3, Channel 8 low)."""
        self.send_command("light")

    def dance(self) -> None:
        """Execute dance sequence (Mode 3, Channel 7)."""
        self.send_command("dance")

    def jump(self) -> None:
        """Execute jump sequence (Mode 3, Channel 6)."""
        self.send_command("jump")

    def stop(self) -> None:
        """Stop all motion by sending neutral values to all movement channels."""
        self.move(x=0.0, y=0.0, z=0.0, pitch=0.0)

    def close(self) -> None:
        """Stop the robot and close the serial connection."""
        self.stop()
        self._serial.close()


def main() -> None:
    """Example usage of the Go2 robot control interface."""
    try:
        # Use context manager for automatic cleanup
        with Go2Control() as robot:
            # Unlock the robot
            print("Unlocking robot...")
            robot.unlock()
            time.sleep(1)

            # Move forward for 2 seconds
            print("Moving forward...")
            start_time = time.time()
            while time.time() - start_time < 2.0:
                robot.move(x=0.5)
                time.sleep(0.05)

            # Stop the robot
            print("Stopping...")
            robot.stop()

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
