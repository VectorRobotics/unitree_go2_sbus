# Unitree Go2 ROS 2 Package

A clean, type-safe ROS 2 package for controlling the Unitree Go2 robot via SBUS protocol over serial communication.

## Features

- **Type-safe Python implementation** with full type hints
- **ROS 2 Jazzy compatible** node for velocity control
- **Flexible message support**: Subscribe to either `Twist` or `TwistStamped` messages
- **Configurable parameters**: Launch file and YAML configuration support
- **Safety features**: Watchdog timer, velocity normalization, auto-unlock option
- **Clean architecture**: Separation of hardware control and ROS logic

## Package Structure

```
unitree_go2_ros/
├── config/
│   └── go2_control.yaml          # Configuration file
├── launch/
│   └── go2_control.launch.py     # Launch file
├── unitree_go2_ros/
│   ├── __init__.py
│   ├── go2_control.py            # Hardware control interface
│   └── go2_control_node.py       # ROS 2 node
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Dependencies

- ROS 2 Jazzy
- Python 3.10+
- `rclpy`
- `geometry_msgs`
- `pyserial`

## Installation

1. Navigate to your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Clone or copy this package into your workspace:
   ```bash
   cp -r /path/to/unitree_go2_ros .
   ```

3. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the package:
   ```bash
   colcon build --packages-select unitree_go2_ros
   ```

5. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Basic Launch

Launch with default parameters (subscribes to `Twist` on `/cmd_vel`):

```bash
ros2 launch unitree_go2_ros go2_control.launch.py
```

### Launch with TwistStamped

To use `TwistStamped` messages instead:

```bash
ros2 launch unitree_go2_ros go2_control.launch.py use_stamped:=true
```

### Launch with Custom Parameters

```bash
ros2 launch unitree_go2_ros go2_control.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baudrate:=115200 \
    use_stamped:=true \
    cmd_vel_topic:=/my_robot/cmd_vel \
    max_linear_velocity:=2.0 \
    max_angular_velocity:=1.5
```

### Launch with Configuration File

```bash
ros2 launch unitree_go2_ros go2_control.launch.py \
    --params-file src/unitree_go2_ros/config/go2_control.yaml
```

### Running the Node Directly

```bash
ros2 run unitree_go2_ros go2_control_node
```

## Configuration

### Launch Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyACM0` | Serial port for Arduino connection |
| `baudrate` | int | `115200` | Serial communication baud rate |
| `use_stamped` | bool | `false` | Use TwistStamped instead of Twist |
| `cmd_vel_topic` | string | `cmd_vel` | Topic name for velocity commands |
| `auto_unlock` | bool | `true` | Automatically unlock robot on startup |
| `max_linear_velocity` | float | `1.0` | Maximum linear velocity (m/s) |
| `max_angular_velocity` | float | `1.0` | Maximum angular velocity (rad/s) |

### Configuration File

Edit `config/go2_control.yaml` to customize default parameters:

```yaml
go2_control_node:
  ros__parameters:
    serial_port: "/dev/ttyACM0"
    baudrate: 115200
    use_stamped: false
    cmd_vel_topic: "cmd_vel"
    auto_unlock: true
    max_linear_velocity: 1.0
    max_angular_velocity: 1.0
```

## Message Format

### Twist

```
geometry_msgs/Twist
  Vector3 linear
    float64 x    # Forward/backward velocity (m/s)
    float64 y    # Left/right velocity (m/s)
    float64 z    # (unused)
  Vector3 angular
    float64 x    # (unused)
    float64 y    # (unused)
    float64 z    # Yaw rate (rad/s)
```

### TwistStamped

```
geometry_msgs/TwistStamped
  std_msgs/Header header
    builtin_interfaces/Time stamp
    string frame_id
  geometry_msgs/Twist twist
    # Same as Twist above
```

## Coordinate System

The node uses standard ROS conventions:

- **X-axis**: Forward (positive) / Backward (negative)
- **Y-axis**: Left (negative) / Right (positive)
- **Z-axis (angular)**: Counter-clockwise (positive) / Clockwise (negative)

## Testing

### Test with Command Line

Publish a simple forward command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Publish with TwistStamped:

```bash
ros2 topic pub /cmd_vel geometry_msgs/TwistStamped \
    '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
```

### Test with Teleop Keyboard

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Hardware Control API

The `Go2Control` class can also be used standalone (without ROS):

```python
from unitree_go2_ros.go2_control import Go2Control

# Use context manager for automatic cleanup
with Go2Control(port="/dev/ttyACM0", baudrate=115200) as robot:
    # Unlock the robot
    robot.unlock()

    # Move forward
    robot.move(x=0.5, y=0.0, z=0.0)

    # Stop
    robot.stop()
```

### Available Commands

- `move(x, y, z)` - Send movement command (values -1.0 to 1.0)
- `stop()` - Stop all motion
- `unlock()` - Unlock the robot
- `toggle_damping()` - Toggle damping mode
- `recover()` - Execute fall recovery
- `toggle_continuous_movement()` - Toggle continuous movement
- `toggle_stand_height()` - Toggle stand height
- `switch_gait()` - Switch walking gait
- `toggle_light()` - Toggle LED light
- `dance()` - Execute dance sequence
- `jump()` - Execute jump sequence

## Safety Features

1. **Watchdog Timer**: Automatically stops the robot if no commands are received for 0.5 seconds
2. **Velocity Normalization**: Input velocities are normalized to [-1, 1] range based on configured max velocities
3. **Value Clamping**: All SBUS values are clamped to valid ranges
4. **Exception Handling**: Robust error handling for serial communication failures

## Troubleshooting

### Serial Port Permission Denied

Add your user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and log back in.

### Cannot Find Serial Port

List available serial ports:

```bash
ls /dev/tty*
```

Common Arduino ports: `/dev/ttyACM0`, `/dev/ttyUSB0`

### Robot Not Responding

1. Check serial connection and baud rate
2. Verify Arduino is powered and connected
3. Ensure robot is unlocked (check `auto_unlock` parameter)
4. Check ROS topic publishing: `ros2 topic echo /cmd_vel`

## License

MIT

## Author

Your Name (your.email@example.com)
