# Quick Start Guide

## 1. Build the Package

```bash
cd ~/ros2_ws/src
cp -r /path/to/unitree_go2_ros .
cd ~/ros2_ws
colcon build --packages-select unitree_go2_ros
source install/setup.bash
```

## 2. Launch Options

### Option A: Standard Twist Messages (Default)
```bash
ros2 launch unitree_go2_ros go2_control.launch.py
```

### Option B: TwistStamped Messages
```bash
ros2 launch unitree_go2_ros go2_control.launch.py use_stamped:=true
```

### Option C: Custom Configuration
```bash
ros2 launch unitree_go2_ros go2_control.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baudrate:=115200 \
    use_stamped:=false \
    cmd_vel_topic:=cmd_vel \
    max_linear_velocity:=2.0 \
    max_angular_velocity:=1.5
```

## 3. Test the Robot

### Terminal 1: Launch the node
```bash
ros2 launch unitree_go2_ros go2_control.launch.py
```

### Terminal 2: Publish velocity commands
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Stop (Ctrl+C in Terminal 2)
```

### Or use teleop keyboard:
```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 4. Common Commands

```bash
# Check if node is running
ros2 node list

# Check topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check node parameters
ros2 param list /go2_control_node

# Get specific parameter
ros2 param get /go2_control_node serial_port
```

## 5. Troubleshooting

### Serial permission error:
```bash
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### Find your serial port:
```bash
ls /dev/tty* | grep -E "ACM|USB"
```

### Check package installation:
```bash
ros2 pkg list | grep unitree
```
