# F1tenth Turtleship Workspace

This project is for 22nd F1TENTH AUTONOMOUS GRAND Grand Prix at CDC 2024.

    f1tenth_turtleship_ws
    └── src/
        ├── joystick/                 # 
        ├── vesc/                     # 
        └──                           #

# Tested Environment

- Native Ubuntu 20.04 (LTS)
- ROS2 Foxy

# Getting Started

```bash
git clone https://github.com/wontothree/f1tenth_turtleship_ws.git
```

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build
```

```bash
lsusb
ls /dev/input*

source /opt/ros/foxy/setup.bash
source install/local_setup.bash

ros2 launch vesc_driver vesc_driver_node.launch.py
ros2 run joy joy_node
ros2 run joystick joystick_node
```

# Hardware

- Logitech G F710 Wireless Gamepad
- Bldc motor
- Servo motor - steering
- Hokuyo
