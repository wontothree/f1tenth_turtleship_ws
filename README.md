# F1tenth Turtleship Workspace

This project is for 22nd F1TENTH AUTONOMOUS GRAND Grand Prix at CDC 2024.

    f1tenth_turtleship_ws
    └── src/
        ├── joystick/                 # 
        ├── vesc/                     # 
        └── lidar/                    *

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

# Hardware

- Logitech G F710 Wireless Gamepad
- Bldc motor
- Servo motor - steering
- Hokuyo
