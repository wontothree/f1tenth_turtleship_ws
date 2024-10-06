# F1tenth Turtleship Workspace

This project is for 22nd F1TENTH AUTONOMOUS GRAND Grand Prix at CDC 2024.

    f1tenth_turtleship_ws
    └── src/
        ├── vesc/                                 #
        ├── urg_node2/                            # 
        ├── joystick/                             #
        ├── local_costmap_generator/              # 
        └── pid_planner                           #

# Tested Environment

- Native Ubuntu 20.04 (LTS)
- ROS2 Foxy

# Getting Started

```bash
git clone --recursive https://github.com/wontothree/f1tenth_turtleship_ws.git
```

Install dependencies

```bash
rosdep update
rosdep install -i --from-paths urg_node2
```

Build

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build --symlink-install
```

Logitech Joystick and Vesc

```bash
lsusb
ls /dev/input*

source /opt/ros/foxy/setup.bash
source install/local_setup.bash

ros2 launch vesc_driver vesc_driver_node.launch.py

ros2 run joy joy_node
ros2 run joystick joystick_node

ros2 launch urg_node2 urg_node2.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map laser
```

Hokuyo Lidar

```bash
ifconfig
rosdep update
rosdep install -i --from-paths urg_node2
```

# Hardware

- Logitech G F710 Wireless Gamepad
- Bldc motor
- Servo motor - steering
- [Hokuyo](https://www.hokuyo-aut.jp/search/single.php?serial=167)

# Dependencies

I use two dependency packages

https://github.com/Hokuyo-aut/urg_node2/tree/11d02a97a7352af508400d03ed9b0b219a33f1ac

https://github.com/f1tenth/vesc/tree/5b2bd1b3f3896b078e5d91623702e73d62ece7a7
