# F1tenth Turtleship Workspace

This project is for 22nd F1TENTH AUTONOMOUS GRAND Grand Prix at CDC 2024.

    f1tenth_turtleship_ws
    └── src/
        ├── vesc/                                 # depend
        ├── urg_node2/                            # depend
        ├── grid_map/                             # depend
        ├── joystick/                             #
        ├── local_costmap_generator/              # 
        ├── svg_mppi_planner/                     # 
        ├── mppi_planner/                         # 
        └── pid_planner/                          #

# Tested Environment

- Native Ubuntu 20.04 (LTS)
- ROS2 Foxy

# Getting Started

Clone this repository.

```bash
git clone https://github.com/wontothree/f1tenth_turtleship_ws.git
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

Vesc

```bash
lsusb
ls /dev/input*
ros2 launch vesc_driver vesc_driver_node.launch.py
```

Lidar

```bash
ros2 launch urg_node2 urg_node2.launch.py
ros2 run tf2_ros static_transform_publisher 0.09 0 0 0 0 0 map laser
```

Joystick

```bash
ros2 run joy joy_node
ros2 run joystick joystick_node
```

local costmap generator

```bash
ros2 run tf2_ros static_transform_publisher 0.09 0.0 0.0 0.0 0.0 0.0 "ego_racecar/laser" "ego_racecar/base_link"
```

# Hardware

- Logitech G F710 Wireless Gamepad
- Bldc motor
- Servo motor - steering
- [Hokuyo](https://www.hokuyo-aut.jp/search/single.php?serial=167)
- [IMU](https://www.devicemart.co.kr/goods/view?no=15136719&srsltid=AfmBOoqRikGmc_8O2PogU1WQg-s3Kz6dxdQenrYfrV1s8TG_qI2BBXvy)
- Intel NUC

# Dependencies

I use three dependency packages

https://github.com/Hokuyo-aut/urg_node2/tree/11d02a97a7352af508400d03ed9b0b219a33f1ac

https://github.com/f1tenth/vesc/tree/5b2bd1b3f3896b078e5d91623702e73d62ece7a7

https://github.com/ntrexlab/PC_AHRS_ROS2

