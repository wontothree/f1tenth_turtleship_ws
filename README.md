# F1tenth Turtleship Workspace

This project is for 22nd F1TENTH AUTONOMOUS GRAND Grand Prix at CDC 2024.

![](./icons/f1tenth_cdc_2024_banner.png)


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

## Install Dependencies

```bash
git clone https://github.com/wontothree/f1tenth_turtleship_ws.git

rosdep update
rosdep install --from-paths src --ignore-src -r -y

sudo apt-get update
rosdep update —rosdistro=foxy

sudo apt-get install ros-foxy-laser-proc
sudo apt-get install ros-foxy-cv-bridge
sudo apt-get install ros-foxy-filters
sudo apt-get install ros-foxy-serial-driver
sudo apt-get install ros-foxy-pcl-conversions
sudo apt-get install ros-foxy-pcl-ros
```

## Build

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build --symlink-install
```

## Run Packages in Real System

1. Vesc

```bash
rosdep install -i --from-paths urg_node2

lsusb
ls /dev/input*

ros2 launch vesc_driver vesc_driver_node.launch.py # vesc에 대한 입력 & feedback
```

2. Joystick

```bash
ros2 run joy joy_node            # joystick의 입력을 ros2 topic으로 발행하기 위한 node
ros2 run joystick joystick_node  # joystick의 입력을 vesc를 위한 topic으로 발행하기 위한 node
```

3. Lidar

```bash
ros2 launch urg_node2 urg_node2.launch.py
ros2 run tf2_ros static_transform_publisher 0.09 0 0 0 0 0 map laser
```

4. local_costmap_generator_node

```bash
ros2 run tf2_ros static_transform_publisher 0.09 0.0 0.0 0.0 0.0 0.0 "ego_racecar/laser" "ego_racecar/base_link"
ros2 run local_costmap_generator local_costmap_generator_node
```

# Hardware

- NUC11TNHi7 - (CPU: i7-1165G7 (4.7GHz))
- Logitech G F710 Wireless Gamepad
- Bldc motor
- Servo motor - steering
- [Hokuyo](https://www.hokuyo-aut.jp/search/single.php?serial=167)
- [IMU](https://www.devicemart.co.kr/goods/view?no=15136719&srsltid=AfmBOoqRikGmc_8O2PogU1WQg-s3Kz6dxdQenrYfrV1s8TG_qI2BBXvy)

# Dependencies

I use three dependency packages

- https://github.com/Hokuyo-aut/urg_node2/tree/11d02a97a7352af508400d03ed9b0b219a33f1ac
- https://github.com/f1tenth/vesc/tree/5b2bd1b3f3896b078e5d91623702e73d62ece7a7
- https://github.com/ntrexlab/PC_AHRS_ROS2
