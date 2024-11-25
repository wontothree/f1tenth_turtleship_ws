# F1tenth Turtleship Workspace

This project is for F1TENTH AUTONOMOUS GRAND Prix at CDC 2024.

    f1tenth_turtleship_ws
    └── src/
        ├── vesc/                                 # 
        ├── urg_node2/                            # 
        ├── grid_map/                             # 
        ├── joystick/                             #
        ├── local_costmap_generator/              # 
        ├── svg_mppi_planner/                     # 
        ├── mppi_planner/                         # 
        └── pid_planner/                          #

# Tested Environment

## Software

- Native Ubuntu 20.04 (LTS)
- ROS1 Noetic

## Hardware

- NUC11TNHi7 : CPU: i7-1165G7 (4.7GHz)
- Logitech G F710 Wireless Gamepad
- Bldc motor : for driving
- Servo motor : for steering
- [Hokuyo UST-10LX](https://hokuyo-usa.com/products/lidar-obstacle-detection/ust-10lx) : 2D LiDAR, 270 degree field-of-view, up to 10m, 25ms scan speed
- [IMU](https://www.devicemart.co.kr/goods/view?no=15136719&srsltid=AfmBOoqRikGmc_8O2PogU1WQg-s3Kz6dxdQenrYfrV1s8TG_qI2BBXvy)

# Getting Started

## Simulation

```bash
# install dependencies
cd f1tenth_turtleship_ws
make setup

# build controllers
cd f1tenth_turtleship_ws
make build

# launch simulator in the Docker container
cd f1tenth_turtleship_ws
./script/launch_simulator.sh

# launch controllers in another terminal
cd f1tenth_turtleship_ws
./script/launch_controllers.sh 
```

## Physical System

```bash
roscore

# install dependencies
rosdep install --from-paths src --ignore-src -r -y

# build (or catkin_make, catkin_make_isolated)
make build

# setting environment
source devel/setup.bash
```

race stack

```bash
roslaunch stack_master mapping.launch map_name:=1 racecar_version:=NUC2
roslaunch stack_master base_system.launch map_name:=1 racecar_version:=NUC2
```

mppi controller

```bash
roslaunch reference_sdf_generator reference_sdf_generator.launch
roslaunch reference_waypoint_loader reference_waypoint_loader.launch
roslaunch local_costmap_generator local_costmap_generator.launch
roslaunch mppi_controller mppi_controller.launch is_simulation:=false is_localize_less_mode:=false
```

# Acknowledgement

Standing on the shoulders of giants

- [proj-svg_mppi](https://github.com/kohonda/proj-svg_mppi)
- [race_stack](https://github.com/ForzaETH/race_stack/tree/main)
