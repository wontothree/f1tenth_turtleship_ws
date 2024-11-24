# F1tenth Turtleship Workspace

This project is for F!TENTH AUTONOMOUS GRAND Prix at CDC 2024.

# Tested Environment

- Native Ubuntu 20.04 (LTS)
- ROS1 Noetic

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

# TMP

Joystick

```bash
roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch

rosparam set speed_to_erpm_gain 4614.0
rosparam set speed_to_erpm_offset 0
rosparam set steering_angle_to_servo_gain -1.2135
rosparam set steering_angle_to_servo_offset 0.5304
rosrun vesc_ackermann ackermann_to_vesc_node
roslaunch vesc_ackermann ackermann_to_vesc_node.launch

roslaunch vesc_driver vesc_driver_node.launch port:=/dev/ttyACM0
```
