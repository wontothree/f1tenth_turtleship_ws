# F1tenth Turtleship Workspace

This project is for F!TENTH AUTONOMOUS GRAND Prix at CDC 2024.

# Tested Environment

- Native Ubuntu 20.04 (LTS)
- ROS1 Noetic

# Getting Started

```bash
# install dependencies
rosdep install --from-paths src --ignore-src -r -y

# build
catkin_make

# setting environment
source devel/setup.bash
```

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

mppi controller

```bash
roslaunch reference_waypoint_loader reference_waypoint_loader.launch

roslaunch local_costmap_generator local_costmap_generator.launch

roslaunch mppi_controller mppi_controller.launch is_simulation:=false is_localize_less_mode:=false
```

# Acknowledgement

- [[GITHUB] cartographer](https://github.com/cartographer-project/cartographer)
- [[GITHUB] cartopgraher_ros](https://github.com/cartographer-project/cartographer_ros)
- [[GITHUB] als_ros](https://github.com/NaokiAkai/als_ros?tab=readme-ov-file)

using

- [[GITHUB] global_racetrajectory_optimization](https://github.com/TUMFTM/trajectory_planning_helpers)
- [[GITHUB] proj-svg_mppi](https://github.com/kohonda/proj-svg_mppi)
- [[GITHUB] f1tenth_system](https://github.com/ForzaETH/f1tenth_system/tree/56a2bbfc663b2cf3e7275a4d3323ebcbdbf9592b)
