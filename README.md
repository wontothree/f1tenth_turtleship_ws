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

# Acknowledgement

- [[GITHUB] cartographer](https://github.com/cartographer-project/cartographer)
- [[GITHUB] cartopgraher_ros](https://github.com/cartographer-project/cartographer_ros)
- [[GITHUB] als_ros](https://github.com/NaokiAkai/als_ros?tab=readme-ov-file)
- [[GITHUB] global_racetrajectory_optimization](https://github.com/TUMFTM/trajectory_planning_helpers)
- [[GITHUB] proj-svg_mppi](https://github.com/kohonda/proj-svg_mppi)
- [[GITHUB] f1tenth_system](https://github.com/f1tenth/f1tenth_system/tree/braking?tab=readme-ov-file)

# Tmp

1. Vesc

2. Joystick

3. Lidar
