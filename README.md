# F1tenth Turtleship Workspace

This project is for F!TENTH AUTONOMOUS GRAND Prix at CDC 2024.

# Tested Environment

- Native Ubuntu 20.04 (LTS)
- ROS1 Noetic

# Getting Started

```
# Usage: make [command]
SHELL := /bin/bash
setup:
	# install dependencies for ROS 	
	rosdep install -i --from-path src --rosdistro noetic -y
	sudo apt update
	sudo apt install -y ros-noetic-map-server python3-catkin-tools libomp-dev ros-noetic-jsk-rviz-plugins mpv

# build without simulator with ROS
.PHONY: build
build:
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_GPU=OFF

clean:
	# clean build files
	rm -rf build devel logs .catkin_tools install

```

```
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_GPU=OFF

./script/launch_simulator.sh

./script/launch_controllers.sh 
```
