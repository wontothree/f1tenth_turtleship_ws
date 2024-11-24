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

# run mppi controller in physical system
.PHONY: run_physical_system
run_physical_system:
	@tmux new-session -d -s ros_system \; \
	    send-keys "roslaunch reference_sdf_generator reference_sdf_generator.launch" C-m \; \
	    split-window -v \; \
	    send-keys "roslaunch reference_waypoint_loader reference_waypoint_loader.launch" C-m \; \
	    split-window -v \; \
	    send-keys "roslaunch local_costmap_generator local_costmap_generator.launch" C-m \; \
	    split-window -v \; \
	    send-keys "roslaunch mppi_controller mppi_controller.launch is_simulation:=false is_localize_less_mode:=false" C-m \; \
	    attach

