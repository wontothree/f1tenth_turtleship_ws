#!/bin/bash

parallel --ungroup :::\
 "roslaunch mppi_controller mppi_controller.launch"\
 "roslaunch reference_sdf_generator reference_sdf_generator.launch"\
 "roslaunch reference_waypoint_loader reference_waypoint_loader.launch"\
 "roslaunch local_costmap_generator local_costmap_generator.launch"




 



