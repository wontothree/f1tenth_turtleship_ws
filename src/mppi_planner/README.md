# MPPI Local Planner

    mppi_planner
    ├── include
    |   └── mppi_planner
    |      ├── mppi_types.hpp                     # data type definition for mppi
    |      ├── mppi_base.hpp                      # fundamental functions for mppi, no dependencies for ROS
    |      ├── mppi_template.hpp                  # abstract class for mppi implementation, no dependencies for ROS
    |      ├── mppi_planner_ros.hpp               # ROS dependencies
    |      └── mppi_planner.hpp                   # mppi implementation for local planner
    └── src/
        ├── mppi_planner_node.cpp                 #
        ├── mppi_base.cpp                         #
        ├── mppi_planner_ros.cpp                  #
        └── mppi_planner.cpp                      #
