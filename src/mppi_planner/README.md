# MPPI Local Planner

    mppi_planner
    ├── include
    |   └── mppi_planner
    |      ├── mppi_types.hpp                     # data type definition for mppi
    |      ├── mppi_template.hpp                  # abstract class for mppi implementation, no dependencies for ROS
    |      ├── mppi_base.hpp                      # fundamental functions for mppi, no dependencies for ROS
    |      ├── mppi_planner_ros.hpp               # ROS dependencies
    |      └── mppi.hpp                           # mppi implementation using 'mppi_template.hpp'
    └── src/
        ├── mppi_base.cpp                         # 
        ├── mppi_planner_node.cpp                 #
        ├── mppi_planner_ros.cpp                  #
        └── mppi.cpp                              #

# Dependencies

'grid_map_ros'