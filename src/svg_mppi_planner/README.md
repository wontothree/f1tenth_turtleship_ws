# [Package] svg_mppi_planner

    local_costmap_generator
    ├── include
    |   └── svg_mppi_planner
    |      ├── common.hpp                         # 
    |      ├── svg_mppi_planner_ros.hpp           # 
    |      └── svg_mppi.hpp                       #
    └── src/
        ├── svg_mppi_planner_node.cpp             # 
        ├── svg_mppi_planner_ros.cpp              # 
        └── svg_mppi.cpp                          #

# Subscribed Topics

```cpp
rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr cost_map_subscriber_;
cost_map_subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
    "cost_map",
    10, 
    std::bind(&SVGMPPIPlannerROS::local_cost_map_callback, this, std::placeholders::_1)
);
```

`ego_racecar/odom` [nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)

```
std_msgs/msg/Header header
string child_frame_id
geometry_msgs/msg/PoseWithCovariance pose
geometry_msgs/msg/TwistWithCovariance twist
```

# Published Topics

```cpp
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "candidate_paths",
    10
);
```

# Node

`svg_mppi_planner_node`