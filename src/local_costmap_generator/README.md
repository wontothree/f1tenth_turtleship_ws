# [Package] local_costmap_generator

    local_costmap_generator
    ├── include
    |   └── mppi_planner
    |      ├── local_costmap_generator.hpp        # 
    └── src/
        ├── local_costmap_generator_node.cpp      # 
        └── local_costmap_generator.cpp           #

# Subscribed Topics

`/scan` [sensor_msgs/msg/LserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)

```
std_msgs/msg/Header header
float angle_min
float angle_max
float angle_increment
float time_increment
float scan_time
float range_min
float range_max
float[] ranges
float[] intensities
```

```cpp
scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&LocalCostmapGenerator::scanCallback, this, std::placeholders::_1)
);
```

# Publisheed Topics

```cpp
// publisher for costmap
costmapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "cost_map", 10
);
occupancyGridPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "occupancy_grid_map", 10
);
```

# Nodes

`local_costmap_generator_node`

# Dependencies

`rclcpp`, `sensor_msgs`, `laser_geometry`, `pcl_conversions`, `pcl_ros`, `grid_map`

you shoud install package `laser_geometry`. 이 패키지는 `tf2`에 의존하므로 같이 install해야 한다.

```
sudo apt-get update
sudo apt install ros-foxy-laser-geometry
sudo apt-get install ros-foxy-laser-geometry
```

pcl_ros

```bash
sudo apt update
sudo apt install libpcl-dev
sudo apt install ros-foxy-pcl-ros
```

```bash
sudo apt install ros-foxy-tf2-tools
```

```bash
sudo apt-get install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations
```

```bash
sudo apt-get install ros-foxy-filters
```

# Used Message

경로 생성에서는 field가 핵심적으로 사용된다.

`/PointCloud2` [sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

```
std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
```
