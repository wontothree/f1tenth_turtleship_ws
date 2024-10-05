# [Package] local_costmap_generator

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

# Publisheed Topics

# Nodes

`local_costmap_generator`

# Dependencies

`rclcpp`, `sensor_msgs`, `laser_geometry`

you shoud install package `laser_geometry`. 이 패키지는 `tf2`에 의존하므로 같이 install해야 한다.

```
sudo apt-get update
sudo apt install ros-foxy-laser-geometry #
sudo apt-get install ros-foxy-laser-geometry
```
