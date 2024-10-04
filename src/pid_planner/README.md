# [Package] pid_planner

# Subscribed Topics

`/scan` [sensor_msgs/msg/LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)

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

# Published Topics

- `/commands/motor/speed` (std_msgs::msg::Float64) : $[-23250, 23250]$
- `/commands/motor/brake` (std_msgs::msg::Float64) : $[-20000, 200000]$
- `/commands/servo/position` (std_msgs::msg::Float64) : $[0.15, 0.85]$

# Nodes

`pid_planner_node`

# Dependencies

`rclcpp`, `std_msgs`
