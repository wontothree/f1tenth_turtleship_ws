# [Package] joystick

The purpose of this package is to provide a generic facility for tele-operating Twist-based ROS2 robots with joystick [**Logitech G F710 Wireless Gamepad**](https://www.amazon.com/Logitech-Wireless-Nano-Receiver-Controller-Vibration/dp/B0041RR0TW?th=1). It converts joy messages to velocity commands.

The packages comes with the `teleop_node` that republishes `sensor_msgs/msg/Joy` messages as scaled `geometry_msgs/msg/Twist` messages.

# Subscribed Topics

Joystick messages to be translated to velocity commands.

`/joy` (sensor_msgs/msg/Joy)

```
float32[] axes
int32[] buttons
```

```bash
header:
  stamp:
    sec: 1727591787
    nanosec: 304888379
  frame_id: joy
axes:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
buttons:
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
```

# Published Topics

Command velocity messages arising from Joystick commands.

- `/cmd_vel` (geometry_msgs/msg/Twist) just for test in turtlesim

```
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
```

```bash
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

- `/commands/motor/speed` (std_msgs::msg::Float64) : $[-23250, 23250]$
- `/commands/motor/brake` (std_msgs::msg::Float64) : $[-20000, 200000]$
- `/commands/servo/position` (std_msgs::msg::Float64) : $[0.15, 0.85]$

# Nodes

`joystick_node`

# Dependencies

`rclcpp`, `std_msgs`, `geometry_msgs`

# To do

- 급정지
- Teleoperation or navigation mode switching
