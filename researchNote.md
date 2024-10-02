# Research Note

*2024.09.28*

#1 workspace 

I made ros2 worksapce for f1tenth.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
colcon build
```

#2 Turtlesim

I will control turtle using keyborad direction in turtlesim.

```bash
sudo apt update
sudo apt install ros-foxy-turtlesim

ros2 pkg executables turtlesim

ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

- package : turtlesim
- node : turtlesim_node, turtle_teleop_key

turtle_teleop_key

- input : user keyboard direction input
- output : '/turtle1/cmd_vel'

You can see rqt graph

```bash
rqt_graph
```

#3 Joystick

You can find connected device to your computer using the following command

```bash
lsusb

ls /dev/input*
```

Useful app for joystick - jstest-gtk

```bash
sudo apt install joystick jstest-gtk ros-kinetic-joy
sudo apt purge ros-kinetic-teleop-twist-joy
cd ~/catkin_ws/src
git clone https://github.com/robotpilot/teleop_twist_joy.git
cd ~/catkin_ws && catkin_make
sudo apt install jstest-gtk
jstest-gtk
```

This repository provides transform from '/joy' to 'cmd_vel'. The package comes with the teleop_node that republishes sensor_msgs/msg/Joy messages as scaled geometry_msgs/msg/Twist messages.

[teleop_twist_joy foxy](https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/)

```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

Insert the usb of joystic to my computer. You can check if input of joystic is received.

```bash
ros2 topic echo \joy

ros2 topic echo cmd_vel
```

Though input of joystic control angular z, it can not change the other values. I think button map is not correct.

```bash
---
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.0
---
```

Unfortunately, this repository doesn't provide interface for my joystick - F7 10 logitect

https://github.com/husarion/joy2twist/blob/ros2/joy2twist/package.xml

*2024.09.29*

Let's start from turtlesim. What is prinple that I can control turtle using 'turtle_teleop_key' node in turtlesim?

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

```bash
ros2 node list
// ...
/teleop_turtle
/turtlesim
```

```bash
ros2 topic list
// ...
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

```bash
ros2 topic echo /turtle1/cmd_vel
// ...
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

When I press upper direction in keyboard, x = 2.0 in linear.

When I press down direction in keyboard, x = -2.0 in linear.

When I press right direction in keyboard, z = -2.0 in angular.

When I press left direction in keyboard, z = 2.0 in angular.

In order to control turtle in turtlesim by joystick, I should transform input of joystick to /turtle1/cmd/vel (geometry_msgs/msg/Twist)

I conclude that it is more fast to myself make package for f7 10 jostick.

1. Input of joystick F7 10 to '/joy' (sensor_msgs/Joy)  
2. '/joy' (sensor_msgs/Joy) to '/cmd_vel' (geometry_msgs/Twist)

Process 1 is possible for ROS2 standard package. 

```bash
ros2 run joy joy_node
```

I shoud develop package for process 2 - joystick_f710_controller

I get many help from [GITHUB - joy2twist](https://github.com/husarion/joy2twist/tree/ros2/joy2twist/include/joy2twist)

https://github.com/f1tenth/f1tenth_system

*2024.10.01*

What do I need to do to control vesc driver

#4 Vesc

```bash
/commands/motor/brake # *

/commands/motor/current
/commands/motor/duty_cycle 
/commands/motor/position # 우리 motor가 hall sensor less라 측정할 수 없다.
/commands/motor/speed # v

/commands/servo/position # *

/sensors/core
/sensors/imu
/sensors/imu/raw
/sensors/servo_position_command
```

*2024.10.02*

The object that i shoud do

```bash
/commands/motor/brake
/commands/servo/position
```

I wonder What is difference between `/commands/motor/duty_cycle` and `/commands/motor/speed`. Is it sufficient to give one among the supposing fuor topics? I will use speed control at first.

```bash
/commands/motor/current
/commands/motor/duty_cycle
/commands/motor/position
/commands/motor/speed
```

just memo. Why do it work only when i run this node? To understand this mechanism, i need to know how to work [this node](https://github.com/f1tenth/vesc/tree/1952e799110f5c4eed82c68a6172bfcafd9998ac).

```bash
ros2 launch vesc_driver vesc_driver_node.launch.py
```

solution

