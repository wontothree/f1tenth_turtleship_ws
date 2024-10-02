#pragma once

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"       // for Joy message
#include "geometry_msgs/msg/twist.hpp"   // for Twist message
#include <std_msgs/msg/float64.hpp>

class Joystick : public rclcpp::Node {

public:
    Joystick();

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_;   // subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher_;   // publisher

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr commandMotorSpeedPublisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};
