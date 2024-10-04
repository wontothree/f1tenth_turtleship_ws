#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_msgs/msg/float64.hpp>

class PidPlanner : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr commandsServoPositionPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr commandsMotorSpeedPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr commandsMotorBrakePublisher_;

public:
    PidPlanner();

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};
