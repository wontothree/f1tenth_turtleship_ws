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

    // PID Gain
    double kp = 3.0;
    double ki = 0.0;
    double kd = 0.1;
    double prev_error = 0.0;
    double integral = 0.0;
    double desired_distance_to_wall = 1.0; // 원하는 벽과의 거리

public:
    PidPlanner();

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    double get_range(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double angle);

    double calculate_error(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    void pid_control(double error);
};
