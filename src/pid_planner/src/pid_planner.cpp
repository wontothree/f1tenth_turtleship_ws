#include "pid_planner/pid_planner.hpp"

PidPlanner::PidPlanner() : Node("pid_planner")
{
    // subscriber
    scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PidPlanner::scanCallback, this, std::placeholders::_1)
    );

    // publisher
    commandsServoPositionPublisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);
    commandsMotorSpeedPublisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/speed", 10);
    commandsMotorBrakePublisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/brake", 10);
}

void PidPlanner::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // std::cout << "Ranges:" << std::endl;
    // for (size_t i = 0; i < scan->ranges.size(); ++i) {
    //     std::cout << "  [" << i << "] " << scan->ranges[i] << std::endl;
    // }


    // speed by axes[1]
    std_msgs::msg::Float64 speed_;
    speed_.data = 5000;

    commandsMotorSpeedPublisher_ -> publish(speed_);

    std::cout << "speed: " << speed_.data << std::endl;
}
