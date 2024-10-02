#include "joystick/joystick.hpp"

Joystick::Joystick() : Node("joystick_node")
{
    // subscriber
    joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Joystick::joyCallback, this, std::placeholders::_1)
    );

    // publisher
    twistPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "turtle1/cmd_vel", 10
    );

    // publishers for vesc control
    commandsServoPositionPublisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);
    commandsMotorSpeedPublisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/speed", 10);
    commandsMotorBrakePublisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/brake", 10);
}

void Joystick::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg)
{
    std::cout << "Received Joy Message:" << std::endl;

    // brake by open button[4]
    std_msgs::msg::Float64 brake_;
    brake_.data = (joyMsg->buttons[4]) * 200000;
    commandsMotorBrakePublisher_ -> publish(brake_);

    // steering by axes[3]
    std_msgs::msg::Float64 steering_;
    steering_.data = (-(joyMsg->axes[3]) * 0.35 + 0.5);

    // speed by axes[1]
    std_msgs::msg::Float64 speed_;
    speed_.data = (joyMsg->axes[1] * 5000);

    if (brake_.data != 0)
    {
        commandsServoPositionPublisher_ -> publish(steering_);
        commandsMotorSpeedPublisher_ -> publish(speed_);
    }

    std::cout << "steering: " << steering_.data << std::endl;
    std::cout << "speed: " << speed_.data << std::endl;
    std::cout << "brake: " << brake_.data << std::endl;
}
