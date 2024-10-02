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

    // publisher
    commandMotorSpeedPublisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/current", 10);
    
    // 설정한 주기로 speed 값 publish
    auto publish_speed = [this]() -> void {
        std_msgs::msg::Float64 msg;
        msg.data = 10;  // 일정한 motor speed 값을 설정 (예: 1000)
        commandMotorSpeedPublisher_->publish(msg);
        std::cout << "publishing commands/motor/current = 10" << std::endl;
    };

    // 100ms (0.1초) 주기로 publish
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), publish_speed);
}

void Joystick::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg)
{
    // std::cout << "Received Joy Message:" << std::endl;
    // std::cout << "Axes: ";
    // for (const auto& axis : joyMsg->axes) {
    //     std::cout << axis << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "Buttons: ";
    // for (const auto& button : joyMsg->buttons) {
    //     std::cout << button << " ";
    // }
    // std::cout << std::endl;

    std::cout << "Received Joy Message:" << std::endl;
    
    // Create Twist message
    auto twistMsg = geometry_msgs::msg::Twist();

    // 예: 조이스틱의 축 값에 따라 선형 속도 및 각속도 설정
    twistMsg.linear.x = joyMsg->axes[1];  // 왼쪽 조이스틱 상하 움직임
    twistMsg.angular.z = joyMsg->axes[0]; // 왼쪽 조이스틱 좌우 움직임

    // 퍼블리시
    twistPublisher_->publish(twistMsg);

    // 축과 버튼 정보 출력
    std::cout << "Axes: ";
    for (const auto& axis : joyMsg->axes) {
        std::cout << axis << " ";
    }
    std::cout << std::endl;

    std::cout << "Buttons: ";
    for (const auto& button : joyMsg->buttons) {
        std::cout << button << " ";
    }
    std::cout << std::endl;
}
