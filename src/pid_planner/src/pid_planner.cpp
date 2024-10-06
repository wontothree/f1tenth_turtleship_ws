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
    double error = calculate_error(scan); // 오류 계산
    pid_control(error); // PID 제어
}

double PidPlanner::get_range(const sensor_msgs::msg::LaserScan::SharedPtr scan, double angle)
{
    // LiDAR의 특정 각도에 대한 거리 측정값을 반환하는 헬퍼 함수
    assert(angle >= scan->angle_min && angle <= scan->angle_max);
    
    int i = static_cast<int>((angle - scan->angle_min) / scan->angle_increment);
    
    // 인덱스가 유효한지 확인
    if (i < 0 || i >= scan->ranges.size()) {
        RCLCPP_WARN(this->get_logger(), "Index out of bounds: %d", i);
        return scan->range_max; // 인덱스가 유효하지 않은 경우 최대 거리 반환
    }
    
    // 거리 값이 유효한지 확인
    if (std::isnan(scan->ranges[i]) || scan->ranges[i] > scan->range_max) {
        return scan->range_max; // 유효하지 않은 거리일 경우 최대 거리 반환
    }
    
    return scan->ranges[i];
}

double PidPlanner::calculate_error(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // 왼쪽 벽에 대한 오차를 계산합니다.
    double a = get_range(scan, -50.0 * M_PI / 180.0); // -50도
    double b = get_range(scan, -90.0 * M_PI / 180.0); // -90도
    double theta = 40.0 * M_PI / 180.0; // 40도

    // 계산 전에 값이 유효한지 확인
    if (std::isnan(a) || std::isnan(b)) {
        RCLCPP_WARN(this->get_logger(), "Invalid range values: a=%f, b=%f", a, b);
        return desired_distance_to_wall; // 유효하지 않은 경우 기본 거리 반환
    }

    double alpha = std::atan((a * std::cos(theta) - b) / (a * std::sin(theta)));
    double D_t = b * std::cos(alpha);

    return desired_distance_to_wall - D_t;
}

void PidPlanner::pid_control(double error)
{
    // PID 제어를 구현합니다.
    integral += error; // 적분 값
    double derivative = error - prev_error; // 미분 값
    double angle = kp * error + ki * integral + kd * derivative;

    // 서보 위치와 모터 속도를 명령합니다.
    std_msgs::msg::Float64 servo_position;
    servo_position.data = -1 * angle;

    // 모터 속도는 항상 일정하게 설정하거나 PID 제어의 출력에 따라 조정합니다.
    std_msgs::msg::Float64 speed_;
    speed_.data = 2000.0; // 차량이 이동하는 기본 속도

    // 오류가 클 경우 속도를 줄여서 안전하게 회전하도록 합니다.
    // if (std::abs(error) > 0.5) {
    //     speed_.data = 500; // 큰 오류 시 느리게 이동
    // }

    commandsServoPositionPublisher_->publish(servo_position);
    commandsMotorSpeedPublisher_->publish(speed_);

    std::cout << "Servo Position: " << servo_position.data << ", Speed: " << speed_.data << std::endl;

    prev_error = error; // 이전 오차 업데이트
}
