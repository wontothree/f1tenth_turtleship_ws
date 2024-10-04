#include "pid_planner/pid_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Package pid_planner started\n" << std::endl;

  rclcpp::spin(std::make_shared<PidPlanner>());

  rclcpp::shutdown();
  return 0;
}
