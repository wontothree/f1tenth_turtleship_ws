#include "mppi_planner/mppi_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Package mppi_planner started\n";

  rclcpp::spin(std::make_shared<MPPIPlanner>());

  rclcpp::shutdown();
  std::cout << "Shutdown" << std::endl;

  return 0;
}
