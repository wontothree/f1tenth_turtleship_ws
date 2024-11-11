#include "svg_mppi_planner/svg_mppi_planner_ros.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "package svg_mppi_planner started\n";

  rclcpp::spin(std::make_shared<svg_mppi::SVGMPPIPlannerROS>());;

  rclcpp::shutdown();
  std::cout << "shutdown" << std::endl;

  return 0;
}
