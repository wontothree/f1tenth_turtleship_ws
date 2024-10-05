#include "local_costmap_generator/local_costmap_generator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::cout <<"Package local_costmap_generator started\n" << std::endl;

  rclcpp::spin(std::make_shared<LocalCostmapGenerator>());
  printf("spin");

  rclcpp::shutdown();
  printf("shutdown");

  return 0;
}
