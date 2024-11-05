#include "joystick/joystick.hpp"

int main(int argc, char * argv [])
{
  rclcpp::init(argc, argv);
  printf("Package joystick started\n");

  rclcpp::spin(std::make_shared<Joystick>());
  printf("spin");

  rclcpp::shutdown();
  printf("shutdown");

  return 0;
}
