#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pcl_converter.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLConverter>());
  rclcpp::shutdown();

  return 0;
}
