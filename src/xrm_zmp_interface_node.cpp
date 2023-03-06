#include <xrm_zmp_interface/xrm_zmp_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XrmZmpNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}