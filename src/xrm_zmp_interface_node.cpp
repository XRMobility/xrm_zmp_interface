#include <xrm_zmp_interface/xrm_zmp_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv)
{
  std::cout << "xrm_zmp_interface_node" << std::endl;
  rclcpp::init(argc, argv);
  std::cout << "xrm_zmp_interface_node init" << std::endl;
  auto node = std::make_shared<XrmZmpNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}