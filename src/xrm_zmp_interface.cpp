#include <xrm_zmp_interface/xrm_zmp_interface.hpp>

XrmZmpNode::XrmZmpNode() : Node("xrm_zmp_interface")
{
    // Subcribers
    // From Autoware
    control_cmd_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "control_cmd", 10, std::bind(&XrmZmpNode::callbackControlCmd, this, std::placeholders::_1));

    //Publishers
}

void XrmZmpNode::callbackControlCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
    control_command_received_time_ = this->now();
    control_cmd_ptr_ = msg;
}

void XrmZmpNode::publishCommands()
{
  const rclcpp::Time current_time = get_clock()->now();
}