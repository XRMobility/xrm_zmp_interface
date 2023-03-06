#ifndef XRM_ZMP_INTERFACE_HPP
#define XRM_ZMP_INTERFACE_HPP
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

class XrmZmpNode : public rclcpp::Node
{
public:
  XrmZmpNode();

private:
  // time stamps
  rclcpp::Time control_command_received_time_;
  rclcpp::Time actuation_command_received_time_;
  rclcpp::Time last_shift_inout_matched_time_;

  // input value
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;

  // Subcribers
  // From Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicators_cmd_sub_;

  // Publishers
  // To Autoware

  // Callbacks
  void callbackControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void callbackGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void callbackTurnIndicatorsCmd(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);

  // Functions
  void publishCommands();
};
#endif  // XRM_ZMP_INTERFACE_HPP