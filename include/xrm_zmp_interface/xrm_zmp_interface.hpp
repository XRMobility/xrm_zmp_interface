#ifndef XRM_ZMP_INTERFACE_HPP
#define XRM_ZMP_INTERFACE_HPP
#include <rclcpp/rclcpp.hpp>
#include <zmp/vehicle_info.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

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
  tier4_control_msgs::msg::GateMode::ConstSharedPtr gate_mode_ptr_;

  // Subcribers
  // From Autoware.Control
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  //From Autoware.Planning
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicators_cmd_sub_;
  rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::SharedPtr gate_mode_sub_;

  // Publishers
  // To Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;

  rclcpp::TimerBase::SharedPtr publish_to_autoware_timer_;
  rclcpp::TimerBase::SharedPtr get_data_from_vehecle_timer_;

  // Callbacks
  void callbackControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void callbackGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void callbackTurnIndicatorsCmd(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void callbackGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg);
  
  // Functions
  void publishCommands();
  void getDataFromVehicle();
};
#endif  // XRM_ZMP_INTERFACE_HPP