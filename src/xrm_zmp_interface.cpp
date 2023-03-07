#include <xrm_zmp_interface/xrm_zmp_interface.hpp>

XrmZmpNode::XrmZmpNode() : Node("xrm_zmp_interface")
{
  // Subcribers
  // From Autoware
  control_cmd_sub_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", rclcpp::QoS{1},
      std::bind(&XrmZmpNode::callbackControlCmd, this, std::placeholders::_1));

  gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", rclcpp::QoS{1},
    std::bind(&XrmZmpNode::callbackGearCmd, this, std::placeholders::_1));

  turn_indicators_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "control/command/turn_indicators_cmd", rclcpp::QoS{1},
      std::bind(&XrmZmpNode::callbackTurnIndicatorsCmd, this, std::placeholders::_1));
  
  gate_mode_sub_ = this->create_subscription<tier4_control_msgs::msg::GateMode>(
    "/control/gate_mode", rclcpp::QoS{1},
    std::bind(&XrmZmpNode::callbackGateMode, this, std::placeholders::_1));

  // Publishers
  // To Autoware
  control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "control_mode_report", rclcpp::QoS{1});

  vehicle_twist_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});

  steering_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});

  gear_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});

  turn_indicators_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});

  // Timer
  const auto period_ns = rclcpp::Rate(ZMP_LOOP_RATE).period();
  publish_to_autoware_timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&XrmZmpNode::publishCommands, this));
  get_data_from_vehecle_timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&XrmZmpNode::getDataFromVehicle, this));
}

void XrmZmpNode::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
}

void XrmZmpNode::callbackGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
}

void XrmZmpNode::callbackTurnIndicatorsCmd(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_cmd_ptr_ = msg;
}

void XrmZmpNode::callbackGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  gate_mode_ptr_ = msg;
}

void XrmZmpNode::publishCommands() { const rclcpp::Time current_time = get_clock()->now(); }

void XrmZmpNode::getDataFromVehicle() { const rclcpp::Time current_time = get_clock()->now(); }