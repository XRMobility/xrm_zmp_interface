#include <xrm_zmp_interface/xrm_zmp_interface.hpp>

XrmZmpNode::XrmZmpNode() : Node("xrm_zmp_interface")
{

  vehicle_util_ = new VehicleUtil();
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
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
    "/vehicle/status/control_mode", rclcpp::QoS{1});

  vehicle_twist_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});

  steering_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});

  gear_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});

  turn_indicators_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});

  steering_wheel_status_pub_ =
    this->create_publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>(
      "/vehicle/status/steering_wheel_status", rclcpp::QoS{1});

  actuation_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
    "/vehicle/status/actuation_status", rclcpp::QoS{1});

  door_status_pub_ = this->create_publisher<tier4_api_msgs::msg::DoorStatus>(
    "/vehicle/status/door_status", rclcpp::QoS{1});

  // Timer
  const auto period_ns = rclcpp::Rate(ZMP_LOOP_RATE).period();
  publish_to_autoware_timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&XrmZmpNode::publishCommands, this));
  get_data_from_vehecle_timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&XrmZmpNode::sendDataToVehicle, this));
}

void XrmZmpNode::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  // control_cmd_ptr_ = msg;
  static long long int old_tstamp = 0;
  vehicle_util_->cycle_time= (vehicle_util_->vstate.tstamp-old_tstamp)/1000.0;

  double current_velocity = vehicle_util_->vstate.velocity; //km/h
  double current_steering_angle = vehicle_util_->vstate.steering_angle; //deg
  int cmd_velocity = msg->longitudinal.speed * 3.6; //m/s -> km/h
  int cmd_steering_angle;
  if (msg->longitudinal.speed<0.1) {
    cmd_steering_angle=current_steering_angle;
  } else {
    double wheel_angle = vehicle_util_->RadToDeg(msg->lateral.steering_tire_angle);
    cmd_steering_angle = wheel_angle*WHEEL_TO_STEERING;
  }
  for (int i = 0; i < vehicle_util_->cmd_rx_interval/STEERING_INTERNAL_PERIOD-1; i++) {
    vehicle_util_->StrokeControl(current_velocity,cmd_velocity);
    vehicle_util_->SteeringControl(current_steering_angle,cmd_steering_angle);
    vehicle_util_->UpdateInfo();
    current_velocity = vehicle_util_->vstate.velocity;
    current_steering_angle = vehicle_util_->vstate.steering_angle;
  }
  vehicle_util_->StrokeControl(current_velocity,cmd_velocity);
  vehicle_util_->SteeringControl(current_steering_angle,cmd_steering_angle);
  old_tstamp = vehicle_util_->vstate.tstamp;
}

void XrmZmpNode::callbackGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  // gear_cmd_ptr_ = msg;
  int current_gear = vehicle_util_->getGear();
  vehicle_util_->gear_is_setting = true;
  if (
    (msg->command == autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE && current_gear == 1) ||
    (msg->command == autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL && current_gear == 2) ||
    (msg->command == autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE && current_gear == 3)) {
    return;
  } else if (msg->command == autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE) {
    vehicle_util_->SetGear(CMD_GEAR_R);
  } else if (msg->command == autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE) {
    vehicle_util_->SetGear(CMD_GEAR_D);
  } else if (msg->command == autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL) {
    vehicle_util_->SetGear(CMD_GEAR_N);
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown gear command");
  }
  vehicle_util_->gear_is_setting = false;
}

void XrmZmpNode::callbackTurnIndicatorsCmd(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  // turn_indicators_cmd_ptr_ = msg;
  if (msg->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE) {
    vehicle_util_->SetBlinkerLeftOFF();
    vehicle_util_->SetBlinkerRightOFF();
  } else if (msg->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT) {
    vehicle_util_->SetBlinkerLeftON();
    vehicle_util_->SetBlinkerRightOFF();
  } else if (msg->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT) {
    vehicle_util_->SetBlinkerRightON();
    vehicle_util_->SetBlinkerLeftOFF();
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown turn indicators command");
  }
}

void XrmZmpNode::callbackGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  // gate_mode_ptr_ = msg;
  vehicle_util_->mode_is_setting = true;
  vehicle_util_->ClearCntDiag();
  sleep(1);
  if (
    (msg->data == tier4_control_msgs::msg::GateMode::AUTO && vehicle_util_->ZMP_DRV_CONTROLED()==1 &&
     vehicle_util_->ZMP_STR_CONTROLED()==1) ||
    (msg->data == tier4_control_msgs::msg::GateMode::EXTERNAL &&
     vehicle_util_->ZMP_DRV_CONTROLED()==0 && vehicle_util_->ZMP_STR_CONTROLED()==0)) {
    return;
  } else if (msg->data == tier4_control_msgs::msg::GateMode::EXTERNAL) {
    vehicle_util_->SetManualMode();
  } else if (msg->data == tier4_control_msgs::msg::GateMode::AUTO) {
    vehicle_util_->SetProgramMode();
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown gate mode");
  }
  vehicle_util_->mode_is_setting = false;
}

void XrmZmpNode::publishCommands()
{
  const rclcpp::Time current_time = get_clock()->now();
  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = current_time;

  // Publish control mode
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
  control_mode_msg.stamp = current_time;
  if (vehicle_util_->ZMP_DRV_CONTROLED()==1 && vehicle_util_->ZMP_STR_CONTROLED()==1) {
    control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  } else if (vehicle_util_->ZMP_DRV_CONTROLED()==0 && vehicle_util_->ZMP_STR_CONTROLED()==0) {
    control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
  } else if (vehicle_util_->ZMP_DRV_CONTROLED()==0 && vehicle_util_->ZMP_STR_CONTROLED()==1) {
    control_mode_msg.mode =
      autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_STEER_ONLY;
  } else if (vehicle_util_->ZMP_DRV_CONTROLED()==1 && vehicle_util_->ZMP_STR_CONTROLED()==0) {
    control_mode_msg.mode =
      autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;
  }
  control_mode_pub_->publish(control_mode_msg);

  // Publish vehicle twist
  double current_velocity =
    VehicleUtil::Kmh100ToMs(vehicle_util_->getVelocity());  // current vehicle velocity [m/s]
  const double current_steer_wheel = VehicleUtil::DegToRad(
    vehicle_util_->getSteeringAngle() / 10);  // current vehicle steering wheel angle [rad]
  const double adaptive_gear_ratio =
    vehicle_util_->calculateVariableGearRatio(current_velocity, current_steer_wheel);
  const double current_steer = current_steer_wheel / adaptive_gear_ratio;
  autoware_auto_vehicle_msgs::msg::VelocityReport vehicle_twist_msg;
  vehicle_twist_msg.header = header;
  vehicle_twist_msg.longitudinal_velocity = current_velocity;
  vehicle_twist_msg.heading_rate =
    current_velocity * std::tan(current_steer) / WHEEL_BASE;  // [rad/s]
  vehicle_twist_pub_->publish(vehicle_twist_msg);

  // Publish steering status
  double steering_tire_angle = current_steer_wheel / WHEEL_TO_STEERING;
  autoware_auto_vehicle_msgs::msg::SteeringReport steering_status_msg;
  steering_status_msg.stamp = current_time;
  steering_status_msg.steering_tire_angle = current_steer;
  steering_status_pub_->publish(steering_status_msg);

  // Publish gear status
  autoware_auto_vehicle_msgs::msg::GearReport gear_status_msg;
  gear_status_msg.stamp = current_time;
  if (vehicle_util_->getGear() == 1) {
    gear_status_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
  } else if (vehicle_util_->getGear() == 2) {
    gear_status_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NONE;
  } else if (vehicle_util_->getGear() == 3) {
    gear_status_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::REVERSE;
  } else {
    gear_status_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NONE;
  }
  gear_status_pub_->publish(gear_status_msg);

  // Publish turn indicators status
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators_status_msg;
  turn_indicators_status_msg.stamp = current_time;
  if (vehicle_util_->getBlinkerLeft()) {
    turn_indicators_status_msg.report =
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
  } else if (vehicle_util_->getBlinkerRight()) {
    turn_indicators_status_msg.report =
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
  } else {
    turn_indicators_status_msg.report =
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
  }
  turn_indicators_status_pub_->publish(turn_indicators_status_msg);

  //  Publish actuation status
  tier4_vehicle_msgs::msg::ActuationStatusStamped actuation_status_msg;
  actuation_status_msg.header = header;
  actuation_status_msg.status.accel_status = vehicle_util_->getCurrentAccel();
  actuation_status_msg.status.brake_status = vehicle_util_->getCurrentBrake();
  actuation_status_msg.status.steer_status = current_steer;
  actuation_status_pub_->publish(actuation_status_msg);

  // Publish steering wheel status
  tier4_vehicle_msgs::msg::SteeringWheelStatusStamped steering_wheel_status_msg;
  steering_wheel_status_msg.stamp = current_time;
  steering_wheel_status_msg.data = current_steer_wheel;
  steering_wheel_status_pub_->publish(steering_wheel_status_msg);

  // Publish door status
  tier4_api_msgs::msg::DoorStatus door_status_msg;
  if (vehicle_util_->getDoorStatus()) {
    door_status_msg.status = door_status_msg.DOOR_CLOSED;
  } else {
    door_status_msg.status = door_status_msg.DOOR_OPENED;
  }
  door_status_pub_->publish(door_status_msg);
}

void XrmZmpNode::sendDataToVehicle() { const rclcpp::Time current_time = get_clock()->now(); }