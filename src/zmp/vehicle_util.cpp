#include <zmp/vehicle_util.hpp>

double VehicleUtil::steering_diff_sum = 0.0;
double VehicleUtil::estimate_accel = 0.0;
int VehicleUtil::target_accel_level = 0;
double VehicleUtil::accel_diff_sum = 0.0;
double VehicleUtil::brake_diff_sum = 0.0;
std::queue<double> VehicleUtil::steering_diff_buffer;
std::queue<double> VehicleUtil::accel_diff_buffer;
std::queue<double> VehicleUtil::brake_diff_buffer;

VehicleUtil::VehicleUtil()
{
  hev = new HevCnt();
  hev->Init();
  hev->Start();
  std::cout << "VehicleUtil::VehicleUtil() LOG: HevCnt initialized" << std::endl;
  // read_thread = std::thread(&VehicleUtil::readLoop, this);
  // read_thread.detach();
  hev->SetDrvMode(MODE_MANUAL);
  hev->SetStrMode(MODE_MANUAL);
}

void VehicleUtil::clear_diff_str()
{
  std::cout << "VehicleUtil::clear_diff_str() LOG: Clear steering diff" << std::endl;
  int i = 0;

  steering_diff_sum = 0;
  for (i = 0; i < steering_diff_buffer.size(); i++)
  {
    steering_diff_buffer.pop();
  }
}

void VehicleUtil::ZMP_STOP()
{
  std::cout << "VehicleUtil::ZMP_STOP() LOG: Stop vehicle" << std::endl;
  hev->SetDrvStroke(0);
  usleep(200000);
  hev->SetBrakeStroke(4095);
  usleep(200000);
}

int VehicleUtil::ZMP_DRV_CONTROLED()
{
  std::cout << "VehicleUtil::ZMP_DRV_CONTROLED() LOG: Check if vehicle is controlled" << std::endl;
  if (_drvInf.mode == 0x10 && _drvInf.servo == 0x10)
  {
    std::cout << "VehicleUtil::ZMP_DRV_CONTROLED() LOG: Vehicle is controlled" << std::endl;
    return 1;
  }
  else
  {
    std::cout << "VehicleUtil::ZMP_DRV_CONTROLED() LOG: Vehicle is not controlled" << std::endl;
    return 0;
  }
}

void VehicleUtil::ZMP_SET_DRV_PROGRAM()
{
  std::cout << "VehicleUtil::ZMP_SET_DRV_PROGRAM() LOG: Set vehicle to program mode" << std::endl;
  if (_drvInf.mode == 0x00)
  {
    hev->SetDrvMode(0x10);
    usleep(200000);
    hev->SetDrvMode(CONT_MODE_STROKE);
    usleep(200000);
  }
  if (_drvInf.servo == 0x00)
  {
    hev->SetDrvServo(0x10);
    usleep(200000);
  }
}

void VehicleUtil::ZMP_SET_DRV_MANUAL()
{
  std::cout << "VehicleUtil::ZMP_SET_DRV_MANUAL() LOG: Set vehicle to manual mode" << std::endl;
  if (_drvInf.mode == 0x10)
  {
    hev->SetDrvMode(0x00);
    usleep(200000);
  }
  if (_drvInf.servo == 0x10)
  {
    hev->SetDrvServo(0x00);
    usleep(200000);
  }
}

int VehicleUtil::ZMP_STR_CONTROLED()
{
  std::cout << "VehicleUtil::ZMP_STR_CONTROLED() LOG: Check if steering is controlled" << std::endl;
  if (_strInf.mode == 0x10 && _strInf.servo == 0x10)
  {
    std::cout << "VehicleUtil::ZMP_STR_CONTROLED() LOG: Steering is controlled" << std::endl;
    return 1;
  }
  else
  {
    std::cout << "VehicleUtil::ZMP_STR_CONTROLED() LOG: Steering is not controlled" << std::endl;
    return 0;
  }
}

void VehicleUtil::ZMP_SET_STR_PROGRAM()
{
  std::cout << "VehicleUtil::ZMP_SET_STR_PROGRAM() LOG: Set steering to program mode" << std::endl;
  if (_strInf.mode == 0x00)
  {
    hev->SetStrMode(0x10);
    usleep(200000);
    hev->SetStrMode(CONT_MODE_TORQUE);
    usleep(200000);
  }
  if (_strInf.servo == 0x00)
  {
    hev->SetStrServo(0x10);
    usleep(200000);
  }
}
void VehicleUtil::ZMP_SET_STR_MANUAL()
{
  std::cout << "VehicleUtil::ZMP_SET_STR_MANUAL() LOG: Set steering to manual mode" << std::endl;
  if (_strInf.mode == 0x10)
  {
    hev->SetStrMode(0x00);
    usleep(200000);
  }
  if (_strInf.servo == 0x10)
  {
    hev->SetStrServo(0x00);
    usleep(200000);
  }
}

void VehicleUtil::ZMP_SET_STR_ANGLE(double angle)
{
  std::cout << "VehicleUtil::ZMP_SET_STR_ANGLE() LOG: Set steering angle to " << angle << std::endl;
  hev->SetStrAngle(angle);
}

void VehicleUtil::ZMP_SET_STR_TORQUE(double torque)
{
  std::cout << "VehicleUtil::ZMP_SET_STR_TORQUE() LOG: Set steering torque to " << torque << std::endl;
  hev->SetStrTorque(torque);
}

void VehicleUtil::SetStrMode(int mode)
{
  std::cout << "VehicleUtil::SetStrMode() LOG: Set steering mode to " << mode << std::endl;
  switch (mode)
  {
    case CMD_MODE_MANUAL:
      std::cout << "Manual mode(Steering)" << std::endl;
      ZMP_SET_STR_MANUAL();
      break;
    case CMD_MODE_PROGRAM:
      std::cout << "Program mode(Steering)" << std::endl;
      ZMP_SET_STR_PROGRAM();
      clear_diff_str();
      break;
    default:
      std::cout << "Invalid mode(Steering)" << std::endl;
      break;
  }
}

double VehicleUtil::_str_torque_pid_control(double current_steering_angle, double cmd_steering_angle)
{
  std::cout << "VehicleUtil::_str_torque_pid_control() LOG: Steering torque PID control" << std::endl;
  double e;
  static double e_prev = 0xffffffff;
  double e_i;
  double e_d;
  double ret;

  current_steering_angle -= _STEERING_ANGLE_ERROR;
  double steering_diff = cmd_steering_angle - current_steering_angle;
  steering_diff_sum += steering_diff;
  if (steering_diff_sum > _STEERING_MAX_SUM)
  {
    steering_diff_sum = _STEERING_MAX_SUM;
  }

  if (steering_diff_sum < -_STEERING_MAX_SUM)
  {
    steering_diff_sum = -_STEERING_MAX_SUM;
  }

  e = steering_diff;
  e_i = steering_diff_sum;

  if (e_prev == 0xffffffff)
  {
    e_prev = e;
  }

  e_d = (e - e_prev) / (STEERING_INTERNAL_PERIOD / 1000.0);

  double k_p = _K_STEERING_P;
  double k_i = _K_STEERING_I;
  double k_d = _K_STEERING_D;

  double steering_max_torque = _STEERING_MAX_TORQUE;
  static double targert_steering_torque = 0;

  if (fabs(e) < 0.3 && fabs(cmd_steering_angle < 3 && vstate.velocity < 10))
  {
    steering_diff_sum = 0;
    if (e > 0)
    {
      targert_steering_torque = 10;
    }
    else
    {
      targert_steering_torque = -10;
    }
  }
  else
  {
    targert_steering_torque = e * k_p + e_i * k_i + e_d * k_d;
  }

  if (targert_steering_torque > steering_max_torque)
  {
    targert_steering_torque = steering_max_torque;
  }
  else if (targert_steering_torque < -steering_max_torque)
  {
    targert_steering_torque = -steering_max_torque;
  }
  ret = targert_steering_torque;
  e_prev = e;
  return ret;
}

void VehicleUtil::SteeringControl(double current_steering_angle, double cmd_steering_angle)
{
  std::cout << "VehicleUtil::SteeringControl() LOG: Steering control" << std::endl;
  double torque;
  if (!ZMP_STR_CONTROLED())
  {
    clear_diff_str();
  }
  cmd_steering_angle -= _STEERING_ANGLE_ERROR;
  ZMP_SET_STR_ANGLE(cmd_steering_angle * 10);

  if (vstate.velocity < 3)
  {
    torque = 0;
  }
  else
  {
    torque = _str_torque_pid_control(current_steering_angle, cmd_steering_angle);
  }
  ZMP_SET_STR_ANGLE(torque);
}

void VehicleUtil::clear_diff_drv()
{
  std::cout << "VehicleUtil::clear_diff_drv() LOG: Clear driving difference" << std::endl;
  int i;
  accel_diff_sum = 0;
  brake_diff_sum = 0;
  for (i = 0; i < (int)accel_diff_buffer.size(); i++)
  {
    accel_diff_buffer.pop();
  }
  for (i = 0; i < (int)brake_diff_buffer.size(); i++)
  {
    brake_diff_buffer.pop();
  }
}

void VehicleUtil::SetDrvMode(int mode)
{
  std::cout << "VehicleUtil::SetDrvMode() LOG: Set driving mode to " << mode << std::endl;
  switch (mode)
  {
    case CMD_MODE_MANUAL:
      std::cout << "Manual mode(Driving)" << std::endl;
      ZMP_SET_DRV_MANUAL();
      break;
    case CMD_MODE_PROGRAM:
      std::cout << "Program mode(Driving)" << std::endl;
      ZMP_SET_DRV_PROGRAM();
      clear_diff_drv();
      break;
    default:
      std::cout << "Invalid mode(Driving)" << std::endl;
      break;
  }
}

void VehicleUtil::ZMP_SET_SHIFT_POS_D()
{
  std::cout << "VehicleUtil::ZMP_SET_SHIFT_POS_D() LOG: Set shift position to D" << std::endl;
  hev->SetDrvShiftMode(SHIFT_POS_D);
}
void VehicleUtil::ZMP_SET_SHIFT_POS_N()
{
  std::cout << "VehicleUtil::ZMP_SET_SHIFT_POS_N() LOG: Set shift position to N" << std::endl;
  hev->SetDrvShiftMode(SHIFT_POS_N);
}
void VehicleUtil::ZMP_SET_SHIFT_POS_R()
{
  std::cout << "VehicleUtil::ZMP_SET_SHIFT_POS_R() LOG: Set shift position to R" << std::endl;
  hev->SetDrvShiftMode(SHIFT_POS_R);
}
void VehicleUtil::ZMP_SET_SHIFT_POS_B()
{
  std::cout << "VehicleUtil::ZMP_SET_SHIFT_POS_B() LOG: Set shift position to B" << std::endl;
  hev->SetDrvShiftMode(SHIFT_POS_B);
}

void VehicleUtil::SetGear(int gear)
{
  std::cout << "VehicleUtil::SetGear() LOG: Set gear to " << gear << std::endl;
  double current_velocity = vstate.velocity;  // km/h
  if (current_velocity != 0)
  {
    std::cout << "Vehicle is moving. Can't change gear" << std::endl;
    return;
  }
  ZMP_STOP();
  switch (gear)
  {
    case CMD_GEAR_D:
      std::cout << "Set Gear D" << std::endl;
      ZMP_SET_SHIFT_POS_D();
      break;
    case CMD_GEAR_N:
      std::cout << "Set Gear N" << std::endl;
      ZMP_SET_SHIFT_POS_N();
      break;
    case CMD_GEAR_R:
      std::cout << "Set Gear R" << std::endl;
      ZMP_SET_SHIFT_POS_R();
      break;
    case CMD_GEAR_B:
      std::cout << "Set Gear B" << std::endl;
      ZMP_SET_SHIFT_POS_B();
      break;
    default:
      std::cout << "Invalid gear" << std::endl;
      break;
  }
  sleep(1);
}

double VehicleUtil::_accel_stroke_pid_control(double current_velocity, double cmd_velocity)
{
  std::cout << "VehicleUtil::_accel_stroke_pid_control() LOG: Acceleration control" << std::endl;
  double e;
  static double e_prev = 0;
  double e_i;
  double e_d;
  double ret;

  if (vstate.brake_stroke > _BRAKE_PEDAL_OFFSET)
  {
    ret = 0;
    e_prev = 0;
  }
  else
  {
    double target_accel_stroke;
    e = cmd_velocity - current_velocity;
    e_d = e - e_prev;
    accel_diff_sum += e;
    if (accel_diff_sum > _ACCEL_MAX_I)
    {
      e_i = _ACCEL_MAX_I;
    }
    else
    {
      e_i = accel_diff_sum;
    }
    if (current_velocity > 15)
    {
      target_accel_stroke = _K_ACCEL_P_UNTIL20 * e + _K_ACCEL_I_UNTIL20 * e_i + _K_ACCEL_D_UNTIL20 * e_d;
    }
    else
    {
      target_accel_stroke = _K_ACCEL_P_UNTIL10 * e + _K_ACCEL_I_UNTIL10 * e_i + _K_ACCEL_D_UNTIL10 * e_d;
    }
    if (target_accel_stroke > _ACCEL_PEDAL_MAX)
    {
      target_accel_stroke = _ACCEL_PEDAL_MAX;
    }
    else if (target_accel_stroke < 0)
    {
      target_accel_stroke = 0;
    }
    ret = target_accel_stroke;
    e_prev = e;
  }
  return ret;
}

double VehicleUtil::_brake_stroke_pid_control(double current_velocity, double cmd_velocity)
{
  double e;
  static double e_prev = 0;
  double e_i;
  double e_d;
  double ret;

  if (vstate.accel_stroke > _ACCEL_PEDAL_OFFSET)
  {
    ret = 0;
    e_prev = 0;
  }
  else
  {
    double target_brake_stroke;
    e = -1 * (cmd_velocity - current_velocity);
    if (e > 0 && e <= 1)
    {
      e = 0;
    }
    e_d = e - e_prev;
    brake_diff_sum += e;
    if (brake_diff_sum > _BRAKE_MAX_I)
    {
      e_i = _BRAKE_MAX_I;
    }
    else
    {
      e_i = brake_diff_sum;
    }
    target_brake_stroke = _K_BRAKE_P * e + _K_BRAKE_I * e_i + _K_BRAKE_D * e_d;
    if (target_brake_stroke > _BRAKE_PEDAL_MAX)
    {
      target_brake_stroke = _BRAKE_PEDAL_MAX;
    }
    else if (target_brake_stroke < 0)
    {
      target_brake_stroke = 0;
    }
    ret = target_brake_stroke;
    e_prev = e;
  }
  return ret;
}

double VehicleUtil::_stopping_control(double current_velocity)
{
  std::cout << "VehicleUtil::_stopping_control() LOG: Stopping control" << std::endl;
  double ret;
  static double old_brake_stroke = _BRAKE_PEDAL_STOPPING_MED;
  if (current_velocity < 0.1)
  {
    int gain = (int)(((double)_BRAKE_PEDAL_STOPPING_MAX) * cycle_time);
    ret = old_brake_stroke + gain;
    if ((int)ret > _BRAKE_PEDAL_STOPPING_MAX)
    {
      ret = _BRAKE_PEDAL_STOPPING_MAX;
    }
  }
  else
  {
    ret = _BRAKE_PEDAL_STOPPING_MED;
    old_brake_stroke = _BRAKE_PEDAL_STOPPING_MED;
  }
  return ret;
}

long long int VehicleUtil::getTime()
{
  std::cout << "VehicleUtil::getTime() LOG: Get time" << std::endl;
  struct timeval current_time;
  struct timezone ttz;
  double t;
  gettimeofday(&current_time, &ttz);
  t = (current_time.tv_sec * 1000.0) + (current_time.tv_usec) / 1000.0;
  return static_cast<long long int>(t);
}

void VehicleUtil::UpdateInfo()
{
  std::cout << "VehicleUtil::UpdateInfo() LOG: Update information" << std::endl;
  hev->GetBattInf(&_battInf);
  hev->GetDrvInf(&_drvInf);
  hev->GetBrakeInf(&_brakeInf);
  hev->GetOtherInf(&_otherInf);
  hev->GetStrInf(&_strInf);
  hev->UpdateBattState(REP_BATT_INFO);
  hev->UpdateBattState(REP_BATT_INFO_CURRENT);
  hev->UpdateBattState(REP_BATT_INFO_VOLT);
  hev->UpdateDriveState(REP_DRV_MODE);
  hev->UpdateDriveState(REP_GAS_PEDAL);
  hev->UpdateDriveState(REP_GAS_PEDAL_FROMOBD);
  hev->UpdateDriveState(REP_VELOCITY);
  hev->UpdateDriveState(REP_VELOCITY_FROMOBD);
  hev->UpdateDriveState(REP_VELOCITY_FROMOBD2);
  hev->UpdateDriveState(REP_WHEEL_VELOCITY_F);
  hev->UpdateDriveState(REP_WHEEL_VELOCITY_R);
  hev->UpdateDriveState(REP_BRAKE_PEDAL);
  hev->UpdateDriveState(REP_BRAKE_PEDAL_FROMOBD);
  hev->UpdateDriveState(REP_SHIFT_POS);
  hev->UpdateDriveState(REP_SHIFT_POS_FROMOBD);
  hev->UpdateDriveState(REP_HEV_MODE);
  hev->UpdateDriveState(REP_ICE_RPM);
  hev->UpdateDriveState(REP_ICE_COOLANT_TEMP);
  hev->UpdateDriveState(REP_ACCELERLATION);
  hev->UpdateDriveState(REP_SIDE_ACCELERLATION);
  hev->UpdateDriveState(REP_DRIVE_MODE);
  hev->UpdateDriveState(REP_CRUISE_STATE);
  hev->UpdateDriveState(REP_DTC_STATUS);
  hev->UpdateDriveState(REP_BRAKE_STATUS);
  hev->UpdateOtherState(REP_LIGHT_STATE);
  hev->UpdateOtherState(REP_DOOR_STATE);
  hev->UpdateOtherState(REP_GAS_LEVEL);
  vstate.accel_stroke = _drvInf.actualPedalStr;
  vstate.brake_stroke = _brakeInf.actualPedalStr;
  vstate.velocity = _drvInf.veloc;
  vstate.tstamp = (long long int)(getTime());
  vstate.steering_angle = _strInf.angle;
  vstate.steering_torque = _strInf.torque;
}

void VehicleUtil::readLoop()
{
  std::cout << "VehicleUtil::readLoop() LOG: Read loop" << std::endl;
  while (true)
  {
    UpdateInfo();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

float VehicleUtil::getVelocity()
{
  std::cout << "VehicleUtil::getVelocity() LOG: Get velocity" << vstate.velocity << std::endl;
  return vstate.velocity;
}

double VehicleUtil::KmhToMs(double kmh)
{
  return (kmh * 1000 / (60.0 * 60.0));
}

double VehicleUtil::Kmh100ToMs(double kmh100)
{
  return (kmh100 * 10 / (60.0 * 60.0));
}

double VehicleUtil::RadToDeg(double rad)
{
  return (rad * 180.0 / M_PI);
}
double VehicleUtil::DegToRad(double deg)
{
  return (deg * M_PI / 180.0);
}

double VehicleUtil::getSteeringAngle()
{
  std::cout << "VehicleUtil::getSteeringAngle() LOG: Get steering angle" << vstate.steering_angle << std::endl;
  return vstate.steering_angle;
}

double VehicleUtil::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  std::cout << "VehicleUtil::calculateVariableGearRatio() LOG: Calculate variable gear ratio" << std::endl;
  return std::max(1e-5, VGR_COEF_A + VGR_COEF_B * vel * vel - VGR_COEF_C * std::fabs(steer_wheel));
}

int VehicleUtil::getGear()
{
  std::cout << "VehicleUtil::getGear() LOG: Get gear" << std::endl;
  if (_drvInf.actualShift == SHIFT_POS_D)
  {
    std::cout << "VehicleUtil::getGear() LOG: Gear is D" << std::endl;
    return 1;
  }
  else if (_drvInf.actualShift == SHIFT_POS_N)
  {
    std::cout << "VehicleUtil::getGear() LOG: Gear is N" << std::endl;
    return 2;
  }
  else if (_drvInf.actualShift == SHIFT_POS_R)
  {
    std::cout << "VehicleUtil::getGear() LOG: Gear is R" << std::endl;
    return 3;
  }
  else if (_drvInf.actualShift == SHIFT_POS_B)
  {
    std::cout << "VehicleUtil::getGear() LOG: Gear is B" << std::endl;
    return 4;
  }
  else
  {
    std::cout << "VehicleUtil::getGear() LOG: Gear is ?" << std::endl;
    return 0;
  }
}

bool VehicleUtil::getBlinkerLeft()
{
  std::cout << "VehicleUtil::getBlinkerLeft() LOG: Get blinker left" << std::endl;
  if (_brakeInf.blinkerLeft == 1)
  {
    std::cout << "VehicleUtil::getBlinkerLeft() LOG: Blinker left is on" << std::endl;
    return true;
  }
  else
  {
    std::cout << "VehicleUtil::getBlinkerLeft() LOG: Blinker left is off" << std::endl;
    return false;
  }
}
bool VehicleUtil::getBlinkerRight()
{
  std::cout << "VehicleUtil::getBlinkerRight() LOG: Get blinker right" << std::endl;
  if (_brakeInf.blinkerRight == 1)
  {
    std::cout << "VehicleUtil::getBlinkerRight() LOG: Blinker right is on" << std::endl;
    return true;
  }
  else
  {
    std::cout << "VehicleUtil::getBlinkerRight() LOG: Blinker right is off" << std::endl;
    return false;
  }
}

float VehicleUtil::getCurrentAccel()
{
  std::cout << "VehicleUtil::getCurrentAccel() LOG: Get current accel"
            << (float)_drvInf.actualPedalStr / (float)_ACCEL_PEDAL_MAX << std::endl;
  return (float)_drvInf.actualPedalStr / (float)_ACCEL_PEDAL_MAX;
}
float VehicleUtil::getCurrentBrake()
{
  std::cout << "VehicleUtil::getCurrentBrake() LOG: Get current brake"
            << (float)_brakeInf.actualPedalStr / (float)_BRAKE_PEDAL_MAX << std::endl;
  return (float)_brakeInf.actualPedalStr / (float)_BRAKE_PEDAL_MAX;
}

bool VehicleUtil::getDoorStatus()
{
  std::cout << "VehicleUtil::getDoorStatus() LOG: Get door status" << std::endl;
  if (_otherInf.door == DOOR_CLOSE)
  {
    std::cout << "VehicleUtil::getDoorStatus() LOG: Door is closed" << std::endl;
    return true;
  }
  else
  {
    std::cout << "VehicleUtil::getDoorStatus() LOG: Door is open" << std::endl;
    return false;
  }
}

void VehicleUtil::SetBlinkerLeftON()
{
  std::cout << "VehicleUtil::SetBlinkerLeftON() LOG: Set blinker left on" << std::endl;
  hev->SetBlinkerLeft(1);
}
void VehicleUtil::SetBlinkerRightON()
{
  std::cout << "VehicleUtil::SetBlinkerRightON() LOG: Set blinker right on" << std::endl;
  hev->SetBlinkerRight(1);
}

void VehicleUtil::SetBlinkerLeftOFF()
{
  std::cout << "VehicleUtil::SetBlinkerLeftOFF() LOG: Set blinker left off" << std::endl;
  hev->SetBlinkerLeft(0);
}
void VehicleUtil::SetBlinkerRightOFF()
{
  std::cout << "VehicleUtil::SetBlinkerRightOFF() LOG: Set blinker right off" << std::endl;
  hev->SetBlinkerRight(0);
}

void VehicleUtil::SetManualMode()
{
  std::cout << "VehicleUtil::SetManualMode() LOG: Set manual mode" << std::endl;
  hev->SetBrakeMode(0x00);
  usleep(200000);
  SetDrvMode(CMD_MODE_MANUAL);
  SetStrMode(CMD_MODE_MANUAL);
}
void VehicleUtil::SetProgramMode()
{
  std::cout << "VehicleUtil::SetProgramMode() LOG: Set program mode" << std::endl;
  SetDrvMode(CMD_MODE_PROGRAM);
  SetStrMode(CMD_MODE_PROGRAM);
  hev->SetBrakeMode(0x10);
  usleep(200000);
}

void VehicleUtil::sndBrkLampOn()
{
  std::cout << "VehicleUtil::sndBrkLampOn() LOG: Set brake lamp on" << std::endl;
  hev->SetBrakeLamp(1);
}
void VehicleUtil::sndBrkLampOff()
{
  std::cout << "VehicleUtil::sndBrkLampOff() LOG: Set brake lamp off" << std::endl;
  hev->SetBrakeLamp(0);
}

void VehicleUtil::StrokeControl(double current_velocity, double cmd_velocity)
{
  std::cout << "VehicleUtil::StrokeControl() LOG: Stroke control" << std::endl;
  static queue<double> vel_buffer;
  static uint vel_buffer_size = 10;
  double old_velocity = 0.0;

  if (ZMP_DRV_CONTROLED() == 0)
  {
    clear_diff_drv();
#ifdef USE_BRAKE_LAMP
    sndBrkLampOff();
#endif
    return;
  }

  vel_buffer.push(current_velocity);
  if (vel_buffer.size() > vel_buffer_size)
  {
    old_velocity = vel_buffer.front();
    vel_buffer.pop();
    estimate_accel = (current_velocity - old_velocity) / (cycle_time * vel_buffer_size);
    std::cout << "estimate_accel:" << estimate_accel << std::endl;
    if (fabs(cmd_velocity) > current_velocity && fabs(cmd_velocity) > 0.0 && current_velocity < SPEED_LIMIT)
    {
      double accel_stroke;
      std::cout << "accelerate: current_velocity:" << current_velocity << " cmd_velocity:" << cmd_velocity << std::endl;
      accel_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity);
      if (accel_stroke > 0.0)
      {
        std::cout << "ZMP_SET_DRV_STROKE(" << accel_stroke << ")" << std::endl;
        hev->SetDrvStroke(accel_stroke);
        ZMP_SET_BRAKE_STROKE(0);
      }
      else
      {
        std::cout << "ZMP_SET_DRV_STROKE(0)" << std::endl;
        hev->SetDrvStroke(0);
        std::cout << "ZMP_SET_BRAKE_STROKE(" << -accel_stroke << ")" << std::endl;
        ZMP_SET_BRAKE_STROKE(-accel_stroke);
      }
    }
    else if (fabs(cmd_velocity) < current_velocity && fabs(cmd_velocity) > 0.0)
    {
      double brake_stroke;
      std::cout << "decelerate: current_velocity:" << current_velocity << " cmd_velocity:" << cmd_velocity << std::endl;
      brake_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity);
      if (brake_stroke > 0)
      {
        std::cout << "ZMP_SET_DRV_STROKE(" << brake_stroke << ")" << std::endl;
        ZMP_SET_BRAKE_STROKE(brake_stroke);
        hev->SetDrvStroke(0);
      }
      else
      {
        std::cout << "ZMP_SET_DRV_STROKE(0)" << std::endl;
        ZMP_SET_BRAKE_STROKE(0);
        std::cout << "ZMP_SET_BRAKE_STROKE(" << -brake_stroke << ")" << std::endl;
        hev->SetDrvStroke(-brake_stroke);
      }
    }
    else if (cmd_velocity == 0.0 && current_velocity != 0.0)
    {
      double brake_stroke;
      std::cout << "stop: current_velocity:" << current_velocity << " cmd_velocity:" << cmd_velocity << std::endl;
      brake_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity);
      if (current_velocity < 4.0)
      {
        ZMP_SET_BRAKE_STROKE(0);
        brake_stroke = _stopping_control(current_velocity);
        std::cout << "ZMP_SET_DRV_STROKE(" << brake_stroke << ")" << std::endl;
        ZMP_SET_BRAKE_STROKE(brake_stroke);
      }
      else
      {
        brake_stroke = _brake_stroke_pid_control(current_velocity, 0);
        if (brake_stroke > 0)
        {
          std::cout << "ZMP_SET_DRV_STROKE(" << brake_stroke << ")" << std::endl;
          ZMP_SET_BRAKE_STROKE(brake_stroke);
        }
        else
        {
          std::cout << "ZMP_SET_DRV_STROKE(0)" << std::endl;
          ZMP_SET_BRAKE_STROKE(0);
          std::cout << "ZMP_SET_BRAKE_STROKE(" << -brake_stroke << ")" << std::endl;
          ZMP_SET_BRAKE_STROKE(-brake_stroke);
        }
      }
    }
    std::cout << "current_velocity:" << current_velocity << " cmd_velocity:" << cmd_velocity << std::endl;
  }
}

void VehicleUtil::ClearCntDiag()
{
  std::cout << "VehicleUtil::ClearCntDiag() LOG: Clear cnt diag" << std::endl;
  hev->ClearCntDiag();
}