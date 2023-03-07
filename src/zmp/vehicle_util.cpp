#include <zmp/vehicle_util.hpp>

void VehicleUtil::clear_diff_str()
{
  int i = 0;

  steering_diff_sum = 0;
  for (i = 0; i < steering_diff_buffer.size(); i++) {
    steering_diff_buffer.pop();
  }
}

void VehicleUtil::ZMP_STOP()
{
  hev->SetDrvStroke(0);
  usleep(200000);
  hev->SetBrakeStroke(4095);
  usleep(200000);
}

int VehicleUtil::ZMP_DRV_CONTROLED()
{
  if (_drvInf.mode == 0x10 && _drvInf.servo == 0x10) {
    return 1;
  } else {
    return 0;
  }
}

void VehicleUtil::ZMP_SET_DRV_PROGRAM()
{
  if (_drvInf.mode == 0x00) {
    hev->SetDrvMode(0x10);
    usleep(200000);
    hev->SetDrvMode(CONT_MODE_STROKE);
    usleep(200000);
  }
  if (_drvInf.servo == 0x00) {
    hev->SetDrvServo(0x10);
    usleep(200000);
  }
}

void VehicleUtil::ZMP_SET_DRV_MANUAL()
{
  if (_drvInf.mode == 0x10) {
    hev->SetDrvMode(0x00);
    usleep(200000);
  }
  if (_drvInf.servo == 0x10) {
    hev->SetDrvServo(0x00);
    usleep(200000);
  }
}

int VehicleUtil::ZMP_STR_CONTROLED()
{
  if (_strInf.mode == 0x10 && _strInf.servo == 0x10) {
    return 1;
  } else {
    return 0;
  }
}

void VehicleUtil::ZMP_SET_STR_PROGRAM()
{
  if (_strInf.mode == 0x00) {
    hev->SetStrMode(0x10);
    usleep(200000);
    hev->SetStrMode(CONT_MODE_TORQUE);
    usleep(200000);
  }
  if (_strInf.servo == 0x00) {
    hev->SetStrServo(0x10);
    usleep(200000);
  }
}
void VehicleUtil::ZMP_SET_STR_MANUAL()
{
  if (_strInf.mode == 0x10) {
    hev->SetStrMode(0x00);
    usleep(200000);
  }
  if (_strInf.servo == 0x10) {
    hev->SetStrServo(0x00);
    usleep(200000);
  }
}

void VehicleUtil::ZMP_SET_STR_ANGLE(double angle){
    hev->SetStrAngle(angle);
}

void VehicleUtil::ZMP_SET_STR_TORQUE(double torque){
    hev->SetStrTorque(torque);
}

void VehicleUtil::SetStrMode(int mode)
{
  switch (mode) {
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
  double e;
  static double e_prev = 0xffffffff;
  double e_i;
    double e_d;
    double ret;

    current_steering_angle-= _STEERING_ANGLE_ERROR;
    double steering_diff = cmd_steering_angle - current_steering_angle;
    steering_diff_sum += steering_diff;
    if(steering_diff_sum>_STEERING)
}

void VehicleUtil::SteeringControl(double current_steering_angle, double cmd_steering_angle)
  {
    double torque;
    if (!ZMP_STR_CONTROLED()) {
      clear_diff_str();
    }
    cmd_steering_angle -= _STEERING_ANGLE_ERROR;
    ZMP_SET_STR_ANGLE(cmd_steering_angle*10);

    if (vstate.velocity<3)
    {
      torque = 0;
    }
    else
    {
      torque = _str
    }
    
  }