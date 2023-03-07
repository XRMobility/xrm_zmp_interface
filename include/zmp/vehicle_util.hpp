#ifndef vehicle_util_hpp
#define vehicle_util_hpp
#include "HevCnt.h"

#include <vehicle_info.hpp>

#include <iostream>
#include <queue>
#include <string>

static double steering_diff_sum = 0;
std::queue<double> steering_diff_buffer;

class VehicleUtil
{
public:
  VehicleUtil();
  ~VehicleUtil();

private:
  HevCnt * hev;
  BattInf _battInf;
  BrakeInf _brakeInf;
  OtherInf _otherInf;
  DrvInf _drvInf;
  StrInf _strInf;
  ConfigInf _config;
  vehicle_state_t vstate;
  static void clear_diff_str();
  void SetStrMode(int mode);
  void ZMP_STOP();
  int ZMP_DRV_CONTROLED();
  void ZMP_SET_DRV_PROGRAM();
  void ZMP_SET_DRV_MANUAL();
  int ZMP_STR_CONTROLED();
  void ZMP_SET_STR_PROGRAM();
  void ZMP_SET_STR_MANUAL();
  void ZMP_SET_STR_ANGLE(double angle);
  void ZMP_SET_STR_TORQUE(double torque);
  void SteeringControl(double current_steering_angle, double cmd_steering_angle);
  double _str_torque_pid_control(double current_steering_angle, double cmd_steering_angle);
  
};

#endif  // vehicle_util_hpp