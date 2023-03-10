#ifndef vehicle_util_hpp
#define vehicle_util_hpp
#include "zmp/HevCnt.h"

#include <zmp/vehicle_info.hpp>

#include <chrono>
#include <iostream>
#include <queue>
#include <string>
#include <thread>

class VehicleUtil
{
public:
  VehicleUtil();
  ~VehicleUtil();
  HevCnt * hev;
  BattInf _battInf;
  BrakeInf _brakeInf;
  OtherInf _otherInf;
  DrvInf _drvInf;
  StrInf _strInf;
  vehicle_state_t vstate;
  double cycle_time;
  int cmd_rx_interval = 100;  // ms

  static double steering_diff_sum;
  static double estimate_accel;
  static int target_accel_level;
  static double accel_diff_sum;
  static double brake_diff_sum;
  static std::queue<double> steering_diff_buffer;
  static std::queue<double> accel_diff_buffer;
  static std::queue<double> brake_diff_buffer;

  int current_mode = -1;
  int current_gear = -1;
  int mode_is_setting = false;
  int gear_is_setting = false;

  void ZMP_STOP();
  void readLoop();
  static long long int getTime();
  static double KmhToMs(double kmh);
  static double Kmh100ToMs(double kmh100);
  static double RadToDeg(double rad);
  static double DegToRad(double deg);
  double calculateVariableGearRatio(const double vel, const double steer_wheel);
  void ClearCntDiag();
  void UpdateInfo();

  // streering
  static void clear_diff_str();
  void SetStrMode(int mode);
  int ZMP_STR_CONTROLED();
  void ZMP_SET_STR_PROGRAM();
  void ZMP_SET_STR_MANUAL();
  void ZMP_SET_STR_ANGLE(double angle);
  void ZMP_SET_STR_TORQUE(double torque);
  void SteeringControl(double current_steering_angle, double cmd_steering_angle);
  double _str_torque_pid_control(double current_steering_angle, double cmd_steering_angle);

  // drive
  static void clear_diff_drv();
  void SetDrvMode(int mode);
  int ZMP_DRV_CONTROLED();
  void ZMP_SET_DRV_PROGRAM();
  void ZMP_SET_DRV_MANUAL();
  void SetGear(int gear);
  void ZMP_SET_SHIFT_POS_D();
  void ZMP_SET_SHIFT_POS_N();
  void ZMP_SET_SHIFT_POS_R();
  void ZMP_SET_SHIFT_POS_B();
  double _accel_stroke_pid_control(double current_velocity, double cmd_velocity);
  double _brake_stroke_pid_control(double current_velocity, double cmd_velocity);
  double _stopping_control(double current_velocity);
  void StrokeControl(double current_velocity, double cmd_velocity);

  std::thread read_thread;

  // Setter
  void SetBlinkerLeftON();
  void SetBlinkerRightON();
  void SetBlinkerLeftOFF();
  void SetBlinkerRightOFF();
  void SetManualMode();
  void SetProgramMode();
  void sndBrkLampOn();
  void sndBrkLampOff();

  // Getter
  float getVelocity();
  double getSteeringAngle();
  int getGear();
  bool getBlinkerLeft();
  bool getBlinkerRight();

  float getCurrentBrake();
  float getCurrentAccel();
  bool getDoorStatus();
};

#endif  // vehicle_util_hpp