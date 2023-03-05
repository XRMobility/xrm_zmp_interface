#ifndef MAIN_CNT_H
#define MAIN_CNT_H
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <string>
#include <sys/time.h>
#include <fstream>
#include "HevCnt.h"

struct selectLogInf
{
    bool start;
    bool drvInf;
    bool strInf;
    bool brkInf;
    bool battInf;
    bool otherInf;
};

class MainCnt
{
public:
    MainCnt();
    virtual ~MainCnt();
    // autoware
    //  actuation
    // void SteeringControl(double current_steering_angle, double cmd_steering_angle);
    // void StrokeControl(double current_velocity, double cmd_velocity);
    // void VelocityControl(double current_velocity, double cmd_velocity);

    // cmd
    // static void* CMDGetterEntry(void *a);

    // mode
    // static void* ModeSetterEntry(void *a);
    // void SetStrMode(int mode);
    // void SetDrvMode(int mode);

    // gear
    // static void* GearSetterEntry(void *a);
    // void SetGear(int gear);

    // can
    void SendCAN(void); // can.cpp

    // common
    bool ConfigSocket(void);
    void UpdateState(void);
    void ClearCntDiag(void);

private:
    struct tm *_s_time;
    time_t _day_time;
    timeval _getTime;
    struct selectLogInf _selectLog;
    HevCnt *hev;
    BattInf _battInf;
    BrakeInf _brakeInf;
    OtherInf _otherInf;
    DrvInf _drvInf;
    StrInf _strInf;
    ConfigInf _config;
};
#endif /* MAIN_CNT_H */