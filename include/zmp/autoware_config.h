#ifndef __AUTOWARE_CONFIG_H__
#define __AUTOWARE_CONFIG_H__

// zmp 参数
#define ZMP_CLASS HevCnt
// 定义了清除计数器诊断信息的函数调用
#define ZMP_CLEAR_CNT_DIAG() hev->ClearCntDiag()

// 检查转向电机是否被控制
#define ZMP_STR_CONTROLLED() \
    ((_strInf.mode == 0x10 && _strInf.servo == 0x10) ? 1 : 0)

#define ZMP_DRV_CONTROLLED() \
    ((_drvInf.mode == 0x10 && _drvInf.servo == 0x10) ? 1 : 0)

// 定义了更新状态的函数调用
#define ZMP_UPDATE_STATE()            \
    {                                 \
        hev->GetBrakeInf(&_brakeInf); \
        hev->GetDrvInf(&_drvInf);     \
        hev->GetStrInf(&_strInf);     \
    }

// 获取加速踏板行程的函数调用
#define ZMP_ACCEL_STROKE() _drvInf.actualPedalStr
// 获取刹车踏板行程的函数调用
#define ZMP_BRAKE_STROKE() _brakeInf.actualPedalStr
// 获取转向扭矩的函数调用
#define ZMP_STEERING_TORQUE() _strInf.torque
// 获取转向角度的函数调用
#define ZMP_STEERING_ANGLE() _strInf.angle
// 获取速度的函数调用
#define ZMP_VELOCITY() _drvInf.veloc
// 设置驾驶电机速度的函数调用
#define ZMP_SET_DRV_VELOC(x) hev->SetDrvVeloc((x))
// 设置驾驶电机行程的函数调用
#define ZMP_SET_DRV_STROKE(x) hev->SetDrvStroke((x))
// 是否使用刹车灯 设置刹车行程的函数调用，使用 hev->SetBrakeStroke(x)，如果刹车行程大于 0，刹车灯就会被打开。
#define USE_BRAKE_LAMP
#ifdef USE_BRAKE_LAMP
#define ZMP_SET_BRAKE_STROKE(x)   \
    do                            \
    {                             \
        hev->SetBrakeStroke((x)); \
        if ((x) > 0)              \
        {                         \
            sndBrkLampOn();       \
        }                         \
        else                      \
        {                         \
            sndBrkLampOff();      \
        }                         \
    } while (0)
#else
#define ZMP_SET_BRAKE_STROKE(x) hev->SetBrakeStroke((x))
#endif /* USE_BRAKE_LAMP */

#define ZMP_SET_STR_TORQUE(x) hev->SetStrTorque((x))
#define ZMP_SET_STR_ANGLE(x) hev->SetStrAngle((x))

#define ZMP_SET_SHIFT_POS_D() hev->SetDrvShiftMode(SHIFT_POS_D)
#define ZMP_SET_SHIFT_POS_R() hev->SetDrvShiftMode(SHIFT_POS_R)
#define ZMP_SET_SHIFT_POS_B() hev->SetDrvShiftMode(SHIFT_POS_B)
#define ZMP_SET_SHIFT_POS_N() hev->SetDrvShiftMode(SHIFT_POS_N)

#define ZMP_STOP()                 \
    {                              \
        hev->SetDrvStroke(0);      \
        usleep(200000);            \
        hev->SetBrakeStroke(4095); \
        usleep(200000);            \
    }

#define ZMP_SET_DRV_MANUAL()        \
    {                               \
        if (_drvInf.mode == 0x10)   \
        {                           \
            hev->SetDrvMode(0x00);  \
            usleep(200000);         \
        }                           \
        if (_drvInf.servo == 0x10)  \
        {                           \
            hev->SetDrvServo(0x00); \
            usleep(200000);         \
        }                           \
    }

#define ZMP_SET_DRV_PROGRAM()                   \
    {                                           \
        if (_drvInf.mode == 0x00)               \
        {                                       \
            hev->SetDrvMode(0x10);              \
            usleep(200000);                     \
            hev->SetDrvCMode(CONT_MODE_STROKE); \
            usleep(200000);                     \
        }                                       \
        if (_drvInf.servo == 0x00)              \
        {                                       \
            hev->SetDrvServo(0x10);             \
            usleep(200000);                     \
        }                                       \
    }
// 将转向电机切换到手动模式；
#define ZMP_SET_STR_MANUAL()       \
    {                              \
        if (_strInf.mode == 0x10)  \
        {                          \
            hev->SetStrMode(0x00); \
            usleep(200000);        \
        }                          \
        if (_strInf.servo == 0x10) \
        {
hev->SetStrServo(0x00);
usleep(200000);
}
}
// 将转向电机切换到程序控制模式，并设置控制模式为力矩模式。
#define ZMP_SET_STR_PROGRAM()                   \
    {                                           \
        if (_strInf.mode == 0x00)               \
        {                                       \
            hev->SetStrMode(0x10);              \
            usleep(200000);                     \
            hev->SetStrCMode(CONT_MODE_TORQUE); \
            usleep(200000);                     \
        }                                       \
        if (_strInf.servo == 0x00)              \
        {                                       \
            hev->SetStrServo(0x10);             \
            usleep(200000);                     \
        }                                       \
    }

// prius parameters
#define WHEEL_BASE 2.7                                           // tire-to-tire size of Prius.//车轮轴距的大小
#define WHEEL_ANGLE_MAX 31.28067                                 // max angle of front tires. //前轮转向角度的最大值
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX / WHEEL_ANGLE_MAX) // 转向角度与前轮转向角度的比值
#define STEERING_ANGLE_MAX 666                                   // max angle of steering  转向角度的最大值
#define STEERING_ANGLE_LIMIT 550                                 // could be STEERING_ANGLE_MAX but... 转向角度的限制值
#define STEERING_INTERNAL_PERIOD 20                              // ms (10ms is too fast for HEV) 转向控制中 PID 控制器运行周期的大小，单位是毫秒。

// accel/brake parameters
// 20 km/h 以下的速度下 PID 控制的比例、积分和微分参数。
#define _K_ACCEL_P_UNTIL20 40.0 // 30.0
#define _K_ACCEL_I_UNTIL20 0.1  // 5.0
#define _K_ACCEL_D_UNTIL20 0.1
// 10 km/h 以下的速度下 PID 控制的比例、积分和微分参数。
#define _K_ACCEL_P_UNTIL10 40.0 // 30.0
#define _K_ACCEL_I_UNTIL10 0.1  // 5.0
#define _K_ACCEL_D_UNTIL10 0.1
// 积分限幅的周期数。
#define _K_ACCEL_I_CYCLES 1000
// 积分项的最大值。
#define _ACCEL_MAX_I 3000
// 两次控制间加速踏板的最大变化量。
#define _ACCEL_STROKE_DELTA_MAX 1000
// 松开加速踏板时每次控制间踏板位置的最大变化量。
#define _ACCEL_RELEASE_STEP 400
// 加速踏板的最大位置值。
#define _ACCEL_PEDAL_MAX 1700
// 加速踏板位置的偏移值。
#define _ACCEL_PEDAL_OFFSET 200

#define _K_BRAKE_P 40.0
#define _K_BRAKE_I 10.0
#define _K_BRAKE_D 10.0
// 积分限幅的周期数。
#define _K_BRAKE_I_CYCLES 100
#define _BRAKE_MAX_I 500               // 200 是积分项的最大值，用于限制积分项对控制器输出的影响。
#define _BRAKE_STROKE_DELTA_MAX 1000   // 刹车踏板位移的最大变化量，用于限制刹车踏板输出的变化速度。
#define _BRAKE_RELEASE_STEP 500        // 刹车踏板释放时每次变化的步长。
#define _BRAKE_PEDAL_MAX 3500          // 4095 //刹车踏板的最大输出值。
#define _BRAKE_PEDAL_MED 3000          // 刹车踏板在制动中等力度下的输出值。
#define _BRAKE_PEDAL_STOPPING_MAX 2500 // 急停车时刹车踏板的最大输出值。
#define _BRAKE_PEDAL_STOPPING_MED 1500 // 急停车时刹车踏板的中等输出值。
// 刹车踏板偏移量
#define _BRAKE_PEDAL_OFFSET 1000

// steering parameters
#define _STEERING_MAX_ANGVELSUM 1000    // 方向盘角速度和的最大值
#define _STEERING_MAX_TORQUE 1800       // 2000 方向盘扭矩最大值
#define _STEERING_MAX_SUM 3000          // 方向盘偏差累加和的最大值
#define _STEERING_ANGVEL_BOUNDARY 500.0 // deg/s //角速度边界
#define _STEERING_IGNORE_ERROR 0.5      // 1.0 // deg //忽略的方向盘偏差
#define _STEERING_DIFF_SMALL 10         // deg //方向盘偏差小于该值时使用更严格的PID系数

// fast PID factors
#define _K_STEERING_P 8.0 // 用于计算方向盘扭矩的PID系数
#define _K_STEERING_I 0.8
#define _K_STEERING_D 1.5

// fast PID factors
#define _K_STEERING_P_STRICT 8.0 // 60
#define _K_STEERING_I_STRICT 0.8 // 5.0
#define _K_STEERING_D_STRICT 1.5 // 5.0

// slow PID factors
// D = 1.7 might also work.
#define _K_STEERING_P_SLOW 8.0
#define _K_STEERING_I_SLOW 0.8
#define _K_STEERING_D_SLOW 1.5

#define _STEERING_ANGLE_ERROR 0 // deg

#endif //__AUTOWARE_CONFIG_H__