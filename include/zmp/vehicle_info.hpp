#ifndef vehicle_info_hpp
#define vehicle_info_hpp

#define ZMP_LOOP_RATE 50 // Hz

// prius parameters
#define WHEEL_BASE 2.7                                           // tire-to-tire size of Prius.//车轮轴距的大小
#define WHEEL_ANGLE_MAX 31.28067                                 // max angle of front tires. //前轮转向角度的最大值
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX / WHEEL_ANGLE_MAX) // 转向角度与前轮转向角度的比值
#define STEERING_ANGLE_MAX 666                                   // max angle of steering  转向角度的最大值
#define STEERING_ANGLE_LIMIT 550                                 // could be STEERING_ANGLE_MAX but... 转向角度的限制值
#define STEERING_INTERNAL_PERIOD 20                              // ms (10ms is too fast for HEV) 转向控制中 PID 控制器运行周期的大小，单位是毫秒。

// 表示不使用时间戳。在某些情况下，例如测试和调试中，可能不需要使用时间戳来标记数据。
#define NO_TSTAMP 0
// 两个常量，分别表示手动和自动控制模式。在自动驾驶系统中，可以使用这些常量来指示控制模式。
#define CMD_MODE_MANUAL 0
#define CMD_MODE_PROGRAM 1
// 这些常量表示车辆的挡位，分别对应于驾驶、倒车、制动和空档。
#define CMD_GEAR_D 1
#define CMD_GEAR_R 2
#define CMD_GEAR_B 3
#define CMD_GEAR_N 4
// 这些常量用于表示CAN数据的不同类型，包括控制模式、字符串和驱动模式等。
#define CAN_KEY_MODE (0)
#define CAN_MODE_STR (1)
#define CAN_MODE_DRV (2)
// 这些常量用于表示CAN数据中的不同关键字，例如时间戳、速度、角度、扭矩、加速度和刹车等。
#define CAN_KEY_TIME (1)
#define CAN_KEY_VELOC (2)
#define CAN_KEY_ANGLE (3)
#define CAN_KEY_TORQUE (4)
#define CAN_KEY_ACCEL (5)
#define CAN_KEY_BRAKE (6)
#define CAN_KEY_SHIFT (7)
// 一个常量，表示一毫秒的时间，可以用于在代码中指定时间单位。
#define MILLISECOND 1000
// 表示车辆的速度限制，通常以公里/小时为单位。
#define SPEED_LIMIT 80 // km/h
// 表示在使用冲程控制时的车辆速度限制。
#define STROKE_SPEED_LIMIT 80 // use stroke until 80 km/h
// 用于存储车辆的位置、姿态和时间戳等信息。
typedef struct pose_data
{
    double x;
    double y;
    double z;
    double R;
    double P;
    double Y;
    long long int tstamp;
} pose_data_t;
// 用于存储车辆的速度、角度和时间戳等信息。
typedef struct vel_data
{
    double tv;
    double sv;
    double lv; // linear_velocity
    double sa; // steering_angle
    long long int tstamp;
} vel_data_t;
// 包含一个vel_data_t结构体和一个控制模式（手动或自动）。
typedef struct vel_cmd_data
{
    vel_data_t vel;
    int mode;
} vel_cmd_data_t;
// 一个结构体，包含pose_data_t和vel_data_t结构体，以及电池电压、电机状态和时间戳等信息。
typedef struct base_data
{
    pose_data_t odom_pose;
    vel_data_t vel_cur;
    unsigned char batt_voltage;
    bool motor_state;
    long long int tstamp;
} base_data_t;
// 用于存储车辆的状态信息，包括基本数据、估计位置、速度指令和控制模式等。
typedef struct state_data
{
    long long int update_tstamp;
    long long int control_tstamp;

    // state data reading from base
    base_data_t base;

    // control info
    pose_data_t est_pose;
    vel_data_t vel_cmd;
    int control_mode;

} state_data_t;
// 一个结构体，用于存储毫米波雷达的配置信息，包括扫描范围、分辨率等。
typedef struct lrf_config
{
    int N;
    double fov; // field of view radians
    double res;
    double start_a;
    double stop_a;
} lrf_config_t;

typedef int lrf_data_t; // range data type
// 包含速度、控制模式、挡位、油门、转向和刹车等信息。
typedef struct _CMDDATA
{
    vel_data_t vel;
    int mode;
    int gear;
    int accel;
    int steer;
    int brake;
} CMDDATA;
// 用于存储车辆的加速度、刹车位置、转向扭矩、转向角度、速度和时间戳等信息。
typedef struct vehicle_state
{
    float accel_stroke;
    float brake_stroke;
    float steering_torque;
    float steering_angle;
    float velocity;
    long long int tstamp;
} vehicle_state_t;
// 包含PID控制器的参数（比例、积分和微分系数）。
struct PID_valueSet
{
    float P;
    float I;
    float D;
};

struct struct_PID_controller
{
    struct PID_valueSet accel;
    struct PID_valueSet brake;
    struct PID_valueSet steer;
};

// convert km/h to m/s
static inline double KmhToMs(double v)
{
    return (v * 1000.0 / (60.0 * 60.0));
}

#define _STEERING_ANGLE_ERROR 0 //deg
#endif // vehicle_info_hpp