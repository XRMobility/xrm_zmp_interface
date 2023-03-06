#ifndef vehicle_info_hpp
#define vehicle_info_hpp

// prius parameters
#define WHEEL_BASE 2.7                                           // tire-to-tire size of Prius.//车轮轴距的大小
#define WHEEL_ANGLE_MAX 31.28067                                 // max angle of front tires. //前轮转向角度的最大值
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX / WHEEL_ANGLE_MAX) // 转向角度与前轮转向角度的比值
#define STEERING_ANGLE_MAX 666                                   // max angle of steering  转向角度的最大值
#define STEERING_ANGLE_LIMIT 550                                 // could be STEERING_ANGLE_MAX but... 转向角度的限制值
#define STEERING_INTERNAL_PERIOD 20                              // ms (10ms is too fast for HEV) 转向控制中 PID 控制器运行周期的大小，单位是毫秒。

#endif // vehicle_info_hpp