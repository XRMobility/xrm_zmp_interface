#ifndef vehicle_util_hpp
#define vehicle_util_hpp
#include <vehicle_info.hpp>

#include <iostream>
#include <queue>
#include <string>

static double steering_diff_sum = 0;
std::queue<double> steering_diff_buffer;

static void clear_diff();

void SetStrMode(int mode);


#endif  // vehicle_util_hpp