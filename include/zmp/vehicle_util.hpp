#ifndef vehicle_util_hpp
#define vehicle_util_hpp
#include <vehicle_util.hpp>

#include <iostream>
#include <queue>
#include <string>

static double steering_diff_sum = 0;
std::queue<double> steering_diff_buffer;

static void clear_diff()
{
  int i = 0;

  steering_diff_sum = 0;
  for (i = 0; i < steering_diff_buffer.size(); i++) {
    steering_diff_buffer.pop();
  }
}

#endif  // vehicle_util_hpp