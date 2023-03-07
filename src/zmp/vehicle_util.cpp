#include <zmp/vehicle_util.hpp>

static void clear_diff()
{
  int i = 0;

  steering_diff_sum = 0;
  for (i = 0; i < steering_diff_buffer.size(); i++) {
    steering_diff_buffer.pop();
  }
}

void SetStrMode(int mode){
    switch (mode)
    {
    case CMD_MODE_MANUAL:
        std::cout << "Manual mode" << std::endl;
        
        break;
    
    default:
        break;
    }
}
