#include "main_cnt.h"
#include "autoware_socket.h"

#include <sys/shm.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <iostream>
using namespace std;

struct struct_PID_controller *shm_ptr;

std::string ros_ip_address = "127.0.0.1";
// CAN通信间隔时间，单位为毫秒。
int can_tx_interval = 10; // ms
// 从ROS主机接收指令间隔时间，单位为毫秒。
int cmd_rx_interval = 100; // ms

vehicle_state_t vstate;

// 用于配置网络通信。
bool MainCnt::ConfigSocket(void)
{
    std::ifstream ifs("./config");
    std::string str;
    if (ifs.fail())
    {
        return false;
    }

    if (getline(ifs, str))
    {
        can_tx_interval = atoi(str.c_str());
        cout << "CAN Interval = " << can_tx_interval << " ms" << std::endl;
    }
    else
    {
        return false;
    }

    if (getline(ifs, str))
    {
        cmd_rx_interval = atoi(str.c_str());
        cout << "CMD Interval = " << cmd_rx_interval << " ms" << std::endl;
    }
    else
    {
        return false;
    }

    if (getline(ifs, str))
    {
        ros_ip_address = str;
        cout << "ROS IP Address = " << ros_ip_address << std::endl;
    }
    else
    {
        return false;
    }

    // key generation
    key_t shm_key = ftok(SHM_SEED_PATH.c_str(), 1);

    // open shared memory
    int shm_id = shmget(shm_key, sizeof(struct_PID_controller), 0444); // read only

    // get address of shared memory
    shm_ptr = (struct struct_PID_controller *)shmat(shm_id, NULL, 0);

    return true;
}

// 用于更新车辆状态。
void MainCnt::UpdateState(void)
{
    ZMP_UPDATE_STATE();
    vstate.accel_stroke = ZMP_ACCEL_STROKE();
    vstate.brake_stroke = ZMP_BRAKE_STROKE();
    vstate.steering_torque = ZMP_STEERING_TORQUE();
    vstate.steering_angle = ZMP_STEERING_ANGLE();
    vstate.velocity = ZMP_VELOCITY();
    vstate.tstamp = (long long int)(getTime());
}

void MainCnt::ClearCntDiag(void)
{
    ZMP_CLEAR_CNT_DIAG();
}