/*
 * Copyright 2015 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "main_cnt.h"
#include "autoware_socket.h"
#include <queue>
// 当前的加速度估计值。
double estimate_accel = 0.0;
// 目标加速度水平。
int target_accel_level = 0;
// 加速度变化量之和。
double accel_diff_sum = 0;
// 制动变化量之和。
double brake_diff_sum = 0;
// 表示存储加速度变化量的队列。
queue<double> accel_diff_buffer;
// 表示存储制动变化量的队列。
queue<double> brake_diff_buffer;
// 清除加速度和刹车差分缓冲区中的所有元素，同时将 accel_diff_sum 和 brake_diff_sum 重置为零。
static void clear_diff()
{
    int i;

    accel_diff_sum = 0;
    brake_diff_sum = 0;

    for (i = 0; i < (int)accel_diff_buffer.size(); i++)
    {
        accel_diff_buffer.pop();
    }
    for (i = 0; i < (int)brake_diff_buffer.size(); i++)
    {
        brake_diff_buffer.pop();
    }
}

void MainWindow::SetDrvMode(int mode)
{
    switch (mode)
    {
    case CMD_MODE_MANUAL:
        cout << "Switching to MANUAL (Accel/Brake)" << endl;
        ZMP_SET_DRV_MANUAL();
        break;
    case CMD_MODE_PROGRAM:
        cout << "Switching to PROGRAM (Accel/Brake)" << endl;
        ZMP_SET_DRV_PROGRAM();
        clear_diff(); // initialize I control.
        break;
    default:
        cout << "Unknown mode: " << mode << endl;
    }
}

void MainWindow::SetGear(int gear)
{
    double current_velocity = vstate.velocity; // km/h

    // double check if the velocity is zero,
    // SetGear() should not be called when driving.
    if (current_velocity != 0.0)
    {
        return;
    }

    // make sure to stop the vehicle.
    ZMP_STOP();

    switch (gear)
    {
    case CMD_GEAR_D:
        cout << "Shifting to Gear D" << endl;
        ZMP_SET_SHIFT_POS_D();
        break;
    case CMD_GEAR_R:
        cout << "Shifting to Gear R" << endl;
        ZMP_SET_SHIFT_POS_R();
        break;
    case CMD_GEAR_B:
        cout << "Shifting to Gear B" << endl;
        ZMP_SET_SHIFT_POS_B();
        break;
    case CMD_GEAR_N:
        cout << "Shifting to Gear N" << endl;
        ZMP_SET_SHIFT_POS_N();
        break;
    default:
        cout << "Unknown gear: " << gear << endl;
    }

    sleep(1); // wait for a while to change the gear
}

// 这段代码实现了一个PID控制器，用于控制汽车加速度，并返回所需的加速踏板行程。具体来说，该函数的输入参数是当前速度和目标速度，输出参数是所需的加速踏板行程。函数实现了三个控制器参数P、I、D的调整，具体方式是将调整后的参数与误差、误差积分和误差微分相乘，并将结果加起来，得到一个控制量，即所需的加速踏板行程。PID控制器的输出值限制在最大加速踏板行程和0之间，并记录PID控制器的输入和输出值。

double _accel_stroke_pid_control(double current_velocity, double cmd_velocity)
{
    double e;
    static double e_prev = 0;
    double e_i;
    double e_d;
    double ret;

    // acclerate by releasing the brake pedal if pressed.
    if (vstate.brake_stroke > _BRAKE_PEDAL_OFFSET)
    {
        /*
        double target_brake_stroke = vstate.brake_stroke - _BRAKE_RELEASE_STEP;
        if (target_brake_stroke < 0)
          target_brake_stroke = 0;
        ret = -target_brake_stroke; // if ret is negative, brake will be applied.
        */

        // vstate has some delay until applying the current state.
        // perhaps we can just return 0 (release brake pedal) here to avoid acceleration delay.
        ret = 0;

        /* reset PID variables. */
        e_prev = 0;
        // clear_diff();
    }
    else
    { // PID control
        double target_accel_stroke;

        e = cmd_velocity - current_velocity;

        e_d = e - e_prev;

        accel_diff_sum += e;

#if 0 // shouldn't we limit the cycles for I control?
    accel_diff_buffer.push(e);
    if (accel_diff_buffer.size() > _K_ACCEL_I_CYCLES) {
      double e_old = accel_diff_buffer.front();
      accel_diff_sum -= e_old;
      if (accel_diff_sum < 0) {
        accel_diff_sum = 0;
      }
      accel_diff_buffer.pop();
    }
#endif

        if (accel_diff_sum > _ACCEL_MAX_I)
        {
            e_i = _ACCEL_MAX_I;
        }
        else
        {
            e_i = accel_diff_sum;
        }

#if 1
        // target_accel_stroke = _K_ACCEL_P * e + _K_ACCEL_I * e_i + _K_ACCEL_D * e_d;
        if (current_velocity > 15 /***10**20160905***/)
        {
            target_accel_stroke = _K_ACCEL_P_UNTIL20 * e + _K_ACCEL_I_UNTIL20 * e_i +
                                  _K_ACCEL_D_UNTIL20 * e_d;
        }
        else
        {
            target_accel_stroke = _K_ACCEL_P_UNTIL10 * e + _K_ACCEL_I_UNTIL10 * e_i +
                                  _K_ACCEL_D_UNTIL10 * e_d;
        }
#else
        printf("accel_p = %lf, accel_i = %lf, accel_d = %lf\n", shm_ptr->accel.P, shm_ptr->accel.I, shm_ptr->accel.D);
        target_accel_stroke = shm_ptr->accel.P * e + shm_ptr->accel.I * e_i + shm_ptr->accel.D * e_d;
#endif

        if (target_accel_stroke > _ACCEL_PEDAL_MAX)
        {
            target_accel_stroke = _ACCEL_PEDAL_MAX;
        }
        else if (target_accel_stroke < 0)
        {
            target_accel_stroke = 0;
        }

        // cout << "e = " << e << endl;
        // cout << "e_i = " << e_i << endl;
        // cout << "e_d = " << e_d << endl;

        ret = target_accel_stroke;

        e_prev = e;

#if 1 /* log */
        ofstream ofs("/tmp/drv_accel.log", ios::app);
        ofs << cmd_velocity << " "
            << current_velocity << " "
            << e << " "
            << e_i << " "
            << e_d << " "
            << target_accel_stroke << " "
            << endl;
#endif
    }

    return ret;
}

double _brake_stroke_pid_control(double current_velocity, double cmd_velocity)
{
    double e;
    static double e_prev = 0;
    double e_i;
    double e_d;
    double ret;

    // decelerate by releasing the accel pedal if pressed.
    if (vstate.accel_stroke > _ACCEL_PEDAL_OFFSET)
    {
        /*
        double target_accel_stroke = vstate.accel_stroke - _ACCEL_RELEASE_STEP;
        if (target_accel_stroke < 0)
          target_accel_stroke = 0;
        ret = -target_accel_stroke; // if ret is negative, accel will be applied.
        */

        // vstate has some delay until applying the current state.
        // perhaps we can just return 0 (release accel pedal) here to avoid deceleration delay.
        ret = 0;

        /* reset PID variables. */
        e_prev = 0;
        // clear_diff();
    }
    else
    { // PID control
        double target_brake_stroke;

        // since this is braking, multiply -1.
        e = -1 * (cmd_velocity - current_velocity);
        if (e > 0 && e <= 1)
        { // added @ 2016/Aug/29
            e = 0;
        }

        e_d = e - e_prev;

        brake_diff_sum += e;
#if 0
    brake_diff_buffer.push(e);
    if (brake_diff_buffer.size() > _K_BRAKE_I_CYCLES) {
      double e_old = brake_diff_buffer.front();
      brake_diff_sum -= e_old;
      if (brake_diff_sum < 0) {
        brake_diff_sum = 0;
      }
      brake_diff_buffer.pop();
    }
#endif

        if (brake_diff_sum > _BRAKE_MAX_I)
        {
            e_i = _BRAKE_MAX_I;
        }
        else
        {
            e_i = brake_diff_sum;
        }

        target_brake_stroke = _K_BRAKE_P * e + _K_BRAKE_I * e_i + _K_BRAKE_D * e_d;
        if (target_brake_stroke > _BRAKE_PEDAL_MAX)
        {
            target_brake_stroke = _BRAKE_PEDAL_MAX;
        }
        else if (target_brake_stroke < 0)
        {
            target_brake_stroke = 0;
        }

        cout << "target: " << target_brake_stroke << endl;
        cout << "vstate: " << vstate.brake_stroke << endl;
#ifdef USE_BRAKE_STROKE_DELTA_MAX // should I use??
        if (target_brake_stroke - vstate.brake_stroke > _BRAKE_STROKE_DELTA_MAX)
        {
            target_brake_stroke = vstate.brake_stroke + _BRAKE_STROKE_DELTA_MAX;
        }
#endif

        cout << "e = " << e << endl;
        cout << "e_i = " << e_i << endl;
        cout << "e_d = " << e_d << endl;

        ret = target_brake_stroke;

        e_prev = e;

#if 1 /* log */
        ofstream ofs("/tmp/drv_brake.log", ios::app);
        ofs << cmd_velocity << " "
            << current_velocity << " "
            << e << " "
            << e_i << " "
            << e_d << " "
            << target_brake_stroke << " "
            << vstate.brake_stroke << " "
            << endl;
#endif
    }

    return ret;
}

// 该函数是控制车辆停止的函数，输入为当前速度，输出为需要施加的制动踏板位置。如果车速小于0.1km/h，则会将刹车踏板位置逐渐增加，以施加全力制动，直到达到最大制动踏板位置；否则，直接设置制动踏板位置为中等力度。函数中使用了一个静态变量old_brake_stroke记录上一个时刻的制动踏板位置，以计算需要增加的制动踏板力度。
double _stopping_control(double current_velocity)
{
    double ret;
    static double old_brake_stroke = _BRAKE_PEDAL_STOPPING_MED;

    // decelerate by using brake
    if (current_velocity < 0.1)
    {
        // nearly at stop -> apply full brake. brake_stroke should reach BRAKE_PEDAL_MAX in one second.
        int gain = (int)(((double)_BRAKE_PEDAL_STOPPING_MAX) * cycle_time);
        ret = old_brake_stroke + gain;
        if ((int)ret > _BRAKE_PEDAL_STOPPING_MAX)
            ret = _BRAKE_PEDAL_STOPPING_MAX;
        old_brake_stroke = ret;
    }
    else
    {
        /*
        // one second is approximately how fast full brakes applied in sharp stop
        int gain = (int)(((double)_BRAKE_PEDAL_STOPPING_MED)*cycle_time);
        ret = vstate.brake_stroke + gain;
        if ((int)ret > _BRAKE_PEDAL_STOPPING_MED)
          ret = _BRAKE_PEDAL_STOPPING_MED;
        */

        // vstate has some delay until applying the current state.
        // perhaps we can just set BRAKE_PEDAL_MED to avoid deceleration delay.
        ret = _BRAKE_PEDAL_STOPPING_MED;
        old_brake_stroke = _BRAKE_PEDAL_STOPPING_MED;
    }

    return ret;
}
// 这是一个控制车辆刹车和加速的函数。如果车辆不处于程序控制模式，则不执行控制。该函数通过缓存最近的速度值来估算当前加速度。如果命令速度大于当前速度且不为零并且当前速度小于速度限制，则调用 _accel_stroke_pid_control 函数来计算油门踏板的位置，并设置制动踏板位置为零。否则，如果命令速度小于当前速度且不为零，则调用 _brake_stroke_pid_control 函数来计算制动踏板的位置，并将油门踏板位置设置为零。如果命令速度为零且当前速度不为零，则执行减速并停车，即在接近停止时应用制动器，否则应用制动器以减速。所有操作都通过 ZMP_SET_DRV_STROKE 和 ZMP_SET_BRAKE_STROKE 函数来执行。函数还将速度和命令速度记录到日志文件 /tmp/driving.log 中。
void MainWindow::StrokeControl(double current_velocity, double cmd_velocity)
{
    static queue<double> vel_buffer;
    static uint vel_buffer_size = 10;
    double old_velocity = 0.0;

    // don't control if not in program mode.
    if (!ZMP_DRV_CONTROLLED())
    {
        clear_diff();
#ifdef USE_BRAKE_LAMP
        sndBrkLampOff();
#endif /* USE_BRAKE_LAMP */

        return;
    }

    // estimate current acceleration.
    vel_buffer.push(current_velocity);
    if (vel_buffer.size() > vel_buffer_size)
    {
        old_velocity = vel_buffer.front();
        vel_buffer.pop(); // remove old_velocity from the queue.
        estimate_accel =
            (current_velocity - old_velocity) / (cycle_time * vel_buffer_size);
    }

    cout << "estimate_accel: " << estimate_accel << endl;

    if (fabs(cmd_velocity) > current_velocity && fabs(cmd_velocity) > 0.0 && current_velocity < SPEED_LIMIT)
    {
        double accel_stroke;
        cout << "accelerate: current_velocity=" << current_velocity
             << ", cmd_velocity=" << cmd_velocity << endl;
        accel_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity);
        if (accel_stroke > 0)
        {
            cout << "ZMP_SET_DRV_STROKE(" << accel_stroke << ")" << endl;
            ZMP_SET_DRV_STROKE(accel_stroke);
            ZMP_SET_BRAKE_STROKE(0);
        }
        else
        {
            cout << "ZMP_SET_DRV_STROKE(0)" << endl;
            ZMP_SET_DRV_STROKE(0);
            cout << "ZMP_SET_BRAKE_STROKE(" << -accel_stroke << ")" << endl;
            ZMP_SET_BRAKE_STROKE(-accel_stroke);
        }
    }
    else if (fabs(cmd_velocity) < current_velocity && fabs(cmd_velocity) > 0.0)
    {
        double brake_stroke;
        cout << "decelerate: current_velocity=" << current_velocity
             << ", cmd_velocity=" << cmd_velocity << endl;
        brake_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity);
        if (brake_stroke > 0)
        {
            cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
            ZMP_SET_BRAKE_STROKE(brake_stroke);
            ZMP_SET_DRV_STROKE(0);
        }
        else
        {
            cout << "ZMP_SET_BRAKE_STROKE(0)" << endl;
            ZMP_SET_BRAKE_STROKE(0);
            cout << "ZMP_SET_DRV_STROKE(" << -brake_stroke << ")" << endl;
            ZMP_SET_DRV_STROKE(-brake_stroke);
        }
    }
    else if (cmd_velocity == 0.0 && current_velocity != 0.0)
    {
        double brake_stroke;
        cout << "stopping: current_velocity=" << current_velocity
             << ", cmd_velocity=" << cmd_velocity << endl;
        if (current_velocity < 4.0)
        { // nearly stopping
            ZMP_SET_DRV_STROKE(0);
            brake_stroke = _stopping_control(current_velocity);
            cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
            ZMP_SET_BRAKE_STROKE(brake_stroke);
        }
        else
        {
            brake_stroke = _brake_stroke_pid_control(current_velocity, 0);
            if (brake_stroke > 0)
            {
                cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
                ZMP_SET_BRAKE_STROKE(brake_stroke);
            }
            else
            {
                cout << "ZMP_SET_DRV_STROKE(0)" << endl;
                ZMP_SET_DRV_STROKE(0);
                cout << "ZMP_SET_DRV_STROKE(" << -brake_stroke << ")" << endl;
                ZMP_SET_DRV_STROKE(-brake_stroke);
            }
        }
    }
    else
    {
        cout << "unknown: current_velocity=" << current_velocity
             << ", cmd_velocity=" << cmd_velocity << endl;
    }
}
// 该函数实现了速度控制，根据当前速度和目标速度，增加或减少驾驶速度，以达到目标速度的效果。在增加速度时，先将当前速度增加1（vel_diff_inc），然后加上一个偏移值（vel_offset_inc）作为目标速度。在减速时，先将当前速度减少2（vel_diff_dec），如果减少后的速度仍然大于一个偏移值（vel_offset_dec），则将其作为目标速度；否则，将速度设置为0。
void MainWindow::VelocityControl(double current_velocity, double cmd_velocity)
{
    double vel_diff_inc = 1.0;
    double vel_diff_dec = 2.0;
    double vel_offset_inc = 2;
    double vel_offset_dec = 4;
    if (cmd_velocity > current_velocity)
    {
        double increase_velocity = current_velocity + vel_diff_inc;
        ZMP_SET_DRV_VELOC((increase_velocity + vel_offset_inc) * 100);
        cout << "increase: "
             << "vel = " << increase_velocity << endl;
    }
    else
    {
        double decrease_velocity = current_velocity - vel_diff_dec;
        if (decrease_velocity > vel_offset_dec)
        {
            ZMP_SET_DRV_VELOC((decrease_velocity - vel_offset_dec) * 100);
        }
        else if (current_velocity > 0)
        {
            decrease_velocity = 0;
            ZMP_SET_DRV_VELOC(0);
        }
        cout << "decrease: "
             << "vel = " << decrease_velocity << endl;
    }
}
