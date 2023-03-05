#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "main_cnt.h"
#include "autoware_socket.h"

// cmd_rx_interval 接收指令的时间间隔，以毫秒为单位。
// vstate：保存汽车当前的状态，包括速度、位置、方向等参数。
// 控制循环的周期，以秒为单位。
double cycle_time = 0.0;
// 线程标识符，用于控制模式和档位的设置。
pthread_t _modesetter;
pthread_t _gearsetter;

// hmm, dirty hacks...
// 当前的模式和档位。
int current_mode = -1;
int current_gear = -1;
// 用于控制模式和档位设置的互斥锁。
int mode_is_setting = false;
int gear_is_setting = false;
// 用于将字符串按照指定的字符分割成多个子串，返回分割后的子串数组。
std::vector<std::string> split(const std::string &input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter))
    {
        result.push_back(field);
    }
    return result;
}
// 用于与 ROS 系统进行通信，获取控制指令，将指令解析成速度、模式、档位等参数，存储在 CMDDATA 结构体中返回。
void Getter(CMDDATA &cmddata)
{
    std::string request;
    std::string cmdRes;
    char recvdata[32];

    struct sockaddr_in server;
    int sock;
    // char deststr[80] = serverIP.c_str();
    unsigned int **addrptr;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        perror("socket");
        return;
    }

    server.sin_family = AF_INET;
    server.sin_port = htons(10001);

    server.sin_addr.s_addr = inet_addr(ros_ip_address.c_str());
    if (server.sin_addr.s_addr == 0xffffffff)
    {
        struct hostent *host;

        host = gethostbyname(ros_ip_address.c_str());
        if (host == NULL)
        {
            if (h_errno == HOST_NOT_FOUND)
            {
                fprintf(stdout, "cmd : ROS PC not found : %s\n", ros_ip_address.c_str());
            }
            else
            {
                fprintf(stdout, "cmd : %s : %s\n", hstrerror(h_errno), ros_ip_address.c_str());
            }
            return;
        }

        addrptr = (unsigned int **)host->h_addr_list;

        while (*addrptr != NULL)
        {
            server.sin_addr.s_addr = *(*addrptr);

            /* break the loop when connected. */
            if (connect(sock, (struct sockaddr *)&server, sizeof(server)) == 0)
            {
                break;
            }

            addrptr++;
            // let's try another IP address if not successfully connected.
        }

        // if all connections failed...
        if (*addrptr == NULL)
        {
            perror("cmd : connect");
            return;
        }
    }
    else
    {
        if (connect(sock,
                    (struct sockaddr *)&server, sizeof(server)) != 0)
        {
            perror("cmd : connect");
            return;
        }
    }

    int n;

    while (true)
    {
        memset(recvdata, 0, sizeof(recvdata));
        n = recv(sock, recvdata, sizeof(recvdata), 0);
        if (n < 0)
        {
            perror("cmd : read erro");
            return;
        }
        else if (n == 0)
        {
            break;
        }
        cmdRes.append(recvdata, n);
    }

    // string version
    std::vector<std::string> cmdVector;
    cmdVector = split(cmdRes, ',');
    if (cmdVector.size() == 7 || cmdVector.size() == 9)
    {
        cmddata.vel.tv = atof(cmdVector[0].c_str());
        cmddata.vel.sv = atof(cmdVector[1].c_str());

        cout << endl
             << endl;
        cout << "cmddata.vel.tv = " << cmddata.vel.tv << endl;
        cout << "cmddata.vel.sv = " << cmddata.vel.sv << endl;

#if 0 /* log */
      ofstream ofs("/tmp/cmd.log", ios::app);
      ofs << cmddata.vel.tv << " " 
      << cmddata.vel.sv << " " 
      << endl;
#endif

        cmddata.mode = atoi(cmdVector[2].c_str());
        cmddata.gear = atoi(cmdVector[3].c_str());
        cmddata.accel = atoi(cmdVector[4].c_str());
        cmddata.steer = atoi(cmdVector[5].c_str());
        cmddata.brake = atoi(cmdVector[6].c_str());

        if (cmdVector.size() == 7)
        {
            cmddata.vel.lv = -1;
            cmddata.vel.sa = 0;
        }
        else
        {
            cmddata.vel.lv = atof(cmdVector[7].c_str());
            cmddata.vel.sa = atof(cmdVector[8].c_str());
        }
    }
    else
    {
        fprintf(stderr, "cmd : Recv data is invalid\n");
    }
    cout << "cmd : return data : " << cmdRes.c_str() << endl;

    close(sock);
}
// 用于更新汽车状态，包括速度、位置、方向等参数。
void Update(void *p)
{
    MainWindow *main = (MainWindow *)p;

    // update robocar state.
    main->UpdateState();
}
// 用于根据获取到的模式和档位信息，设置汽车的模式和档位。
void SetState(int mode, int gear, void *p)
{
    // 0=manual, 1=str, 2=drv, 3=str+drv
    if (mode >= 0 && mode != current_mode)
    {
        current_mode = mode;
        pthread_create(&_modesetter, NULL, MainWindow::ModeSetterEntry, p);
    }

    if (gear != current_gear)
    {
        double current_velocity = vstate.velocity; // km/h
        // never change the gear when driving!
        if (current_velocity == 0)
        {
            current_gear = gear;
            pthread_create(&_gearsetter, NULL, MainWindow::GearSetterEntry, p);
        }
    }
}

// 用于根据获取到的速度、方向等指令，控制汽车的运动状态。
void Control(vel_data_t vel, void *p)
{
    MainCnt *main = (MainCnt *)p;
    static long long int old_tstamp = 0;

    cycle_time = (vstate.tstamp - old_tstamp) / 1000.0; /* seconds */

    double current_velocity = vstate.velocity;             // km/h
    double current_steering_angle = vstate.steering_angle; // degree

    int cmd_velocity;
    if (vel.lv < 0)
    {
        cmd_velocity = vel.tv * 3.6;
    }
    else
    {
        cmd_velocity = vel.lv * 3.6;
    }
    int cmd_steering_angle;

    // We assume that the slope against the entire arc toward the
    // next waypoint is almost equal to that against
    // $l = 2 \pi r \times \frac{\theta}{360} = r \times \theta$
    // \theta = cmd_wheel_angle
    // vel.sv/vel.tv = Radius
    // l \simeq VEHICLE_LENGTH
    if (vel.tv < 0.1 && vel.lv < 0)
    { // just avoid divided by zero.
        cmd_steering_angle = current_steering_angle;
    }
    else
    {
        double wheel_angle_pi;
        double wheel_angle;
        if (vel.lv < 0)
        {
            wheel_angle_pi = (vel.sv / vel.tv) * WHEEL_BASE;
        }
        else
        {
            wheel_angle_pi = vel.sa;
        }
        wheel_angle = (wheel_angle_pi / M_PI) * 180.0;
        cmd_steering_angle = wheel_angle * WHEEL_TO_STEERING;

        cout << "Current: "
             << "vel = " << current_velocity
             << ", str = " << current_steering_angle << endl;
        cout << "Command: "
             << "vel = " << cmd_velocity
             << ", str = " << cmd_steering_angle << endl;

        for (int i = 0; i < cmd_rx_interval / STEERING_INTERNAL_PERIOD - 1; i++)
        {
            // Accel and Brake

            main->StrokeControl(current_velocity, cmd_velocity);
            // main->VelocityControl(current_velocity, cmd_velocity);

            //////////////////////////////////////////////////////
            // Steering
            //////////////////////////////////////////////////////

            main->SteeringControl(current_steering_angle, cmd_steering_angle);

            usleep(STEERING_INTERNAL_PERIOD * 1000);
            Update(main);
            current_velocity = vstate.velocity;             // km/h
            current_steering_angle = vstate.steering_angle; // degree
        }

        //////////////////////////////////////////////////////
        // remaining period.
        //////////////////////////////////////////////////////

        main->StrokeControl(current_velocity, cmd_velocity);

        main->SteeringControl(current_steering_angle, cmd_steering_angle);

        // save the time stamp.
        old_tstamp = vstate.tstamp;
    }
    // 用于在单独的线程中设置汽车的模式。
    void *MainWindow::ModeSetterEntry(void *a)
    {
        MainWindow *main = (MainWindow *)a;

        mode_is_setting = true; // loose critical section

        main->ClearCntDiag();
        // 0=manual, 1=str, 2=drv, 3=str+drv
        sleep(1);
        main->SetStrMode(((current_mode & CAN_MODE_STR) != 0) ? 1 : 0); // steering
        sleep(1);
        main->SetDrvMode(((current_mode & CAN_MODE_DRV) != 0) ? 1 : 0); // accel/brake
        sleep(1);

        mode_is_setting = false; // loose critical section

        return NULL;
    }
    // 用于在单独的线程中设置汽车的档位。
    void *MainWindow::GearSetterEntry(void *a)
    {
        MainWindow *main = (MainWindow *)a;

        gear_is_setting = true; // loose critical section

        main->SetGear(current_gear);
        sleep(1);

        gear_is_setting = false; // loose critical section

        return NULL;
    }
    // 用于在单独的线程中不断获取 ROS 系统发送的控制指令，更新汽车状态并进行控制。
    void *MainWindow::CMDGetterEntry(void *a)
    {
        MainWindow *main = (MainWindow *)a;
        CMDDATA cmddata;
        long long int tstamp;
        long long int interval;

        while (1)
        {

            // get commands from ROS.
            Getter(cmddata);

            // get time in milliseconds.
            tstamp = (long long int)getTime();

            // update robocar state.
            Update(main);

            // set mode and gear.
            SetState(cmddata.mode, cmddata.gear, main);

            if (!mode_is_setting && !gear_is_setting)
            {
#ifdef DIRECT_CONTROL
                // directly set accel, brake, and steer.
                Direct(cmddata.accel, cmddata.brake, cmddata.steer, main);
#else
                // control accel, brake, and steer.
                Control(cmddata.vel, main);
#endif
            }

            // get interval in milliseconds.
            interval = cmd_rx_interval - (getTime() - tstamp);

            if (interval > 0)
            {
                cout << "sleeping for " << interval << "ms" << endl;
                usleep(interval * 1000); // not really real-time...
            }
        }
        return NULL;
    }
