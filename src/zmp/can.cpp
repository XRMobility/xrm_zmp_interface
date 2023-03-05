#include "autoware_socket.h"
#include "main_cnt.h"

// CAN + Drive Mode
std::string candata;
int drvmode;

// 使用了一个线程，用于创建一个套接字并将CAN数据发送到远程主机。它首先创建了一个套接字sock，并指定了要连接的主机和端口号。然后，它将CAN数据打包成一个字符串senddata，其中包含控制模式和CAN数据等信息。最后，它将字符串发送到远程主机，并关闭套接字。
void *CANSenderEntry(void *a)
{
  struct sockaddr_in server;
  int sock;
  std::string senddata;
  std::ostringstream oss;
  oss << CAN_KEY_MODE << "," << drvmode << "," << candata; // global variables.
  senddata = oss.str();
  unsigned int **addrptr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    perror("socket");
    return NULL;
  }
  server.sin_family = AF_INET;
  server.sin_port = htons(10000);
  server.sin_addr.s_addr = inet_addr(ros_ip_address.c_str());
  if (server.sin_addr.s_addr == 0xffffffff)
  {
    struct hostent *host;

    host = gethostbyname(ros_ip_address.c_str());
    if (host == NULL)
    {
      if (h_errno == HOST_NOT_FOUND)
      {
        fprintf(stderr, "info : host not found : %s\n", ros_ip_address.c_str());
      }
      else
      {
        fprintf(stderr, "info : %s : %s\n", hstrerror(h_errno), ros_ip_address.c_str());
      }
      return NULL;
    }
    addrptr = (unsigned int **)host->h_addr_list;

    while (*addrptr != NULL)
    {
      server.sin_addr.s_addr = *(*addrptr);

      // if connected, break out this loop.
      if (connect(sock,
                  (struct sockaddr *)&server,
                  sizeof(server)) == 0)
      {
        break;
      }

      addrptr++;
      // if failed to connect, try another address.
    }

    // if totally failed to connect, return NULL.
    if (*addrptr == NULL)
    {
      perror("info : connect");
      return NULL;
    }
  }
  else
  {
    if (connect(sock,
                (struct sockaddr *)&server, sizeof(server)) != 0)
    {
      perror("info : connect");
      return NULL;
    }
  }

  int n;

  printf("info : %s\n", senddata.c_str());

  n = send(sock, senddata.c_str(), senddata.size() + 1, 0);
  if (n < 0)
  {
    perror("write");
    return NULL;
  }

  close(sock);

  return NULL;
}

void wrapSender(void)
{
  pthread_t _cansender;

  if (pthread_create(&_cansender, NULL, CANSenderEntry, NULL))
  {
    fprintf(stderr, "info : pthread create error");
    return;
  }

  usleep(can_tx_interval * 1000);
  pthread_join(_cansender, NULL);
}

// 将车辆的CAN数据打包成字符串并调用CANSenderEntry()函数来发送。它首先将时间戳打包到字符串中，然后根据不同的数据类型，将速度、角度、油门和刹车等数据打包到字符串中。然后，它将CAN数据存储到全局变量candata中，并使用UpdateState()函数更新车辆状态。最后，它确定控制模式并调用wrapSender()函数来发送CAN数据。wrapSender()函数会在另一个线程中调用CANSenderEntry()函数来发送CAN数据。
void MainCnt::SendCAN(void)
{
  char tmp[300] = "";
  std::string can = "";
  // add time when sent out.
  sprintf(tmp, "%d,'%d/%02d/%02d %02d:%02d:%02d.%ld'",
          CAN_KEY_TIME,
          _s_time->tm_year + 1900, _s_time->tm_mon + 1, _s_time->tm_mday,
          _s_time->tm_hour + 9, _s_time->tm_min, _s_time->tm_sec,
          _getTime.tv_usec);
  can += tmp;
  if (_selectLog.drvInf == true)
  {
    sprintf(tmp, ",%d,%3.2f,%d,%d,%d,%d",
            CAN_KEY_VELOC, _drvInf.veloc,
            CAN_KEY_ACCEL, _drvInf.actualPedalStr,
            CAN_KEY_SHIFT, _drvInf.actualShift);
    can += tmp;
  }
  if (_selectLog.strInf == true)
  {
    sprintf(tmp, ",%d,%3.2f,%d,%d",
            CAN_KEY_ANGLE, _strInf.angle,
            CAN_KEY_TORQUE, _strInf.torque);
    can += tmp;
  }
  if (_selectLog.brkInf == true)
  {
    sprintf(tmp, ",%d,%d", CAN_KEY_BRAKE, _brakeInf.actualPedalStr);
    can += tmp;
  }

  candata = can;
  MainCnt::UpdateState();
  drvmode = ZMP_STR_CONTROLLED() ? CAN_MODE_STR : 0;
  drvmode |= ZMP_DRV_CONTROLLED() ? CAN_MODE_DRV : 0;

  wrapSender();
}