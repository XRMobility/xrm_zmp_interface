#ifndef CANUSB_H_
#define CANUSB_H_

#include "zmp/CANUSBConst.h"
#include <pthread.h>
#include <termios.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

namespace zmp {
	namespace minivan {
class CANUSBZ {
public:
    CANUSBZ();
    virtual ~CANUSBZ();
    bool Init(char* devName = NULL);
    bool Start();
    bool Stop();
    bool Close();
    bool SetReceiveHandler(SerialReceiveHandler* handler);
    bool SendMsg(int ch, CANMsg cmsg);
    bool SetCANUSBZParam(int ch, CANUSBZ_SPEED speed, CANUSBZ_IDKIND kind);

private:
    bool _b_on_rcv_thread;
    int _fd;
    pthread_t _thread;
    bool _b_terminated;


    struct termios _oldtio;
    struct termios _newtio;
    struct pollfd  _client;
    SerialReceiveHandler* _callback_handler;

    bool InitThread();
    bool InitDevice();
    bool InitDevice(char* devName);
    bool InitDevice_sub(int fd);
    void ReadThread();
    static void* ReadThreadEntry(void* pParam);
    unsigned char calc_sum(const unsigned char *data, int len);
    void ParseData(unsigned char* res, int* ch, long* time, CANMsg* cmsg);
};
    }
}

#endif /* CANUSB_H_ */
