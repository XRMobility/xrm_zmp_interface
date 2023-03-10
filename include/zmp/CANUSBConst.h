#ifndef CANUSBCONST_H_
#define CANUSBCONST_H_

#include <stdio.h>
#include "zmp/CANConst.h"


#define _DEVICE_NAME		"/dev/ttyACM0"

#define _BAUDRATE		B115200
#define _RECEIVE_SIZE		18	

#define SERIAL_READ_SIZE 256
#define NUM_OF_ADC2DIST_TABLE 15

enum CANUSBZ_SPEED{
    CAN_SPEED_125 = 1,
    CAN_SPEED_250 = 2,
    CAN_SPEED_500 = 3,
    CAN_SPEED_1000 = 4
};

enum CANUSBZ_IDKIND{
    CANID_KIND_11 = 0,
    CANID_KIND_29 = 1,
};


namespace zmp {
    namespace minivan {

class SerialReceiveHandler{
public:
    virtual void OnReceiveCANUSB(int channel, long timeStamp, CANMsg* msg)	= 0;
};
    }
}
#endif /* CANUSBCONST_H_ */
