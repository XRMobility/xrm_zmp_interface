#include "HevControl.h"
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;
using namespace zmp::minivan;

#define SAMPLE_APP_VERSION "01.01.01"
#define USE_DEMO 1

struct BattInf {
    float current;              // 0x3B 充放電電流[A*10] 当前充电或放电电流，单位为 A*10。
    float max_dischg_current;	// 0x3CB 最大放電電流[A*10]
    float max_chg_current;      // 0x3CB 最大充電電流[A*10]
    float soc;                  // 0x3CB 残容量[%]剩余容量百分比。
    int min_temp;               // 0x3CB 最低温度[℃ ]
    int max_temp;               // 0x3CB 最高温度[℃ ]
    int voltage;                // 0x3CD バッテリ電圧[V]电池电压，单位为 V。
};

struct BrakeInf {
    bool pressed;           // ペダルスイッチ状態(ON=true, OFF=false) 制动踏板开关状态，true 表示按下，false 表示未按下。
    int actualPedalStr;		// ペダルストローク現在値 当前制动踏板行程值。
    int targetPedalStr;		// ペダルストローク目標値 制动踏板目标行程值。
    int inputPedalStr;		// ペダルストローク入力値 制动踏板输入行程值。
    float prl;              // PRLセンサ値
    float pfl;              // PFLセンサ値
    float prr;              // PRRセンサ値
    float pfr;              // PFRセンサ値
    float sks1;             // SKS1ペダルセンサ値
    float sks2;             // SKS2ペダルセンサ値
    float pmc1;             // PMC1ペダルセンサ値
    float pmc2;             // PMC2ペダルセンサ値
//    int targetStr;		// 目標ストローク
    unsigned char brakeLamp;  //制动灯状态。
    unsigned char blinkerLeft; //左转灯状态。
    unsigned char blinkerRight; //右转灯状态。
    unsigned char brakeMode; //制动模式。
};

struct OtherInf {
    float sideAcc;          // 0x22 Yawレート 横向加速度。
    float acc;              // 0x23 前後加速度 纵向加速度。
    float angleFromP;       // 0x25 ステアリング角度 从电控系统中获取的方向盘转角，单位为度。
    float brkPedalStrFromP;// 0x30 ブレーキペダル状態 从电控系统中获取的制动踏板状态。
    float velocFrFromP;		// 0xB1 右前輪速度[km/h*100]
    float velocFlFromP;		// 0xB1 左前輪速度[km/h*100]
    float velocRrFromP;		// 0xB3 右後輪速度[km/h*100]
    float velocRlFromP;		// 0xB3 左後輪速度[km/h*100]
    float velocFromP2;      // 0xB4 速度 从电控系统中获取的速度，单位为 km/h。
    int drv_mode;           // 0x120 ドライブモード 驱动模式。
    unsigned int drvPedalStrFromP;  // 0x244 アクセルペダル状態 从电控系统中获取的油门踏板状态。
    int rpm;                // 0x3C8 エンジンの回転数 发动机转速。
    float velocFromP;       // 0x3CA 速度 从电控系统中获取的速度，单位为 km/h。
    int ev_mode;            // 0x529 EVモード 电动模式。
    int temp;               // 0x52C 温度[℃ ] 温度，单位为℃。
    int shiftFromPrius;     // 0x540 シフト状態 从电控系统中获取的变速器状态。
    LIGHT_STATE light;		// 0x57F ライト状態 灯光状态。
    int level;              // 0x5A4 燃料の残量 剩余燃料量。
    DOOR_STATE door;		// 0x5B6 ドアの状態 车门状态。
    bool cluise;            // 0x5C8 クルーズコントロールON/OFF 巡航控制开关状态。
    char dtcData1;          // 0x7E8,0x7EA,0x7EB //故障码数据，用于故障诊断。
    char dtcData2;          // 0x7E8,0x7EA,0x7EB
    char dtcData3;          // 0x7E8,0x7EA,0x7EB
    char dtcData4;          // 0x7E8,0x7EA,0x7EB
    char dtcData5;          // 0x7E8,0x7EA,0x7EB
    char dtcData6;          // 0x7E8,0x7EA,0x7EB
    char dtcData7;          // 0x7E8,0x7EA,0x7EB
    char dtcData8;          // 0x7E8,0x7EA,0x7EB
};

struct DrvInf {
    int mode;               // ドライブモード(manual=0x00, program=0x10) 驱动模式，可以为 manual 或 program。
    int contMode;           // ドライブ制御モード(velocity=0x00, stroke=0x10) contMode：驱动控制模式，可以为 velocity 或 stroke。
    int overrideMode;		// オーバーライドモード(ON=0x00, OFF=0x10) 覆盖模式，可以为 ON 或 OFF。
    int servo;              // 制御のON/OFF(ON=true, OFF=false) 控制开关状态，true 表示开，false 表示关。
    int actualPedalStr;		// ペダルストローク現在値 当前油门踏板行程值。
    int targetPedalStr;		// ペダルストローク目標値 油门踏板目标行程值。
    int inputPedalStr;		// ペダルストローク入力値 油门踏板输入行程值。
    float vpa1;             // VPA1ペダルセンサ値 VPA1 踏板传感器值。
    float vpa2;             // VPA2ペダルセンサ値 VPA2 踏板传感器值。
    float targetVeloc;		// 目標速度[km/h*100]
    float veloc;            // 現在速度[km/h*100]
    int actualShift;        // シフトポジション現在値 当前变速器档位。
    int targetShift;		// シフトポジション目標値 变速器目标档位。
    int inputShift;         // シフトポジション入力値 变速器输入档位值。
//    int shiftFromPrius;
    int shiftRawVsx1;       // シフトRaw値VSX1 变速器 Raw 值 VSX1。
    int shiftRawVsx2;       // シフトRaw値VSX2 变速器 Raw 值 VSX2。
    int shiftRawVsx3;       // シフトRaw値VSX3 变速器 Raw 值 VSX3。
    int shiftRawVsx4;       // シフトRaw値VSX4 变速器 Raw 值 VSX4。
//    int spls_sp1;		// 車速パルス
//    int spls_fr;		// 右フロント車速パルス
//    int spls_fl;		// 左フロント車速パルス
//    int spls_rr;		// 右リア車速パルス
//    int spls_rl;		// 左リア車速パルス
//    int targetStr;		// 目標ストローク
};

struct StrInf {
    int mode;           // ステアリングモード(manual=0x00, program=0x10) 转向模式，可以为 manual 或 program。
    int cont_mode;      // ステアリング制御モード(torque=0x00, angle=0x10) 转向控制模式，可以为 torque 或 angle。
    int overrideMode;   // オーバーライドモード(ON=0x00, OFF=0x10) 覆盖模式，可以为 ON 或 OFF。
    int servo;          // 制御のON/OFF(ON=true, OFF=false) 控制开关状态，true 表示开，false 表示关。
    int targetTorque;   // 目標トルク 目标转向力矩。 
    int torque;         // 操舵トルク 当前转向力矩。
    float trq1;         // TRQ1トルクセンサ TRQ1 转向传感器值。
    float trq2;         // TRQ2トルクセンサ TRQ2 转向传感器值。
    float angle;        // 操舵角度[deg * 10] 当前转向角度，单位为 deg*10。
    float targetAngle;  // 目標操舵角[deg*10] 目标转向角度，单位为 deg*10。
};

struct AccInf{
    unsigned short time; //加速度数据采集时间。
    short accX;
    short accY;
    short accZ;
};

struct GyroInf{
    unsigned short time; //角速度数据采集时间。
    short gyroX;
    short gyroY;
    short gyroZ;
};

struct CompInf{
    unsigned short time; //磁力计数据采集时间。
    short compX;
    short compY;
    short compZ;
};

struct SensorInf {
    int ofz[4];
    float seat;
    int cntS[5];
    float distance;
};

struct ConfigInf {
    int data[21];
};

struct selectLogInf {
    bool start;
    bool drvInf;
    bool strInf;
    bool brkInf;
    bool battInf;
    bool otherInf;
};

class ChangeConfig
{
public:
    virtual void UpdateConfig(int num, int index, int data[]) = 0;
};

class HevCnt : public ChangeStateObserver
{
public:
    HevCnt();
    virtual ~HevCnt();

    bool Init(); //初始化HEV控制系统并检查连接是否正常。
    bool Start(); //启动HEV控制系统并开始自动控制。
    bool Process(); //处理HEV控制系统的输入输出，并进行自动控制。
    bool Stop(); // 停止HEV控制系统的自动控制，并进行必要的清理。
    bool Close(); //关闭HEV控制系统并释放资源。
    bool SetConfigCallback(ChangeConfig* callback); //设置用于配置更改的回调函数。

    void GetBattInf(BattInf* batt); //获取电池信息。
    void GetBrakeInf(BrakeInf* brake); //获取刹车信息。
    void GetOtherInf(OtherInf* other); //获取其他车辆信息。
    void GetDrvInf(DrvInf* drv); //获取驱动信息。
    void GetStrInf(StrInf* str); //获取转向信息。
    void GetErrCode(int* leve, int* err); //获取错误码和错误级别。
    void GetFirmVersion(char* firm_versin); //获取HEV控制系统的固件版本号。
//    void GetSensInf(SensorInf* sens);

    // Set Steer
    void SetStrMode(int mode); //设置转向模式。
    void SetStrCMode(int cmode); //设置转向控制模式。
    void SetStrOMOde(int omode); //设置转向覆盖模式。
    void SetStrTorque(int torque); //设置转向扭矩。
    void SetStrAngle(int angle); //设置转向角度。
    void SetStrServo(int servo); //设置转向电机的电流输出。

    // Set Drive
    void SetDrvMode(int mode); //设置驱动模式。
    void SetDrvCMode(int cmode); //设置驱动控制模式。
    void SetDrvOMode(int omode); //设置驱动覆盖模式。
    void SetDrvStroke(int stroke); //设置驱动踏板行程。
    void SetDrvVeloc(int veloc); //设置驱动目标速度。
    void SetDrvShiftMode(int shift); //设置驱动换挡模式。
    void SetDrvServo(int servo); //设置驱动电机的电流输出。

    // Set Brake
    void SetBrakeStroke(int stroke); //设置刹车踏板行程。
    void SetBrakeLamp(unsigned char lamp); //设置刹车
    void SetBlinkerLeft(unsigned char blink_left);
    void SetBlinkerRight(unsigned char blink_right); //设置右转向灯状态。
    void SetBrakeMode(unsigned char mode); //设置刹车模式。

    // Set Other
    void SetControlGain(int index, int gain);
    void SetDist(float dist);
    void SetTargetAngle(int target);
    void SndDiagReq(HEV_ECU kind);
    void SndDiagClear(HEV_ECU kind);
    void SndErrReq();
    void SndErrClear();
    void SndVersionReq();

    void GetConfig(HEV_CONFIG kind);
    void SetConfig(HEV_CONFIG kind, int val);
    void SetConfig(HEV_CONFIG kind, float val);
    void SaveConfig();

   // Set log
//    void SetlogEnable(selectLogInf select);

private:
    void UpdateSteerState(REP_STEER_INFO_INDEX index);
    void UpdateDriveState(REP_DRIVE_INFO_INDEX index);
    void UpdateBattState(REP_BATT_INFO_INDEX index);
    void UpdateOtherState(REP_OTHER_INFO_INDEX index);
    void UpdateDemoSensorState(REP_DEMO_SENSOR_INFO_INDEX index);
    void ReceiveConfig(int num, int index, int value[]);
    void ReceiveErrorStatus(int leve, int errCode);
    void ReceiveEcho(int ctlKind, int ctlNo);
    void ReceiveVersion(char c0, char c1, char c2, char c3, char c4, char c5, char c6, char c7);
    void ReceiveImuMsg(REP_IMU_INFO_INDEX index);



    void rcvTime(char* date);
    void getDate();

    void SetSeat(int seat);
    void SetOfz(int index, int val);
    void SetCnts(int index);

    int GetSeat();
    int GetOfz(int index);
    int GetCnts(int index);
    float GetDist();

    HevControl* _hevCnt;
    CANUSBZ* _canCom;
    ChangeConfig* _callback;

    int _errCode;
    int _errLevel;
    BattInf _battInf;
    BrakeInf _brakeInf;
    OtherInf _otherInf;
    DrvInf _drvInf;
    StrInf _strInf;
    int _temp[100];
    SensorInf _sensInf;
    ConfigInf _config;

    int _beforAngle;
    int _targetCnt;
    int _targetAngle;
    
    char _firm_version[9]; //8chars

    selectLogInf _selectLog;
    FILE* _logSave;
    struct tm *_s_time;
    time_t _day_time;
    timeval _getTime;

    int _asistTrq;
};

