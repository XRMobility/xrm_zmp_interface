/**
 * @file		CANConst.h
 * @brief		CANI/Fライブラリの変数定義
 *
 * @date		2013/07/25
 * @author		k.sekiguchi
 * @par			Copyright:
 *				2013 XXX All rights reserved.
 */

#ifndef CANCONST_H_
#define CANCONST_H_

#define DEVICE_CAN0 "/dev/pcan32"
#define DEVICE_CAN1 "/dev/pcan33"

/**
 * @brief CANメッセージ構造体
 */
typedef struct
{
    int ID;                /*!< CAN ID */
    unsigned char LEN;     /*!< DATA length */
    unsigned char DATA[8]; /*!< DATA */
} CANMsg;

/**
 * @brief CANメッセージIDのメッセージグループ(上位5bit)
 */
enum CAN_MSG_GROUP
{
    MSG_GROUP_PRIORITY = 0x00, /*!< Priorityメッセージグループ */
    MSG_GROUP_DRIVE = 0x01,    /*!< ドライブメッセージグループ */
    MSG_GROUP_STEER = 0x02,    /*!< ステアリングメッセージグループ */
    MSG_GROUP_BREAK = 0x03,    /*!< ブレーキメッセージグループ */
    MSG_GROUP_BATTERY = 0x04,  /*!< バッテリーメッセージグループ */
                               // #ifdef USE_DEMO
    MSG_GROUP_DEMO_SENSOR_OFZ0 = 0x0c,
    MSG_GROUP_DEMO_SENSOR_OFZ1 = 0x0d,
    MSG_GROUP_DEMO_SENSOR_OFZ2 = 0x0e,
    MSG_GROUP_DEMO_SENSOR_OFZ3 = 0x0f,
    MSG_GROUP_DEMO_SENSOR_SONAR = 0x10,
    // #endif
    MSG_GROUP_OPTION = 0x05,
    MSG_GROUP_IMU = 0x08,
    MSG_GROUP_POSZ = 0x0a,
    MSG_GROUP_INCLINO = 0x0b,
    MSG_GROUP_COMMON = 0x18, /*!< システム共通メッセージグループ */
};

/**
 * @brief CAN PRIORITYメッセージ
 */
enum CAN_MSG_PRIORITY
{
    MSG_PRIORITY_ERROR = 0x01, /*!< エラーメッセージ */
};

/**
 * @brief CAN DRIVEメッセージ
 */
enum CAN_MSG_DRIVE
{
    MSG_DRIVE_SET_MODE = 0x01,          /*!< モード切替メッセージ */
    MSG_DRIVE_SET_CONTROL_MODE = 0x02,  /*!< 制御モード切替メッセージ */
    MSG_DRIVE_SET_OVERRIDE_MODE = 0x03, /*!< オーバライド切替メッセージ */
    MSG_DRIVE_SET_SERVO = 0x04,         /*!< サーボONOFFメッセージ */
    MSG_DRIVE_SET_VELOCITY = 0x05,      /*!< ドライブ速度指示メッセージ */
    MSG_DRIVE_SET_PEDAL = 0x06,         /*!< ドライブペダル指示メッセージ */
    MSG_DRIVE_SET_SHIFT_POS = 0x07,     /*!< シフト切替指示メッセージ */
    MSG_DRIVE_REP_MODE = 0x21,          /*!< ドライブ内部状態通知（制御モード）メッセージ */
    MSG_DRIVE_REP_PEDAL = 0x22,         /*!< ドライブ内部状態通知（ペダル指示）メッセージ */
    MSG_DRIVE_REP_SHIFT_POS = 0x23,     /*!< ドライブ内部状態通知（シフト）メッセージ */
    MSG_DRIVE_REP_VELOCITY = 0x24,      /*!< ドライブ内部状態通知（速度指示）メッセージ */

};

/**
 * @brief CAN STEERINGメッセージ
 */
enum CAN_MSG_STEER
{
    MSG_STEER_SET_MODE = 0x01,          /*!< ステアリングモード切替メッセージ */
    MSG_STEER_SET_CONTROL_MODE = 0x02,  /*!< ステアリング制御モード切替メッセージ */
    MSG_STEER_SET_OVERRIDE_MODE = 0x03, /*!< ステアリングオーバライドモード切替メッセージ */
    MSG_STEER_SET_SERVO = 0x04,         /*!< ステアリングサーボONOFメッセージF */
    MSG_STEER_SET_TORQUE = 0x05,        /*!< ステアリングトルク指示メッセージ */
    MSG_STEER_SET_ANGLE = 0x06,         /*!< ステアリング角度指示メッセージ */
    MSG_STEER_REP_MODE = 0x21,          /*!< ステアリング内部状態通知（制御モード）メッセージ */
    MSG_STEER_REP_ANGLE = 0x22,         /*!< ステアリング内部状態通知（角度制御）メッセージ */
    MSG_STEER_REP_TORQUE = 0x23,        /*!< ステアリング内部状態通知（トルク制御）メッセージ */
    //    MSG_STEER_REP_TORQUE_RAW	= 0x24	/*!< ステアリング内部状態通知（トルクセンサ）メッセージ */
};

/**
 * @brief CAN BRAKEメッセージ
 */
enum CAN_MSG_BRAKE
{
    MSG_BRAKE_SET_PEDAL = 0x01,        /*!< ペダル指示メッセージ */
    MSG_BRAKE_SET_TAIL_LAMP = 0x02,    /*!< brake lamp ON/OFF */
    MSG_BRAKE_SET_BLINKER_LEFT = 0x03, /*!< left blinker ON/OFF */

    MSG_BRAKE_SET_BLINKER_RIGHT = 0x04, /*!< right blinker ON/OFF */
    MSG_BRAKE_SET_MODE = 0x05,          /*!< brake lamp control mode */
    MSG_BRAKE_REP_PEDAL = 0x21,         /*!< ブレーキ内部状態通知（ペダル指示）メッセージ */
                                        //    MSG_BRAKE_REP_PEDAL_RAW	= 0x22,	/*!< ブレーキ内部状態通知（ペダルセンサ）メッセージ */
                                        //    MSG_BRAKE_REP_RAW	= 0x23,	/*!< ブレーキ内部状態通知（ブレーキセンサ）メッセージ */
    MSG_BRAKE_REP_STATUS = 0x24,        /*!< lamp/blinker/mode status */

};

/**
 * @brief CAN BATTERYメッセージ
 */
enum CAN_MSG_BATTERY
{
    //    MSG_BATTERY_REP_SOC		= 0x21, /*!< SOC通知メッセージ */
    //    MSG_BATTERY_REP_STATUS	= 0x22, /*!< 充電状態通知メッセージ */
    //    MSG_BATTERY_REP_CHG_CURRENT	= 0x23, /*!< 充電電流通知メッセージ */
    //    MSG_BATTERY_REP_TEMP	= 0x24, /*!< バッテリ温度通知メッセージ */
    //    MSG_BATTERY_REP_LEVEL	= 0x25, /*!< 充電レベル通知メッセージ */
    MSG_BATTERY_REP_MAIN_CURRENT = 0x26, /*!< Mainバッテリ電流通知メッセージ */
    MSG_BATTERY_REP_SUB_CURRENT = 0x27,  /*!< Subバッテリ電流通知メッセージ */
    MSG_BATTERY_REP_MAIN_VOLTAGE = 0x28, /*!< Mainバッテリ電圧通知メッセージ */
    MSG_BATTERY_REP_SUB_VOLTAGE = 0x29,  /*!< Subバッテリ電圧通知メッセージ */
};

enum CAN_MSG_OPTION
{
    MSG_OPTION_REP_WHEEL_ENC = 0x21,
};

/**
 * @brief IMU-Zメッセージ
 */
enum IMU_MSG_ID
{
    IMU_REQ_ECHO = 0x01, /*!< Echo要求 */
    IMU_REQ_STATUS = 0x05,
    IMU_SET_PERIOD = 0x07,
    IMU_SET_RANGE = 0x08,
    IMU_RESET_TIMESTAMP = 0x0b,
    IMU_SET_MEASUREMENT_STATE = 0x0c,
    IMU_REQ_DEVICE_PROFILE = 0x0d,
    IMU_RES_ECHO = 0x21,
    IMU_REP_MEASUREMENT_ACC = 0x22,
    IMU_REP_MEASUREMENT_GYRO = 0x23,
    IMU_REP_MEASUREMENT_COMP = 0x24,
    IMU_RES_STATUS = 0x25,
    IMU_RES_DEVICE_PROFILE = 0x2d,
};

/**
 * @brief IMU 加速度レンジ種別
 */
enum IMU_RANGE_ACC
{
    RANGE_ACC_2G = 0,
    RANGE_ACC_4G,
    RANGE_ACC_8G,
};

/**
 * @brief IMU ジャイロレンジ種別
 */
enum IMU_RANGE_GYRO
{
    RANGE_GYRO_30DPS = 0,
    RANGE_GYRO_120DPS,
};

/**
 * @brief IMU 地磁気レンジ種別
 */
enum IMU_RANGE_COMP
{
    RANGE_COMP_07GA = 0,
    RANGE_COMP_10GA,
    RANGE_COMP_15GA,
    RANGE_COMP_20GA,
    RANGE_COMP_32GA,
    RANGE_COMP_38GA,
    RANGE_COMP_45GA,
};

// #ifdef USE_DEMO
enum CAN_MSG_DEMO_SENSOR
{
    MSG_REP_OFZ_AVERAGE = 0x38,
    MSG_REP_SONAR = 0x30,
};
// #endif

/**
 * @brief CAN SYSTEM COMMONメッセージ
 */
enum CAN_MSG_COMMON
{
    MSG_COMMON_REQ_ECHO = 0x01,         /*!< エコー要求 */
    MSG_COMMON_READ_CONFIG = 0x02,      /*!< コンフィグ読み出しメッセージ */
    MSG_COMMON_SET_CONFIG = 0x03,       /*!< コンフィグ設定メッセージ */
    MSG_COMMON_SAVE_CONFIG = 0x04,      /*!< コンフィグ書取得き込みメッセージ */
    MSG_COMMON_REQ_ERROR_STATUS = 0x05, /*!< エラー状態の取得 */
                                        //    MSG_COMMON_MAINTAIN_MODE	= 0x06, /*!< メンテナンスモードON */
    MSG_COMMON_REQ_VERSION = 0x07,      /*!< version要求 */
    MSG_COMMON_RES_ECHO = 0x21,         /*!< エコー応答 */
    MSG_COMMON_RES_CONFIG = 0x22,       /*!< コンフィグ読み出し応答メッセージ */
    MSG_COMMON_RES_ERROR_STATUS = 0x23, /*!< エラー状態の取得応答 */
    MSG_COMMON_RES_VERSION = 0x24,      /*!< version応答 */
};

enum CAN_MSG_SYSCOM
{
    MSG_SYSCOM_REQ_ECHO = 0x01,
    MSG_SYSCOM_READ_CONFIG = 0x02, /*!< コンフィグ読み出しメッセージ */
    MSG_SYSCOM_SET_CONFIG = 0x03,  /*!< コンフィグ設定メッセージ */
    MSG_SYSCOM_SAVE_CONFIG = 0x04, /*!< コンフィグ書き込みメッセージ */

    MSG_SYSCOM_REQ_ERR = 0x05,
    MSG_SYSCOM_REQ_MAINTAINANCE = 0x06,

    MSG_SYSCOM_RES_ECHO = 0x21,
    MSG_SYSCOM_RES_CONFIG = 0x22, /*!< コンフィグ読み出し応答メッセージ */
    MSG_SYSCOM_RES_ERR = 0x23,
    MSG_SYSCOM_REP_SWITCH = 0x24,
};

enum CAN_MSG_IMUZ2
{
    MSG_IMUZ2_REQ_ECHO = 0x01,
    MSG_IMUZ2_REQ_STATUS = 0x05,
    MSG_IMUZ2_SET_ROLE = 0x06,
    MSG_IMUZ2_SET_PERIOD = 0x07,
    MSG_IMUZ2_SET_RANGE = 0x08,
    MSG_IMUZ2_SET_NODE_NO = 0x09,
    MSG_IMUZ2_SAVE = 0x0a,
    MSG_IMUZ2_RESET_TIMESTAMP = 0x0b,
    MSG_IMUZ2_SET_MEASUREMENT_STATE = 0x0c,
    MSG_IMUZ2_REQ_DEVICE_PROFILE = 0x0d,
    MSG_IMUZ2_SET_FACTORY_RESET = 0x0e,
    MSG_IMUZ2_SET_BINARY = 0x0f,
    MSG_IMUZ2_RES_ECHO = 0x21,
    MSG_IMUZ2_REP_MEASUREMENT_ACC = 0x22,
    MSG_IMUZ2_REP_MEASUREMENT_GYRO = 0x23,
    MSG_IMUZ2_REP_MEASUREMENT_COMP = 0x24,
    MSG_IMUZ2_RES_STATUS = 0x25,
    MSG_IMUZ2_RES_DEVICE_PROFILE = 0x2d,
    MSG_IMUZ2_REP_GPS = 0x30,
    MSG_IMUZ2_REP_PRESSURE = 0x31,
    MSG_IMUZ2_REP_HUMIDITY = 0x32,
    MSG_IMUZ2_RES_STATUS_2 = 0x33,
};

enum CAN_MSG_INCLINO
{
    MSG_INCLINO_REP_ACC = 0x22,
    MSG_INCLINO_RES_STATUS = 0x25,
    MSG_INCLINO_REP_INCLINO = 0x33,
};

/**
 * @brief CANデバイスチャネル
 */
enum CAN_DEVICE_CHANNEL
{
    CAN_CHANNEL_0, /*!< CAN 0 */
    CAN_CHANNEL_1  /*!< CAN 1 */
};

/**
 * @brief CAN bit rate 種別
 */
enum CAN_BITRATE
{
    CAN_BITRATE_1M = 0x0014,   /*!< 1Mbps */
    CAN_BITRATE_500K = 0x001C, /*!< 500kbps */
    CAN_BITRATE_250K = 0x011C, /*!< 250kbps */
    CAN_BITRATE_125K = 0x031C, /*!< 125kbps */
    CAN_BITRATE_100K = 0x432F, /*!< 100kbps */
    CAN_BITRATE_50K = 0x472F,  /*!< 50kbps */
    CAN_BITRATE_20K = 0x532F,  /*!< 20kbps */
    CAN_BITRATE_10K = 0x672F,  /*!< 10kbps */
    CAN_BITRATE_5K = 0x7F7F    /*!< 5kbps */
};

#endif /* CANCONST_H_ */
