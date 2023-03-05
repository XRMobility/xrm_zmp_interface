/**
 * @file	HEVConst.h
 * @brief	HEVライブラリの変数定義
 *
 * @date	2012/02/07
 * @author	m.ota
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */

#ifndef HEVCONST_H_
#define HEVCONST_H_

namespace zmp
{
    namespace minivan
    {
        /**
         * @brief ドライブ状態通知Index
         */
        enum REP_DRIVE_INFO_INDEX
        {
            REP_DRV_MODE,            /*!< モード変更 */
            REP_GAS_PEDAL,           /*!< アクセルペダル */
            REP_GAS_PEDAL_FROMOBD,   /*!< アクセルペダル(OBD) */
            REP_VELOCITY,            /*!< 速度 */
            REP_VELOCITY_FROMOBD,    /*!< 速度(OBD) */
            REP_VELOCITY_FROMOBD2,   /*!< 速度2(OBD) */
            REP_WHEEL_VELOCITY_F,    /*!< 4輪速度 */
            REP_WHEEL_VELOCITY_R,    /*!< 4輪速度 */
            REP_BRAKE_PEDAL,         /*!< ブレーキペダル */
            REP_BRAKE_PEDAL_FROMOBD, /*!< ブレーキペダル(OBD) */
            REP_BRAKE_STATUS,        /*!< brake status */
            REP_SHIFT_POS,           /*!< シフトポジション */
            REP_SHIFT_POS_FROMOBD,   /*!< シフトポジション(OBD) */
            REP_HEV_MODE,            /*!< PRIUSのモード */
            REP_ICE_RPM,             /*!< エンジン回転数 */
            REP_ICE_COOLANT_TEMP,    /*!< エンジン水温 */
            REP_ACCELERLATION,       /*!< 加速度 */
            REP_SIDE_ACCELERLATION,  /*!< Yawレート */
            REP_DRIVE_MODE,          /*!< ドライブモード(OBD) */
            REP_CRUISE_STATE,        /*!< クルーズ状態 */
            REP_DTC_STATUS,          /*!< ECU状態 */
        };

        /**
         * @brief ステアリング状態通知Index
         */
        enum REP_STEER_INFO_INDEX
        {
            REP_STR_MODE,      /*!< モード変更 */
            REP_TORQUE,        /*!< トルク */
            REP_ANGLE,         /*!< 角度 */
            REP_ANGLE_FROMOBD, /*!< 角度(OBD) */
        };

        /**
         * @brief バッテリ状態通知Index
         */
        enum REP_BATT_INFO_INDEX
        {
            REP_BATT_INFO,         /*!< バッテリ状態(チャージ最大電流/ディスチャージ最大電流/最高温度/最低温度/state of charge */
            REP_BATT_INFO_CURRENT, /*!< バッテリ状態(電流) */
            REP_BATT_INFO_VOLT     /*!< バッテリ状態(電圧) */
        };

        /**
         * @brief 他状態通知Index
         */
        enum REP_OTHER_INFO_INDEX
        {
            REP_LIGHT_STATE, /*!< ヘッドライト */
            REP_GAS_LEVEL,   /*!< ガソリン残量 */
            REP_DOOR_STATE   /*!< ドア開閉状態 */
        };
        // #ifdef USE_DEMO
        /**
         * @brief
         */
        enum REP_DEMO_SENSOR_INFO_INDEX
        {
            REP_OFZ0,
            REP_OFZ1,
            REP_OFZ2,
            REP_OFZ3,
            REP_SEAT_SENSOR,
        };
        // #endif

        /**
         * @brief IMUメッセージID
         */
        enum REP_IMU_INFO_INDEX
        {
            RES_ECHO,
            REP_ACC,
            REP_GYRO,
            REP_COMP,
            RES_STATUS,
            RES_DEVICE_PROFILE,
        };

        /**
         * @brief シフトポジション
         */
        enum SHIFT_POSITION
        {
            SHIFT_POS_B = 0x00,       /*!< シフトポジション B */
            SHIFT_POS_D = 0x10,       /*!< シフトポジション D */
            SHIFT_POS_N = 0x20,       /*!< シフトポジション N */
            SHIFT_POS_R = 0x40,       /*!< シフトポジション R */
            SHIFT_POS_P = 0x80,       /*!< シフトポジション P */
            SHIFT_POS_START = 0x0f,   /*!< シフトポジション Start */
            SHIFT_POS_UNKNOWN = 0xff, /*!< シフトポジション Unknown */
        };

        /**
         * @brief モード
         */
        enum HEV_MODE
        {
            MODE_MANUAL = 0x00, /*!< マニュアルモード */
            MODE_PROGRAM = 0x10 /*!< プログラムモード */
        };

        /**
         * @brief ステアリングコントロールモード
         */
        enum STEER_CONTROL_MODE
        {
            CONT_MODE_TORQUE = 0x00, /*!< トルク制御モード */
            CONT_MODE_ANGLE = 0x10   /*!< 角度制御モード */
        };

        /**
         * @brief ドライブコントロールモード
         */
        enum DRIVE_CONTROL_MODE
        {
            CONT_MODE_STROKE = 0x00,  /*!< ペダル制御モード */
            CONT_MODE_VELOCITY = 0x10 /*!< 速度制御モード */
        };

        /**
         * @brief オーバーライドモード
         */
        enum OVERRIDE_MODE
        {
            OVERRIDE_MODE_ON = 0x00, /*!< オーバーライドモードON */
            OVERRIDE_MODE_OFF = 0x10 /*!< オーバーライドモードOFF */
        };

        /**
         * @brief ドアの開閉状態
         */
        enum DOOR_STATE
        {
            // DOOR_OPEN_DR	= 0x20,	/*!< 運転席ドアがOPEN */
            // DOOR_OPEN_PASS	= 0x10,	/*!< 助手席ドアがOPEN */
            // DOOR_OPEN_REAR_R	= 0x08,	/*!< 後席右ドアがOPEN */
            // DOOR_OPEN_REAR_L	= 0x04,	/*!< 後席左ドアがOPEN */
            // DOOR_OPEN_BACK	= 0x02,	/*!< BackドアがOPEN */
            // DOOR_CLOSE	= 0x00	/*!< ドアがCLOSE */
            // 組み合わせはORする
            DOOR_CLOSE = 0x00,
            DOOR_OPEN_BACK = 0x02,
            DOOR_OPEN_REAR_L = 0x04,
            DOOR_OPEN_REAR_L_BACK = 0x06,
            DOOR_OPEN_REAR_R = 0x08,
            DOOR_OPEN_REAR_R_BACK = 0x0A,
            DOOR_OPEN_REAR_R_REAR_L = 0x0C,
            DOOR_OPEN_REAR_R_REAR_L_BACK = 0x0E,
            DOOR_OPEN_PASS = 0x10,
            DOOR_OPEN_PASS_BACK = 0x12,
            DOOR_OPEN_PASS_REAR_L = 0x14,
            DOOR_OPEN_PASS_REAR_L_BACK = 0x16,
            DOOR_OPEN_PASS_REAR_R = 0x18,
            DOOR_OPEN_PASS_REAR_R_BACK = 0x1A,
            DOOR_OPEN_PASS_REAR_R_REAR_L = 0x1C,
            DOOR_OPEN_PASS_REAR_R_REAR_L_BACK = 0x1E,
            DOOR_OPEN_DR = 0x20,
            DOOR_OPEN_DR_BACK = 0x22,
            DOOR_OPEN_DR_REAR_L = 0x24,
            DOOR_OPEN_DR_REAR_L_BACK = 0x26,
            DOOR_OPEN_DR_REAR_R = 0x28,
            DOOR_OPEN_DR_REAR_R_BACK = 0x2A,
            DOOR_OPEN_DR_REAR_R_REAR_L = 0x2C,
            DOOR_OPEN_DR_REAR_R_REAR_L_BACK = 0x2E,
            DOOR_OPEN_DR_PASS = 0x30,
            DOOR_OPEN_DR_PASS_BACK = 0x32,
            DOOR_OPEN_DR_PASS_REAR_L = 0x34,
            DOOR_OPEN_DR_PASS_REAR_L_BACK = 0x36,
            DOOR_OPEN_DR_PASS_REAR_R = 0x38,
            DOOR_OPEN_DR_PASS_REAR_R_BACK = 0x3A,
            DOOR_OPEN_DR_PASS_REAR_R_REAR_L = 0x3C,
            DOOR_OPEN_DR_PASS_REAR_R_REAR_L_BACK = 0x3E
        };

        /**
         * @brief ヘッドライトの点灯状態
         */
        enum LIGHT_STATE
        {
            LIGHT_OFF = 0x00,     /*!< ライトOFF */
            LIGHT_PARK = 0x10,    /*!< スモールライト点灯 */
            LIGHT_ON = 0x30,      /*!< ライトON */
            LIGHT_HIGH = 0x60,    /*!< ハイビーム点灯 */
            LIGHT_ON_HIGH = 0x70, /*!< ハイビーム点灯 */
        };

        /**
         * @brief PriusのECU種別
         */
        enum HEV_ECU
        {
            ENGINE_ECU,  /*!< エンジンECU */
            HV_ECU,      /*!< ハイブリッドECU */
            BRAKE_ECU,   /*!< クルーズコントロール（ブレーキ）ECU */
            BATTERY_ECU, /*!< バッテリーECU */
        };

        /**
         * @brief Config種別
         */
        enum HEV_CONFIG
        {
            CONFIG_ANGLE_CTRL_KP = 100,           /*!< ステア角制御ゲインKp */
            CONFIG_ANGLE_CTRL_KI = 101,           /*!< ステア角制御ゲインKi */
            CONFIG_ANGLE_CTRL_KD = 102,           /*!< ステア角制御ゲインKd */
            CONFIG_ANGLE_CTRL_DEC_TIME = 103,     /*!< ステア角制御パラメータ  積分値の減衰時間 */
            CONFIG_ANGLE_CTRL_WINDOW = 104,       /*!< ステア角制御パラメータ  ウィンドウ幅 */
            CONFIG_ANGLE_CTRL_PUNCH = 105,        /*!< ステア角制御パラメータ  パンチ */
            CONFIG_ANGLE_CTRL_OUTPUT_MAX = 106,   /*!< ステア角制御パラメータ  最大トルク出力 */
            CONFIG_ANGLE_CTRL_CCW_FACTOR = 107,   /*!< ステア角制御パラメータ  左右差吸収ファクタ */
            CONFIG_VELOC_CTRL_MAX_SPEED = 108,    /*!< 速度制御  最大指定速度 */
            CONFIG_VELOC_CTRL_KP_P = 109,         /*!< 速度制御ゲインKp  プラス側 */
            CONFIG_VELOC_CTRL_KP_M = 110,         /*!< 速度制御ゲインKp  マイナス側 */
            CONFIG_VELOC_CTRL_WINDOW_P = 111,     /*!< 速度制御パラメータ  ウィンドウ幅  プラス側 */
            CONFIG_VELOC_CTRL_WINDOW_M = 112,     /*!< 速度制御パラメータ  ウィンドウ幅  マイナス側 */
            CONFIG_VELOC_CTRL_PUNCH_P = 113,      /*!< 速度制御パラメータ  パンチ  プラス側 */
            CONFIG_VELOC_CTRL_PUNCH_M = 114,      /*!< 速度制御パラメータ  パンチ  マイナス側 */
            CONFIG_VELOC_CTRL_OUTPUT_MAX_P = 115, /*!< 速度制御パラメータ  最大アクセル出力 */
            CONFIG_VELOC_CTRL_OUTPUT_MAX_M = 116, /*!< 速度制御パラメータ  最大ブレーキ出力 */
            CONFIG_OVERRIDE_STEER_TH = 117,       /*!< ステアオーバーライド閾値 */
            CONFIG_OVERRIDE_ACCEL_TH = 118,       /*!< アクセルオーバーライド閾値 */
            CONFIG_OVERRIDE_BRAKE_TH = 119,       /*!< ブレーキオーバーライド閾値 */
            TEST_FLAGS = 120,                     /*!< テストモード動作(ホストの生存確認チェック無効) */
            CONFIG_EMERGENCY_BRAKE_RATE = 122,    /*!< 緊急停止時のブレーキ強度 */
        };
    }
}

#endif /* HEVCONST_H_ */
