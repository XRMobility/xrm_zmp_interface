/**
 * @file	HevControl.h
 * @brief	HEV制御クラスヘッダファイル
 *
 * @date	2012/02/06
 * @author	m.ota
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */
#ifndef HEVCONTROL_H_
#define HEVCONTROL_H_

#include <math.h>
#include "DriveInformation.h"
#include "SteeringInformation.h"
#include "BrakeInformation.h"
#include "BatteryInformation.h"
#include "OtherInformation.h"
#include "ImuInformation.h"
// #ifdef USE_DEMO
#include "DemoSensorInformation.h"
// #endif
// #include "CanCommunication.h"
#include "HevControlCommon.h"
// #include "CANConst.h"
#include "CANUSB.h"
#include "HEVConst.h"

/*! @brief HEV接続チャンネルの定義 */
#define CAN_CHANNEL_HEV CAN_CHANNEL_0 // 500kbps

/*! @brief コントローラ接続チャンネルの定義 */
#define CAN_CHANNEL_CONTROLLER CAN_CHANNEL_1 // 1000kbps

#define HEV_LIB_VERSION "01.00.00" //

namespace zmp
{
    namespace minivan
    {
        // #ifdef USE_DEMO
        class OfzBuff
        {
        private:
            unsigned char _byteData[2 + 24];
            unsigned int _time;
            bool _valid;
            int _mno;

        public:
            unsigned char *GetDataPtr()
            {
                return _byteData;
            }
            bool AddData(const CANMsg *msg)
            {
                int mno = msg->DATA[0];
                if (mno == 0)
                {
                    _byteData[0] = msg->DATA[1];
                    _byteData[1] = msg->DATA[2];
                    _time = (unsigned int)(_byteData[0] << 8 | _byteData[1]);

                    for (int i = 0; i < 4; i++)
                    {
                        _byteData[2 + i] = msg->DATA[3 + i];
                    }
                    _mno = mno;
                    _valid = true;
                }
                else if (1 <= mno && mno <= 5)
                {
                    if (_valid && mno == _mno + 1 &&
                        (unsigned int)(msg->DATA[1] << 8 | msg->DATA[2]) == _time)
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            _byteData[2 + mno * 4 + i] = msg->DATA[3 + i];
                        }
                        _mno = mno;
                    }
                    else
                    {
                        _valid = false;
                    }
                }
                if (mno == 5)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        };
        // #endif

        /**
         * @brief HEVの状態変化通知コールバック関数を提供するオブザーバクラス
         *
         * @author		m.ota
         * @date		2012/02/15
         */
        class ChangeStateObserver
        {
        public:
            /**
             * @brief	ステアリング状態通知コールバック関数
             *
             * @param	index 通知インデックス
             */
            virtual void UpdateSteerState(REP_STEER_INFO_INDEX index) = 0;

            /**
             * @brief	ドライブ状態通知コールバック関数
             *
             * @param	index 通知インデックス
             */
            virtual void UpdateDriveState(REP_DRIVE_INFO_INDEX index) = 0;

            /**
             * @brief	バッテリ状態通知コールバック関数
             *
             * @param	index 通知インデックス
             */
            virtual void UpdateBattState(REP_BATT_INFO_INDEX index) = 0;

            /**
             * @brief	他状態通知コールバック関数
             *
             * @param	index 通知インデックス
             */
            virtual void UpdateOtherState(REP_OTHER_INFO_INDEX index) = 0;

            /**
             * @brief	IMUメッセージ受信コールバック関数
             *
             * @param	index 通知インデックス
             */
            virtual void ReceiveImuMsg(REP_IMU_INFO_INDEX index) = 0;

            // #ifdef USE_DEMO
            virtual void UpdateDemoSensorState(REP_DEMO_SENSOR_INFO_INDEX index) = 0;
            // #endif
            /**
             * @brief	コンフィグ応答コールバック関数
             *
             * @param	num コンフィグ数
             * @param	index コンフィグの開始INDEX
             * @param	value[] コンフィグ
             */
            virtual void ReceiveConfig(int num, int index, int value[]) = 0;

            /**
             * @brief	エラー通知コールバック関数
             *
             * @param	level エラーレベル
             * @param	error_code エラーコード
             */
            virtual void ReceiveErrorStatus(int level, int error_code) = 0;
            /**
             * @brief	Echo通知コールバック関数
             *
             * @param	kind 装置種別
             * @param	no 装置番号
             */
            virtual void ReceiveEcho(int kind, int no) = 0;
            /**
             * @brief	Version通知コールバック関数
             *
             * @param	firm_version
             */
            virtual void ReceiveVersion(char c0, char c1, char c2, char c3, char c4, char c5, char c6, char c7) = 0;
        };

        /**
         * @brief HEVをコントロールするためのクラス
         *
         * CAN0を介したOBDの情報取得、CAN1を介したコントロールを行うAPI群を提供する
         *
         * @author	m.ota
         * @date	2012/02/15
         */
        class HevControl : public SerialReceiveHandler
        {
        public:
            /** コンストラクタ */
            HevControl();

            /** デストラクタ */
            virtual ~HevControl();

            /**
             * @brief	HEVコントロールクラスの初期化
             * 	引数で渡されるCanCommunicationクラスは、初期化されていなければならない。
             * @param[in]    can_comm CanCommunicationクラス
             * @param[in]    devName  CANデバイス名、省略時:/dev/ttyACM0
             */
            int InitHevControl(CANUSBZ *can_comm, char *devName = NULL);

            /**
             * @brief	状態通知用Callback関数をセットする
             * @param[in]	callback callback関数
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int SetStatusCallback(ChangeStateObserver *callback);

            /**
             * @brief	状態通知用Callback関数を除去する
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int RemoveStatusCallback();

            /**
             * @brief	ステアリングの現在モードを取得する
             * @param[out] mode ステアリングモード (manual = 0x00, program = 0x10)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetStrMode(int &mode);

            /**
             * @brief	 ステアリングの制御モードを取得する
             * @param[out] cont_mode ステアリング制御モード(torque = 0x00, angle = 0x10)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetStrControlMode(int &cont_mode);

            /**
             * @brief	ステアリングのオーバーライドモードを取得する
             * @param[out] override_mode オーバーライドモード (ON = 0x00, OFF = 0x10)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetStrOverrideMode(int &override_mode);

            /**
             * @brief	ステアリングの操舵トルクを取得する
             * @param[out]	torque 操舵トルク (-4096〜4095)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetStrTorque(int &torque);

            /**
             * @brief	ステアリング制御の目標操舵トルクを取得する
             * @param[out]	target_torque 目標操舵トルク (-4096〜4095)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetStrTargetTorque(int &target_torque);

            /**
             * @brief	ステアリングの操舵角度を取得する
             * @param[out]	angle 現在角度 [deg.](-666.0～666.0[deg])
             * @param[out]	target 目標角度 [deg.](-666.0～666.0[deg])
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetStrAngle(float &angle, float &target);
            /**
             * @brief	OBDから受信したステアリングの操舵角度を取得する
             * @param[out]	angleFromObd 操舵角度 [deg.](-666.0～666.0[deg])
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetStrAngleFromOBD(float &angleFromObd);

            /**
             * @brief		ステアリング制御の目標操舵角度を取得する
             * @param[out]	target_angle 目標操舵角度[deg.](-6660～6660, 1=0.1[deg])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetStrTargetAngle(float &target_angle);

            /**
             * @brief		ステアリング制御の制御出力を取得する
             * @param[out]	servo 制御のON/OFF (ON = 0x10, OFF = 0x00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetStrServo(int &servo);

            /**
             * @brief		ドライブの現在モードを取得する
             * @param[out]	mode ドライブモード (manual = 0x00, program = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetDrvMode(int &mode);

            /**
             * @brief		ドライブの制御モードを取得する
             * @param[out] cont_mode ドライブ制御モード (velocity = 0x00, stroke = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetDrvControlMode(int &cont_mode);

            /**
             * @brief		ドライブのオーバーライドモードを取得する
             * @param[out] override_mode オーバーライドモード (ON = 0x00, OFF = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetDrvOverrideMode(int &override_mode);

            /**
             * @brief	アクセルペダルのストローク量を取得する
             * @param[out]	actual アクセルペダルストローク現在量 (0～4095)
             * @param[out]	target アクセルペダルストローク目標量 (0～4095)
             * @param[out]	input アクセルペダルストローク入力量 (0～4095)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetGasStroke(int &actual, int &target, int &input);
            /**
             * @brief	OBDから受信したアクセルペダルのストローク現在量を取得する
             * @param[out] actual OBDから受信したアクセルペダルストローク現在量 (0～4095)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetGasStrokeFromOBD(int &output);

            /**
             * @brief	アクセルペダルの目標ストローク量を取得する
             * @param[out] target_output アクセルペダルの目標ストローク量 (0～4095)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetGasTargetStroke(int &target_output);

            /**
             * @brief	ブレーキペダルのストローク量を取得する
             * @param[out]	actual ブレーキペダルの現在ストローク量(0～4095)
             * @param[out]	target ブレーキペダルの目標ストローク量(0～4095)
             * @param[out]	input ブレーキペダルからの入力量(0～4095)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetBrakeStroke(int &actual, int &target, int &input);
            /**
             * @brief	OBDから受信したブレーキペダルのストローク量を取得する
             * @param[out]	stroke ブレーキペダルの現在ストローク量(0～4095)
             * @param[out]	pressed ブレーキペダルのSW状態(false=OFF, true=ON)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetBrakeStrokeFromOBD(float &stroke, bool &pressed);

            /**
             * @brief	ブレーキペダルの目標ストローク量を取得する
             * @param[out]	target_output ブレーキペダルストローク (0～4095)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetBrakeTargetStroke(int &target_output);

            /**
             * @brief	ブレーキペダルSWのON/OFFを取得する
             * @param[out]	pressed ペダルSWの状態 (ON = true, OFF = false)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetBrakePressState(bool &pressed);

            int GetBrakeStatus(unsigned char &lamp, unsigned char &blinkLeft, unsigned char &blinkRight, unsigned char &mode);

            /**
             * @brief	ドライブ速度を取得する
             * @param[out]	actual 現在速度 [km/h] (0.00～180.00)
             * @param[out]	target 目標速度 [km/h] (0.00〜180.00)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetVeloc(float &actual, float &target);
            /**
             * @brief	OBDから受信したドライブ速度を取得する
             * @param[out]	veloc 現在速度 [km/h] (0.00～180.00)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetVelocFromOBD(float &veloc);
            /**
             * @brief	OBDから受信したドライブ速度を取得する2
             * @param[out]	veloc2 現在速度 [km/h] (0.00～180.00)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetVelocFromOBD2(float &veloc2);

            /**
             * @brief	目標速度を取得する
             * @param[out]	target_veloc 目標速度 [km/h] (0.00～180.00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetTargetVeloc(float &target_veloc);

            /**
             * @brief		前輪速度を取得する
             * @param[out]	veloc_fr 右前輪速度 [km/h](0.00〜180.00)
             * @param[out]	veloc_fl 左前輪速度 [km/h](0.00〜180.00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetWheelVelocF(float &veloc_fr, float &veloc_fl);
            /**
             * @brief		後輪速度を取得する
             * @param[out]	veloc_rr 右後輪速度 [km/h](0.00〜180.00)
             * @param[out]	veloc_rl 左後輪速度 [km/h](0.00〜180.00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetWheelVelocR(float &veloc_rr, float &veloc_rl);

            /**
             * @brief		右前輪速度を取得する
             * @param[out]	veloc_fr 右前輪速度 [km/h](0.00〜180.00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetVelocFR(float &fr);
            /**
             * @brief		左前輪速度を取得する
             * @param[out]	veloc_fl 左前輪速度 [km/h](0.00〜180.00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetVelocFL(float &fl);
            /**
             * @brief		右後輪速度を取得する
             * @param[out]	veloc_rr 右後輪速度 [km/h](0.00〜180.00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetVelocRR(float &rr);
            /**
             * @brief		左後輪速度を取得する
             * @param[out]	veloc_rl 左後輪速度 [km/h](0.00〜180.00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetVelocRL(float &rl);

            /**
             * @brief	シフトスイッチの値を取得する
             * @param[out]	actual シフトポジション現在値(0x00=B, 0x10=D, 0x20=N, 0x40=R, 0x80=P 0x0F=Start 0xFF=Unknown)
             * @param[out]	target シフトポジション指令値(0x00=B, 0x10=D, 0x20=N, 0x40=R, 0x0F=Start)
             * @param[out]	input シフトポジション入力値(0x00=B, 0x10=D, 0x20=N, 0x40=R, 0x80=P, 0x0F=Start, 0xFF=Unknow)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetShiftMode(int &actual, int &target, int &input);
            /**
             * @brief	OBDから受信したシフトスイッチの値を取得する
             * @param[out]	shift シフトポジション現在値(0x00=B, 0x10=D, 0x20=N, 0x40=R)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetShiftModeFromOBD(int &shift);

            /**
             * @brief	シフトスイッチの目標値を取得する
             * @param[out]	shift シフトポジション目標値(0x00=B, 0x10=D, 0x20=N, 0x40=R)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetTargetShift(int &shift);
            /**
             * @brief		ドライブ制御の制御出力を取得する
             * @param[out]	servo 制御のON/OFF (ON = 0x10, OFF = 0x00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetDrvServo(int &servo);

            /**
             * @brief	OBDからの動作モードを取得する
             * @param[out]	drv_mode ドライブモード(0=P, 1=R, 2=N, 3=D, 4=B)
             * @param[out]	ev_mode EVモード(0x00=Normal, 0x40=EVMode, 0x80=EVDenied, other=cancelled)
             * @param[out]	cluise クルーズコントロール(false=OFF, true=ON)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetDrvModeFromOBD(int &drv_mode, int &ev_mode, bool &cluise);
            /**
             * @brief	クルーズコントロール状態を取得する
             * @param[out]	cluise クルーズコントロール(false=OFF, true=ON)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetCruiseControl(bool &cruise);
            /**
             * @brief	ドライブモードを取得する
             * @param[out]	drvMode ドライブモード(0=P, 1=R, 2=N, 3=D, 4=B)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetDriveMode(int &drvMode);
            /**
             * @brief	EVモードを取得する
             * @param[out]	evMode EVモード(0x00=Normal, 0x40=EVMode, 0x80=EVDenied, other=cancelled)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetEvMode(int &evMode);
            /**
             * @brief	ECUエラー状態を取得する
             * @param[out]	data1 エラーデータ1byte目
             * @param[out]	data2 エラーデータ2byte目
             * @param[out]	data3 エラーデータ3byte目
             * @param[out]	data4 エラーデータ4byte目
             * @param[out]	data5 エラーデータ5byte目
             * @param[out]	data6 エラーデータ6byte目
             * @param[out]	data7 エラーデータ7byte目
             * @param[out]	data8 エラーデータ8byte目
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetDtcStatus(char &data1, char &data2, char &data3, char &data4, char &data5, char &data6, char &data7, char &data8);

            /**
             * @brief	エンジンの回転数を取得する
             * @param[out]	rpm 回転数 [rpm]
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetIceRpm(int &rpm);
            /**
             * @brief	車両の加速度を取得する
             * @param[out]	acc 加速度[deg/sec](± 46.0)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetAcc(float &acc);
            /**
             * @brief	車両のYawレートを取得する
             * @param[out]	sideAcc Yawレート[G](± 0.5616)
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int GetSideAcc(float &sideAcc);

            /**
             * @brief		エンジンの冷却液温度を取得する
             * @param[out]	temp 温度 [℃ ] (0～)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetIceCoolantTemp(int &temp);

            /**
             * @brief		メインバッテリ情報を取得する
             * @param[out]	soc 残容量 [%]
             * @param[out]	voltage バッテリ電圧 [V]
             * @param[out]	current 充放電電流 [A * 10]
             * @param[out]	max_temp 最高温度 [℃ ]
             * @param[out]	min_temp 最低温度 [℃ ]
             * @param[out]	max_chg_current 最大充電電流 [A * 10]
             * @param[out]	max_dischg_current 最大放電電流 [A * 10]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBattInfo(float &soc, int &max_temp, int &min_temp, float &max_chg_current, float &max_dischg_current);

            /**
             * @brief		バッテリのstate of chargeを取得する
             * @param[out]	soc 残容量 [%](0.0〜100.0)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBattSoc(float &soc);
            /**
             * @brief		バッテリの電流を取得する
             * @param[out]	current 充放電電流 [A]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBattCurrent(float &current);
            /**
             * @brief		バッテリの最大充電電流を取得する
             * @param[out]	max_chg_current 最大充電電流 [A]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBattCurrentChgMax(float &chgMax);
            /**
             * @brief		バッテリの最大放電電流を取得する
             * @param[out]	max_dischg_current 最大放電電流 [A]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBattCurrentDischgMax(float &dischgMax);
            /**
             * @brief		バッテリの最高温度を取得する
             * @param[out]	max_temp 最高温度 [℃ ]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBattTempMax(int &tempMax);
            /**
             * @brief		バッテリの最低温度を取得する
             * @param[out]	min_temp 最低温度 [℃ ]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBattTempMin(int &tempMin);
            /**
             * @brief		バッテリの電圧を取得する
             * @param[out]	voltage バッテリ電圧 [V]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBattVoltage(int &volt);

            /**
             * @brief		ヘッドライトの状態を取得する
             * @param[out]	light ライトの状態
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetLightState(LIGHT_STATE &light);

            /**
             * @brief		燃料の残量を取得する (0～40)
             * @param[out]	level 燃料の残量[ℓ ]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetGasLevel(int &level);

            /**
             * @brief		ドアの開閉の状態を取得する
             * @param[out]	doors ドアの状態(0x00=CLOSE, 0x04=トランクOPEN, 0x40=助手席OPEN, 0x80=運転席OPEN)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetDoorState(DOOR_STATE &doors);

            /**
             * @brief		IMU加速度のタイムスタンプを取得する
             * @param[out]	加速度のタイムスタンプ(起動からの経過時間[ms])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetAccTime(unsigned short &time);
            /**
             * @brief		IMU加速度X軸の値を取得する
             * @param[out]	加速度X軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetAccX(short &accX);
            /**
             * @brief		IMU加速度Y軸の値を取得する
             * @param[out]	加速度Y軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetAccY(short &accY);
            /**
             * @brief		IMU加速度Z軸の値を取得する
             * @param[out]	加速度Z軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetAccZ(short &accZ);
            /**
             * @brief		IMUジャイロのタイムスタンプを取得する
             * @param[out]	ジャイロのタイムスタンプ(起動からの経過時間[ms])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetGyroTime(unsigned short &time);
            /**
             * @brief		IMUジャイロX軸の値を取得する
             * @param[out]	ジャイロX軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetGyroX(short &gyroX);
            /**
             * @brief		IMUジャイロY軸の値を取得する
             * @param[out]	ジャイロY軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetGyroY(short &gyroY);
            /**
             * @brief		IMUジャイロZ軸の値を取得する
             * @param[out]	ジャイロZ軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetGyroZ(short &gyroZ);
            /**
             * @brief		IMU地磁気のタイムスタンプを取得する
             * @param[out]	地磁気のタイムスタンプ(起動からの経過時間[ms])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetCompTime(unsigned short &time);
            /**
             * @brief		IMU地磁気X軸の値を取得する
             * @param[out]	地磁気X軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetCompX(short &compX);
            /**
             * @brief		IMU地磁気Y軸の値を取得する
             * @param[out]	地磁気Y軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetCompY(short &compY);
            /**
             * @brief		IMU地磁気Z軸の値を取得する
             * @param[out]	地磁気Z軸(raw)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetCompZ(short &compZ);
            /**
             * @brief		IMUの通信方式を取得する
             * @param[out]	通信方式(0=SingleBT, 1=CAN-Master, 2=CAN-Slave))
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetRole(short &role);
            /**
             * @brief		IMUのデータ周期を取得する
             * @param[out]	データ周期(1〜1000(10〜10000[msec]))
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetPeriod(short &period);
            /**
             * @brief		IMU加速度のデータ範囲を取得する
             * @param[out]	データ範囲(0=± 2[G], 1=± 4[G], 2=± 8[G])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetAccRange(char &range);
            /**
             * @brief		IMUジャイロのデータ範囲を取得する
             * @param[out]	データ範囲(0=± 500[deg/sec], 1=± 2000[deg/sec])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetGyroRange(char &range);
            /**
             * @brief		IMU地磁気のデータ範囲を取得する
             * @param[out]	データ範囲(0=± 0.7[Ga], 1=± 1.0[Ga], 2=± 1.5[Ga], 3=± 2.0[Ga], 4=± 3.2[Ga], 5=± 3.8[Ga], 6=± 4.5[Ga])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetCompRange(char &range);
            /**
             * @brief		IMUのバッテリ電圧を取得する
             * @param[out]	バッテリ電圧(0〜255)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetImuBatt(short &batt);
            /**
             * @brief		IMUのフォーマットを取得する
             * @param[out]	フォーマット(0=text, 1=binary)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBinary(short &binary);
            /**
             * @brief		IMUのハードウェアバージョンを取得する
             * @param[out]	ハードウェアバージョン
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetImuHard(short &hard);
            /**
             * @brief		IMUのファームウェアバージョンを取得する
             * @param[out]	ファームウェアバージョン
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetImuFirm(short &firm);
            /**
             * @brief		IMUのBluetoothアドレスを取得する
             * @param[in]	配列のindex(0〜5)
             * @param[out]	Bluetoothアドレス
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetBtAdr(int index, unsigned char &adr);

            /**
             * @brief		IMUの状態取得要求送信
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SndReqImuStatus();
            /**
             * @brief		IMUのデータ周期設定
             * @param[in]	データ周期(1〜1000(10〜10000[msec])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetImuPeriod(short period);
            /**
             * @brief		IMUのデータ範囲設定
             * @param[in]	加速度範囲(0=± 2[G], 1=± 4[G], 2=± 8[G])
             * @param[in]	ジャイロ範囲(0=± 500[deg/sec], 1=± 2000[deg/sec])
             * @param[in]	地磁気範囲(0=± 0.7[Ga], 1=± 1.0[Ga], 2=± 1.5[Ga], 3=± 2.0[Ga], 4=± 3.2[Ga], 5=± 3.8[Ga], 6=± 4.5[Ga])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetImuRange(int acc, int gyro, int comp);
            /**
             * @brief		IMUのデータ周期設定
             * @param[in]	データ周期(1〜1000(10〜10000[msec])
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SndResetTimeStamp();
            /**
             * @brief		IMUの計測状態有効無効設定
             * @param[in]	計測状態有効無効(true=有効, false=無効)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetMeasurementState(bool enable);
            /**
             * @brief		IMUのデバイスプロファイル取得要求送信
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SndReqImuDeviceProfile();

// #ifdef USE_DEMO
#define N_OFZ_VALUE 12
            int GetOfzValue0(float v[N_OFZ_VALUE]);
            int GetOfzValue1(float v[N_OFZ_VALUE]);
            int GetOfzValue2(float v[N_OFZ_VALUE]);
            int GetOfzValue3(float v[N_OFZ_VALUE]);
            int GetSeatSensor(float &v);
            // #endif

            /**
             * @brief		HEVのダイアグコードをクリアする
             * @param[in]	target 消去する対象のECU(0=エンジン, 1=ハイブリッド, 2=クルーズコントロール(ブレーキ), 3=バッテリ)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int ClearDiag(HEV_ECU target);
            /**
             * @brief		HEVのダイアグコードを取得する
             * @param[in]	target 消去する対象のECU(0=エンジン, 1=ハイブリッド, 2=クルーズコントロール(ブレーキ), 3=バッテリ)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int GetDiag(HEV_ECU target);
            int ClearCntDiag();
            int GetCntDiag();

            /**
             * @brief	制御ゲインの取得を要求する
             * @param[in]	index 取得するゲインのインデックス
             *	100=ステアゲインP
             *	101=ステアゲインI
             *	102=ステアゲインD
             *	103=ステア 積分値の減衰時間
             *	104=ステア ウィンドウ幅
             *	105=ステア パンチ
             *	106=ステア 最大トルク
             *	107=ステア 左右差吸収のファクタ
             *	108=速度制御最高速度
             *	109=速度制御ゲインP+
             *	110=速度制御ゲインP-
             *	111=速度制御ウィンドウ幅+
             *	112=速度制御ウィンドウ幅-
             *	113=速度制御パンチ+
             *	114=速度制御パンチ-
             *	115=速度制御最大アクセル出力
             *	116=速度制御最大ブレーキ出力
             *	117=ステアリングオーバーライド閾値
             *	118=アクセルオーバーライド閾値
             *	119=ブレーキオーバーライド閾値
             *	120=テストモード(ホストの生存確認チェック無効)
             * @param[in]	num 取得する数 [1-3]
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int ReqControlGain(int index, int num);

            /**
             * @brief	制御ゲインの書き込みを要求する
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int SaveControlGain();

            /**
             * @brief		制御ゲインを設定する
             * @param[in]	index 設定するゲインのインデックス
             *	100=ステアゲインP(0.0〜32.767, unit:[0.001/], default:1.0[1000])
             *	101=ステアゲインI(0.0〜32.767, unit:[0.001/], default:0.05[500])
             *	102=ステアゲインD(0.0〜32.767, unit:[0.001/], default:1.0[1000])
             *	103=ステア 積分値の減衰時間(10〜32767, unit[ms], default:200[ms])
             *	104=ステア ウィンドウ幅(0.0〜3276.7[deg], unit:[0.1deg], default 0.0[deg]
             *	105=ステア パンチ(0〜4096, unit[/], default:4096)
             *	106=ステア 最大トルク(0〜4096, unit[/], default:4096)
             *	107=ステア 左右差吸収のファクタ(0.0〜32.767, unit[0.001/], default:1.0[1000])
             *	108=速度制御最高速度(0.0〜180.00[km/h], unit[0.01km/h], default:25.00[km/h])
             *	109=速度制御ゲインP+(0.0〜32.767, unit[0.001/], default:6.5[650])
             *	110=速度制御ゲインP-(0.0〜32.767, unit[0.001/], default:5.0[500])
             *	111=速度制御ウィンドウ幅+(0.0〜180.00, unit[0.01km/h], default:5.0[500])
             *	112=速度制御ウィンドウ幅-(0.0〜180.00, unit[0.01km/h], default:0.8[80])
             *	113=速度制御パンチ+(0〜4095, unit[/], default:0)
             *	114=速度制御パンチ-(0〜4095, unit[/], default:100)
             *	115=速度制御最大アクセル出力(0〜4095, unit[/], default:4095)
             *	116=速度制御最大ブレーキ出力(0〜4095, unit[/], default:4095)
             *	117=ステアリングオーバーライド閾値(0〜4095, unit[/], default:3000)
             *	118=アクセルオーバーライド閾値(0〜4095, unit[/], default:1000)
             *	119=ブレーキオーバーライド閾値(0〜4095, unit[/], default:500)
             *	120=テストモード(ホストの生存確認チェック無効)(0 or 1, unit[/], default:0)
             * @param[in]	gain 設定値
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetControlGain(int index, int gain);

            /**
             * @brief  Echo要求を送信する
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int EchoReq();
            /**
             * @brief  エラー状態取得要求を送信する
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int ReadErrorStatusReq();
            /**
             * @brief  Version要求を送信する
             * @retval	0 成功
             * @retval	0以外 不成功
             */
            int ReadVersionReq();

            /**
             * @brief		ステアリングのモードを切り替える
             * @param[in] mode ステアリングモード (manual = 0x00, program = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetStrMode(int mode);

            /**
             * @brief		ステアリングの制御モードを切り替える
             * @param[in] cont_mode ステアリング制御モード(torque = 0x00, angle = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetStrControlMode(int cont_mode);

            /**
             * @brief		ステアオーバーライドの有無を設定する
             * @param[in] override_mode オーバーライドモード (ON = 0x00, OFF = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetStrOverrideMode(int override_mode);

            /**
             * @brief		ステアリングのトルクを指示する
             * @param[in]	torque トルク値(-4096〜4095)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetStrTorque(int torque);

            /**
             * @brief		ステアリングの角度を指示する
             * @param[in]	angle 角度値 [deg.] (-666.0 ～ 666.0)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetStrAngle(float angle);

            /**
             * @brief		ステアリング制御の制御出力を切り替える
             * @param[in]	servo 制御のON/OFF (ON = true, OFF = false)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetStrServo(int servo);

            /**
             * @brief		ドライブのモードを切り替える
             * @param[in]	mode ドライブモード (manual = 0x00, program = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetDrvMode(int mode);

            /**
             * @brief		ドライブの制御モードを切り替える
             * @param[in] cont_mode ドライブ制御モード(velocity = 0x00,stroke = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetDrvControlMode(int cont_mode);

            /**
             * @brief		ドライブオーバーライドの有無を設定する
             * @param[in] override_mode オーバーライドモード (ON = 0x00, OFF = 0x10)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetDrvOverrideMode(int override_mode);

            /**
             * @brief		アクセルのストローク量を指示する
             * @param[in]	output ストローク量 (0～4095)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetGasStroke(int output);

            /**
             * @brief		ブレーキペダルのストローク量を指示する
             * @param[in]	output ブレーキペダルストローク量 (0～4095)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetBrakeStroke(int output);

            int SetBrakeLamp(unsigned char lamp);
            int SetBlinkerLeft(unsigned char blink_left);
            int SetBlinkerRight(unsigned char blink_right);
            int SetBrakeMode(unsigned char mode);

            /**
             * @brief		速度を指示する
             * @param[out]	veloc 速度 [km/h] (0.00～180.00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetVeloc(float veloc);

            /**
             * @brief		シフトを設定する
             * @param[out] shift シフトポジション(N = 0x20,D = 0x10,R = 0x40,B = 0x00)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetShiftMode(int shift);

            /**
             * @brief		ドライブ制御の制御出力を切り替える
             * @param[in]	servo 制御のON/OFF (ON = true, OFF = false)
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int SetDrvServo(int servo);

        private:
            /**
             * @brief		CAN受信コールバックメソッド
             * @param[out]	channel 受信チャネル
             * @param[out]	timestamp メッセージタイムスタンプ
             * @param[out]	msg CANメッセージ
             */
            void OnReceiveCANUSB(int channel, long timeStamp, CANMsg *msg);

            /**
             * @brief		Priorityメッセージの処理
             * @param[in]	msg_id MESSAGE_ID
             * @param[out]	msg CANメッセージ
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int ProcPriorityMessage(int msg_id, CANMsg *msg);

            /**
             * @brief		Driveメッセージの処理
             * @param[in]	msg_id MESSAGE_ID
             * @param[out]	msg CANメッセージ
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int ProcDriveMessage(int msg_id, CANMsg *msg);

            /**
             * @brief		Steeringメッセージの処理
             * @param[in]	msg_id MESSAGE_ID
             * @param[out]	msg CANメッセージ
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int ProcSteerMessage(int msg_id, CANMsg *msg);

            /**
             * @brief		Brakeメッセージの処理
             * @param[in]	msg_id MESSAGE_ID
             * @param[out]	msg CANメッセージ
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int ProcBrakeMessage(int msg_id, CANMsg *msg);

            /**
             * @brief		System Commonメッセージの処理
             * @param[in]	msg_id MESSAGE_ID
             * @param[out]	msg CANメッセージ
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int ProcSysComMessage(int msg_id, CANMsg *msg);

            int ProcImuMessage(int msg_id, CANMsg *msg);

            // #ifdef USE_DEMO
            int ProcDemoSensorMessage(int sensor_id, int msg_id, CANMsg *msg);
            // #endif

            /**
             * @brief		OBDメッセージの処理
             * @param[in]	msg_id MESSAGE_ID
             * @param[out]	msg CANメッセージ
             * @retval		0 成功
             * @retval		0以外 不成功
             */
            int ProcOBDMessage(CANMsg *msg);

            ChangeStateObserver *_callback; /*!< コールバック関数登録ハンドラ */
            //    CanCommunication*	_can_communication;	/*!< CAN通信クラス */
            CANUSBZ *_can_communication;               /*!< CAN通信クラス */
            DriveInformation _drive_information;       /*!< ドライブ情報保存クラス */
            BrakeInformation _brake_information;       /*!< ブレーキ情報保存クラス */
            SteeringInformation _steering_information; /*!< ステアリング情報保存クラス */
            OtherInformation _other_information;       /*!< その他情報保続クラス */
            BatteryInformation _batt_information;      /*!< バッテリ情報保存クラス */
            ImuInformation _imu_information;           /*!< IMUメッセージ保存クラス */
                                                       // #ifdef USE_DEMO
            DemoSensorInformation _demo_sensor_information;
            OfzBuff _ofz_buff[4];
            // #endif
        };
    }
}

#endif /* HEVCONTROL_ */
