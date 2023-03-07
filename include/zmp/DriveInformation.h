/**
 * @file	DriveInformation.h
 * @brief	ドライブ情報クラスヘッダファイル
 *
 * @date	2012/02/07
 * @author	m.ota
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */
#ifndef DRIVEINFORMATION_H_
#define DRIVEINFORMATION_H_

#include "HEVConst.h"

namespace zmp
{
    namespace minivan
    {
/**
 * @brief ドライブ情報を保持するためのクラス
 *
 * @author	m.ota
 * @date	2012/02/07
 */
class DriveInformation
{
public:
    /**
     * @brief コンストラクタ
     */
    DriveInformation();
    /**
     * @brief デストラクタ
     */
    virtual ~DriveInformation();

    /**
     * @brief ドライブ制御のモードを保存する
     * @param[in] mode(0x00=manula, 0x10=program) 
     */
    void SetMode(int mode);
    /**
     * @brief ドライブ制御の制御モードを保存する
     * @param[in] cont_mode(0x00=velocity, 0x10=stroke) 
     */
    void SetContMode(int cont_mode);
    /**
     * @brief ドライブ制御のオーバーライドモードを保存する
     * @param[in] override_mode(0x00=ON, 0x10=OFF) 
     */
    void SetOverrideMode(int override_mode);
    /**
     * @brief アクセルペダルストロークの現在量を保存する 
     * @param[in] actual(0〜4095) 
     */
    void SetActualGasStroke(int actual);
    /**
     * @brief OBDから受信したアクセルペダルストロークの現在量を保存する 
     * @param[in] actual(0〜4095) 
     */
    void SetGasStrokeFromOBD(int actual);
    /**
     * @brief アクセルペダルストロークのペダルからの入力量を保存する 
     * @param[in] input(0〜4095) 
     */
    void SetInputGasStroke(int input);
    /**
     * @brief アクセルペダルストロークの目標量を保存する 
     * @param[in] target(0〜4095) 
     */
    void SetTargetGasStroke(int target);
    /**
     * @brief シフトポジション現在値を保存する 
     * @param[in] actual(0x00=B, 0x10=D, 0x20=N, 0x40=R, 0x80=P, 0x0F=Start, 0xFF=Unknown) 
     */
    void SetActualShift(int shift);
    /**
     * @brief シフトレバーからの入力値を保存する 
     * @param[in] actual(0x00=B, 0x10=D, 0x20=N, 0x40=R, 0x80=P, 0x0F=Start, 0xFF=Unknown) 
     */
    void SetInputShift(int shift);
    /**
     * @brief  OBDから受信したシフトポジション現在値を保存する
     * @param[in] shift(0x00=B, 0x10=D, 0x20=N, 0x40=R) 
     */
    void SetShiftFromOBD(int shift);
    /**
     * @brief シフトポジション値を保存する 
     * @param[in] actual(0x00=B, 0x10=D, 0x20=N, 0x40=R, 0x0F=Start) 
     */
    void SetTargetShift(int target_shift);
    /**
     * @brief 現在速度[km/h]を保存する 
     * @param[in] veloc(0.00〜180.00[km/h]) 
     */
    void SetActualVeloc(float veloc);
    /**
     * @brief OBDから受信した現在速度[km/h]を保存する 
     * @param[in] veloc(0.00〜180.00[km/h]) 
     */
    void SetVelocFromOBD(float veloc);
    /**
     * @brief OBDから受信した現在速度2[km/h]を保存する 
     * @param[in] veloc(0.00〜180.00[km/h]) 
     */
    void SetVelocFromOBD2(float veloc);
    /**
     * @brief 現在速度[km/h]を保存する 
     * @param[in] target_veloc(0.00〜180.00[km/h]) 
     */
    void SetTargetVeloc(float target_veloc);
    /**
     * @brief OBDから受信した右前輪速度[km/h]を保存する 
     * @param[in] veloc(0.00〜180.00[km/h]) 
     */
    void SetVelocFRFromOBD(float veloc);
    /**
     * @brief OBDから受信した左前輪速度[km/h]を保存する 
     * @param[in] veloc(0.00〜180.00[km/h]) 
     */
    void SetVelocFLFromOBD(float veloc);
    /**
     * @brief OBDから受信した右後輪速度[km/h]を保存する 
     * @param[in] veloc(0.00〜180.00[km/h]) 
     */
    void SetVelocRRFromOBD(float veloc);
    /**
     * @brief OBDから受信した左後輪速度[km/h]を保存する 
     * @param[in] veloc(0.00〜180.00[km/h]) 
     */
    void SetVelocRLFromOBD(float veloc);
    /**
     * @brief OBDから受信した温度[℃ ]を保存する 
     * @param[in] temp(0〜[℃ ]) 
     */
    void SetCoolantTempFromOBD(float temp);
    /**
     * @brief OBDから受信したクルーズ状態を保存する 
     * @param[in] cruise_control 
     */
    void SetCruiseControlFromOBD(bool cruise_control);
    /**
     * @brief OBDから受信したドライブモードを保存する 
     * @param[in] drive_mode(0=p, 1=R, 2=N, 3=D, 4=B) 
     */
    void SetDriveModeFromOBD(int drive_mode);
    /**
     * @brief OBDから受信したをEVモードを保存する 
     * @param[in] ev_mode 
     */
    void SetEvModeFromOBD(int ev_mode);
    /**
     * @brief OBDから受信したをエンジン回転数[rpm]を保存する 
     * @param[in] ice_rpm[rpm] 
     */
    void SetIceRpmFromOBD(int ice_rpm);
    /**
     * @brief OBDから受信したを加速度を保存する 
     * @param[in] acc 
     */
    void SetAccFromOBD(float acc);
    /**
     * @brief OBDから受信したをYawレートを保存する 
     * @param[in] yaw_rate
     */
    void SetSideAccFromOBD(short yaw_rate);
    /**
     * @brief ドライブサーボ状態を保存する 
     * @param[in] servo(0x00=OFF, 0x10=ON) 
     */
    void SetServo(int servo);
    /**
     * @brief DTC状態を保存する 
     * @param[in] msg(8byte)
     */
    void SetDtcStatus(unsigned char* msg);

    /**
     * @brief ドライブモードを取得する
     * @retval 0x00 manual
     * @retval 0x10 program 
     */
    int GetMode() const;
    /**
     * @brief ドライブ制御モードを取得する
     * @retval 0x00 velocity 
     * @retval 0x10 stroke 
     */
    int GetContMode() const;
    /**
     * @brief ドライブオーバーライドモードを取得する
     * @retval 0x00 ON 
     * @retval 0x10 OFF 
     */
    int GetOverrideMode() const;
    /**
     * @brief アクセルペダルストローク現在量を取得する
     * @retval ペダルストローク現在量(0〜4095) 
     */
    int GetActualGasStroke() const;
    /**
     * @brief OBDから受信したアクセルペダルストローク現在量を取得する
     * @retval ペダルストローク現在量(0〜4095) 
     */
    int GetGasStrokeFromOBD() const;
    /**
     * @brief アクセルペダルからの入力量を取得する
     * @retval ペダルストローク入力量(0〜4095) 
     */
    int GetInputGasStroke() const;
    /**
     * @brief アクセルペダルストローク目標量を取得する
     * @retval ペダルストローク目標量(0〜4095) 
     */
    int GetTargetGasStroke() const;
//    int GetTargetGasStrokeFromOBD() const;
    /**
     * @brief シフトポジション現在値を取得する
     * @retval 0x00 B 
     * @retval 0x10 D 
     * @retval 0x20 N 
     * @retval 0x40 R 
     * @retval 0x80 P 
     * @retval 0x0F Start 
     * @retval 0xFF Unknown
     */
    int GetActualShift() const;
    /**
     * @brief シフトレバーからの入力値を取得する
     * @retval 0x00 B 
     * @retval 0x10 D 
     * @retval 0x20 N 
     * @retval 0x40 R 
     * @retval 0x80 P 
     * @retval 0x0F Start 
     * @retval 0xFF Unknown
     */
    int GetInputShift() const;
    /**
     * @brief OBDから受信したシフトポジション現在値を取得する
     * @retval 0x00 B 
     * @retval 0x10 D 
     * @retval 0x20 N 
     * @retval 0x40 R 
     */
    int GetShiftFromOBD() const;
    /**
     * @brief シフトポジション目標値を取得する
     * @retval 0x00 B 
     * @retval 0x10 D 
     * @retval 0x20 N 
     * @retval 0x40 R 
     * @retval 0x0F Start 
     */
    int GetTargetShift() const;
    /**
     * @brief 現在速度を取得する
     * @retval 現在速度(0.00〜180.00[km/h]) 
     */
    float GetActualVeloc() const;
    /**
     * @brief OBDから受信した現在速度を取得する
     * @retval OBDから受信した現在速度(0.00〜180.00[km/h]) 
     */
    float GetVelocFromOBD() const;
    /**
     * @brief OBDから受信した現在速度2を取得する
     * @retval OBDから受信した現在速度2(0.00〜180.00[km/h]) 
     */
    float GetVelocFromOBD2() const;
    /**
     * @brief 目標速度を取得する
     * @retval 目標速度(0.00〜180.00[km/h]) 
     */
    float GetTargetVeloc() const;
    /**
     * @brief 右前輪速度を取得する
     * @retval 右前輪速度(0.00〜180.00[km/h]) 
     */
    float GetVelocFRFromOBD() const;
    /**
     * @brief 左前輪速度を取得する
     * @retval 左前輪速度(0.00〜180.00[km/h]) 
     */
    float GetVelocFLFromOBD() const;
    /**
     * @brief 右後輪速度を取得する
     * @retval 右後輪速度(0.00〜180.00[km/h]) 
     */
    float GetVelocRRFromOBD() const;
    /**
     * @brief 左後輪速度を取得する
     * @retval 左後輪速度(0.00〜180.00[km/h]) 
     */
    float GetVelocRLFromOBD() const;
    /**
     * @brief 温度を取得する 
     * @retval 温度(0〜[℃ ]) 
     */
    float GetCoolantTempFromOBD() const;
    /**
     * @brief クルーズ状態を取得する 
     * @retval クルーズ状態 
     */
    bool GetCruiseControlFromOBD() const;
    /**
     * @brief ドライブモードを取得する 
     * @retval 0=P
     * @retval 1=R
     * @retval 2=N
     * @retval 3=D
     * @retval 4=B
     */
    int GetDriveModeFromOBD() const;
    /**
     * @brief EVモードを取得する 
     * @retval EVモード 
     */
    int GetEvModeFromOBD() const;
    /**
     * @brief エンジン回転数[rpm]を取得する 
     * @retval エンジン回転数[rpm] 
     */
    int GetIceRpm() const;
    /**
     * @brief 加速度を取得する 
     * @retval 加速度[G]
     */
    float GetAccFromOBD() const;
    /**
     * @brief Yawレートを取得する 
     * @retval Yawレート[G]
     */
    float GetSideAccFromOBD() const;
    /**
     * @brief サーボ状態を取得する 
     * @retval サーボ状態(0x00=OFF, 0x10=ON)
     */
    int GetServo() const;
    /**
     * @brief DTC状態を取得する 
     * @param[in] index 配列引数(0〜7)	
     * @retval DTC状態(0x00〜0xff)
     */
    unsigned char GetDtcStatus(int index) const;

private:
    int	_mode;		/*!< @brief ドライブモード */
    int	_cont_mode;	/*!< @brief ドライブ制御モード */
    int	_override_mode;	/*!< @brief ドライブオーバーライドモード */
    int	_is_servo;	/*!< @brief サーボのON/OFF */
    int	_target_gas_stroke;/*!< @brief アクセルペダルストローク指令値 */
    int	_actual_gas_stroke;/*!< @brief アクセルペダルストローク現在値 */
    int	_input_gas_stroke;/*!< @brief アクセルペダルストローク入力値 */
    int	_gas_stroke_fromObd;/*!< @brief OBDから受信したアクセルペダルストローク現在値 */
    float _vpa1;	/*!< @brief VPA1 */
    float _vpa2;	/*!< @brief VPA2 */
    float _target_veloc;/*!< @brief 速度指令値 */
    float _actual_veloc;/*!< @brief 速度 */
    float _veloc_fromObd;/*!< @brief OBD速度 */
    float _veloc_fromObd2;/*!< @brief OBD速度2 */
    float _veloc_fr;	/*!< @brief 右前ホイール速度 */
    float _veloc_fl;	/*!< @brief 左前ホイール速度 */
    float _veloc_rr;	/*!< @brief 右後ホイール速度 */
    float _veloc_rl;	/*!< @brief 左後ホイール速度 */
    int	_target_shift;	/*!< @brief シフト指令値 */
    int	_actual_shift;	/*!< @brief シフト現在値 */
    int	_input_shift;	/*!< @brief シフト入力値 */
    int	_shift_fromObd; /*!< @brief OBDシフト現在値 */
    int	_drive_mode_fromObd;/*!< @brief ドライブモード */
    int	_ev_mode;	/*!< @brief EVモード */
    int	_ice_rpm;	/*!< @brief エンジン回転数 */
    float _coolant_temp;/*!< @brief エンジン冷却水温度 */
    bool _cruise_control;/*!< @brief クルーズコントロール */
    float _acc1;	/*!< @brief 縦方向加速度(raw data(new)) */
    float _acc2;	/*!< @brief 縦方向加速度(raw data(old)) */
    float _acc;		/*!< @brief 縦方向加速度[G] */
    float _side_acc;	/*!< @brief Yawレート */
    unsigned char _dtcStatus[8];/*!< @brief DTC状態[8] */
};
    }
}

#endif /* DRIVEINFORMATION_H_ */
