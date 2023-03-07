/**
 * @file	SteeringInformation.h
 * @brief	ステアリング情報クラスヘッダファイル
 *
 * @date	2012/02/07
 * @author	m.ota
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */
#ifndef STEERINGINFORMATION_H_
#define STEERINGINFORMATION_H_

namespace zmp
{
    namespace minivan
    {
/**
 * @brief ステアリング情報を保持するためのクラス
 *
 * @author	m.ota
 * @date	2012/02/07
 */
class SteeringInformation
{
public:
    /**
     * @brief コンストラクタ
     */
    SteeringInformation();
    /**
     * @brief デストラクタ
     */
    ~SteeringInformation();

    /**
     * @brief ステアリングの現在角度[deg]を取得する 
     * @retval ステアリング現在角度[deg](-666.0〜666.0)
     */
    float GetActualAngle() const;
    /**
     * @brief ステアリングの制御モードを取得する 
     * @retval 制御モード 0x00=torque 
     * @retval 制御モード 0x10=angle 
     */
    int GetContMode() const;
    /**
     * @brief ステアリングの目標角度[deg]を取得する 
     * @retval ステアリング目標角度[deg](-666.0〜666.0)
     */
    float GetTargetAngle() const;
    /**
     * @brief ステアリングの目標トルクを取得する 
     * @retval ステアリング目標トルク(-4096〜4095)
     */
    int GetTargetTorque() const;
    /**
     * @brief ステアリングのモードを取得する 
     * @retval モード 0x00=manual 
     * @retval モード 0x10=program 
     */
    int GetMode() const;
    /**
     * @brief ステアリングのオーバーライドモードを取得する 
     * @retval オーバーライドモード 0x00=ON 
     * @retval オーバーライドモード 0x10=OFF 
     */
    int GetOverrideMode() const;
    /**
     * @brief ステアリングの現在トルクを取得する 
     * @retval ステアリング現在トルク(-4096〜4095)
     */
    float GetActualTorque() const;
    /**
     * @brief ステアリングのサーボ状態を取得する 
     * @retval ステアリングサーボ状態 0x00 OFF
     * @retval ステアリングサーボ状態 0x10 ON
     */
    int GetServo() const;

    /**
     * @brief OBDから取得したステアリングの現在角度[deg]を取得する 
     * @retval ステアリング現在角度[deg](-666.0〜666.0)
     */
    float GetAngleFromOBD() const;
	/**
	 * @brief	OBDから受信したステアリング角度[deg]を保存する
	 * @param[in]	angle ステアリング角度[deg](-666.0〜666.0)
	 */
    void SetAngleFromOBD(float angle);

	/**
	 * @brief	ステアリング角度[deg]を保存する
	 * @param[in]	angle ステアリング角度[deg](-666.0〜666.0)
	 */
	void SetActualAngle(float angle);

	/**
	 * @brief	コントロールモードを保存する
	 * @param[in]	cont_mode 制御モード(0x00=torque, 0x10=angle)
	 */
	void SetContMode(int cont_mode);

	/**
 	 * @brief	ステアリング角度目標値を保存する
 	 * @param[in] 	input_angle ステアリング角度目標値[deg](-666.0〜666.0)
	 */
	void SetTargetAngle(float input_angle);

	/**
	 * @brief	トルク目標値を保存する
	 * @param[in]	input_torque 目標トルク(-4096〜4095)
	 */
	void SetTargetTorque(float input_torque);

	/**
	 * @brief	モードを保存する
	 * @param[in]	mode モード(0x00=manual, 0x10=program)
	 */
	void SetMode(int mode);

	/**
	 * @brief	オーバーライドモードを保存する
	 * @param[in]	override_mode オーバーライドモード(0x00=ON, 0x10=OFF)
	 */
	void SetOverrideMode(int override_mode);

	/**
	 * @brief	現在トルクを保存する
	 * @param[in]	torque 現在トルク(-4096〜4095)
	 */
	void SetActualTorque(float torque);

	/**
	 * @brief	サーボ状態を保存する
	 * @param[in]	servo(0x00=OFF, 0x10=ON)
	 */
	void SetServo(int servo);



private:
    int		_mode;		/*!< @brief ステアリングモード */
    int		_cont_mode;	/*!< @brief ステアリング制御モード */
    int		_override_mode;	/*!< @brief ステアリングオーバライドモード */
    float	_target_torque;	/*!< @brief ステアリング操舵トルク指令値 */
    float	_actual_torque;	/*!< @brief ステアリング操舵トルク値 */
    float	_target_angle;	/*!< @brief ステアリング操舵角度指令値 */
    float	_actual_angle;	/*!< @brief ステアリング操舵角度値 */
    float	_angleFromObd;  /*!< @brief OBDから受信したステアリング操舵角度値 */
    int		_is_servo;	/*!< @brief サーボのON/OFF */
};
    }
}

#endif /* STEERINGINFORMATION_H_ */
