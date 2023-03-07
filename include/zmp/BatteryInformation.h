/**
 * @file	BatteryInformation.h
 * @brief	バッテリ情報クラスヘッダファイル
 *
 * @date	2012/02/15
 * @author	m.ota
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */
#ifndef BATTERYINFORMATION_H_
#define BATTERYINFORMATION_H_

namespace zmp
{
    namespace minivan
    {
/**
 * @brief バッテリ情報を保持するためのクラス
 *
 * @author	m.ota
 * @date	2012/02/15
 */
class BatteryInformation
{
public:
    /**
     * @brief	コンストラクタ
     */
    BatteryInformation();
    /**
     * @brief	デストラクタ
     */
    virtual ~BatteryInformation();

    /**
     * @brief バッテリ電流を取得する
     *
     * @retval バッテリ電流[A]
     */
    float GetCurrentFromOBD() const;

    /**
     * @brief バッテリのチャージ電流最大値を取得する
     *
     * @retval バッテリのチャージ電流最大値[A]
     */
    float GetCurrentChgMaxFromOBD() const;

    /**
     * @brief バッテリのディスチャージ電流最大値を取得する
     *
     * @retval バッテリのディスチャージ電流最大値[A]
     */
    float GetCurrentDischgMaxFromOBD() const;

    /**
     * @brief バッテリのstate of chargeを取得する
     *
     * @retval バッテリのstate of charge[%]
     */
    float GetSocFromOBD() const;

    /**
     * @brief バッテリの最高温度を取得する
     *
     * @retval バッテリの最高温度[℃ ]
     */
    int GetTempMaxFromOBD() const;

    /**
     * @brief バッテリの最低温度を取得する
     *
     * @retval バッテリの最低温度[℃ ]
     */
    int GetTempMinFromOBD() const;

    /**
     * @brief バッテリの電圧を取得する
     *
     * @retval バッテリの電圧[V]
     */
    int GetVoltageFromOBD() const;

    /**
     * @brief バッテリの電流を保存する
     *
     * @param[in]	current OBDから受信したバッテリ電流[A]
     */
    void SetCurrentFromOBD(float current);
    /**
     * @brief バッテリのチャージ電流最大値を保存する
     *
     * @param[in]	current OBDから受信したバッテリのチャージ電流最大値[A]
     */
    void SetCurrentChgMaxFromOBD(float current_chrg_max);
    /**
     * @brief バッテリのディスチャージ電流最大値を保存する
     *
     * @param[in]	current OBDから受信したバッテリのディスチャージ電流最大値[A]
     */
    void SetCurrentDischgMaxFromOBD(float current_dischrg_max);
    /**
     * @brief バッテリのstate of chargeを保存する
     *
     * @param[in]	current OBDから受信したバッテリのstate of charge[%]
     */
    void SetSocFromOBD(float soc);
    /**
     * @brief バッテリの最高温度を保存する
     *
     * @param[in]	current OBDから受信したバッテリの最高温度[℃ ]
     */
    void SetTempMaxFromOBD(int temp_max);
    /**
     * @brief バッテリの最低温度を保存する
     *
     * @param[in]	current OBDから受信したバッテリの最低温度[℃ ]
     */
    void SetTempMinFromOBD(int temp_min);
    /**
     * @brief バッテリの電圧を保存する
     *
     * @param[in]	current OBDから受信したバッテリ電圧[V]
     */
    void SetVoltageFromOBD(int voltage);

private:
    int		_voltage;		/*!< バッテリ電圧 */
    float	_soc;			/*!< 充電状態 */
    float	_current;		/*!< 現在電流 */
    int		_temp_max;		/*!< 最大バッテリ温度 */
    int		_temp_min;		/*!< 最小バッテリ温度 */
    float	_current_chrg_max;	/*!< 最大充電電流 */
    float	_current_dischrg_max;	/*!< 最大放電電流 */
};
    }
}

#endif /* BATTERYINFORMATION_H_ */
