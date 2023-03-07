/**
 * @file	ImuInformation.h
 * @brief	IMU情報クラスヘッダファイル
 *
 * @date	2012/08/02
 * @author	k.sekiguchi
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */
#ifndef IMUINFORMATION_H_
#define IMUINFORMATION_H_

#include "CANConst.h"

namespace zmp
{
    namespace minivan
    {

/**
 * @brief IMU 加速度データ構造体
 */
struct ACC_INF
{
    unsigned short time; /*!< タイムスタンプ */
    short accX;		/*!< X軸データ */
    short accY;		/*!< Y軸データ */
    short accZ;		/*!< Z軸データ */
};

/**
 * @brief IMU ジャイロデータ構造体
 */
struct GYRO_INF
{
    unsigned short time; /*!< タイムスタンプ */
    short gyroX;	/*!< X軸データ */
    short gyroY;	/*!< Y軸データ */
    short gyroZ;	/*!< Z軸データ */
};

/**
 * @brief IMU 地磁気データ構造体
 */
struct COMP_INF
{
    unsigned short time; /*!< タイムスタンプ */
    short compX;	/*!< X軸データ */
    short compY;	/*!< Y軸データ */
    short compZ;	/*!< Z軸データ */
};

/**
 * @brief IMU ステータスデータ構造体
 */
struct IMU_STATUS
{
    short role;		/*!< 通信方式 */
    short period;	/*!< 通信周期 */
    IMU_RANGE_ACC accRange; /*!< 加速度データ範囲 */
    IMU_RANGE_GYRO gyroRange; /*!< ジャイロデータ範囲 */
    IMU_RANGE_COMP compRange; /*!< 地磁気データ範囲 */
    short batt;		/*!< バッテリ電圧 */
    short binary;	/*!< フォーマット */
};

/**
 * @brief IMU デバイスプロファイル構造体
 */
struct IMU_DEVICE_PROFILE
{
    short hard;	/*!< ハードウェアバージョン */
    short firm; /*!< ファームウェアバージョン */
    unsigned char btaddr[6]; /*!< Bluetoothアドレス */
};


/**
 * @brief IMU情報を保持するためのクラス
 *
 * @author	k.sekiguchi
 * @date	2012/08/2
 */
class ImuInformation
{
public:
    /**
     * @brief	コンストラクタ
     */
    ImuInformation();
    /**
     * @brief	デストラクタ
     */
    virtual ~ImuInformation();

    /**
     * @brief	加速度タイムスタンプ取得
     * @retval	加速度タイムスタンプ	
     */
    unsigned short GetAccTime() const;
    /**
     * @brief	加速度X軸データ取得
     * @retval	加速度X軸データ
     */
    short GetAccX() const;
    /**
     * @brief	加速度Y軸データ取得
     * @retval	加速度Y軸データ
     */
    short GetAccY() const;
    /**
     * @brief	加速度Z軸データ取得
     * @retval	加速度Z軸データ
     */
    short GetAccZ() const;
    /**
     * @brief	ジャイロタイムスタンプ取得
     * @retval	ジャイロタイムスタンプ	
     */
    unsigned short GetGyroTime() const;
    /**
     * @brief	ジャイロX軸データ取得
     * @retval	ジャイロX軸データ
     */
    short GetGyroX() const;
    /**
     * @brief	ジャイロY軸データ取得
     * @retval	ジャイロY軸データ
     */
    short GetGyroY() const;
    /**
     * @brief	ジャイロZ軸データ取得
     * @retval	ジャイロZ軸データ
     */
    short GetGyroZ() const;
    /**
     * @brief	地磁気タイムスタンプ取得
     * @retval	地磁気タイムスタンプ	
     */
    unsigned short GetCompTime() const;
    /**
     * @brief	地磁気X軸データ取得
     * @retval	地磁気X軸データ
     */
    short GetCompX() const;
    /**
     * @brief	地磁気Y軸データ取得
     * @retval	地磁気Y軸データ
     */
    short GetCompY() const;
    /**
     * @brief	地磁気Z軸データ取得
     * @retval	地磁気Z軸データ
     */
    short GetCompZ() const;
    /**
     * @brief	通信方式取得
     * @retval	通信方式(0=SingleBT, 1=CAN-Master, 2=CAN-Slave)
     */
    short GetRole() const;
    /**
     * @brief	通信周期取得
     * @retval	通信周期(1〜1000(10〜10000[msec]))
     */
    short GetPeriod() const;
    /**
     * @brief	加速度計測範囲取得
     * @retval	計測範囲(0=± 2[G], 1=± 4[G], 2=± 8[G])
     */
    IMU_RANGE_ACC GetAccRange() const;
    /**
     * @brief	ジャイロ計測範囲取得
     * @retval	計測範囲(0=± 500[deg/sec], 1=± 2000[deg/sec])
     */
    IMU_RANGE_GYRO GetGyroRange() const;
    /**
     * @brief	地磁気計測範囲取得
     * @retval	計測範囲(0=± 0.7[Ga], 1=± 1.0[Ga], 2=± 1.5[Ga], 3=± 2.0[Ga], 4=± 3.2[Ga], 5=± 3.8[Ga], 6=± 4.5[Ga])
     */
    IMU_RANGE_COMP GetCompRange() const;
    /**
     * @brief	バッテリ電圧取得
     * @retval  バッテリ電圧	
     */
    short GetImuBatt() const;
    /**
     * @brief	フォーマット取得
     * @retval  フォーマット(0=text, 1=binary)	
     */
    short GetBinary() const;
    /**
     * @brief	ハードウェアバージョン取得
     * @retval  ハードウェアバージョン 
     */
    short GetHard() const;
    /**
     * @brief	ファームウェアバージョン取得
     * @retval  ファームウェアバージョン 
     */
    short GetFirm() const;
    /**
     * @brief	Bluetoothアドレス取得
     * @param[in] 配列のindex
     * @retval	Bluetoothアドレス
     */
    unsigned char GetBtAdr(int index) const;

    /**
     * @brief	加速度タイムスタンプ保存	
     * @param[in] タイムスタンプ 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetAccTime(unsigned short time);
    /**
     * @brief	加速度X軸データ保存	
     * @param[in] X軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetAccX(short accX);
    /**
     * @brief	加速度Y軸データ保存	
     * @param[in] Y軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetAccY(short accY);
    /**
     * @brief	加速度Z軸データ保存	
     * @param[in] Z軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetAccZ(short accZ);
    /**
     * @brief	ジャイロタイムスタンプ保存	
     * @param[in] タイムスタンプ 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetGyroTime(unsigned short time);
    /**
     * @brief	ジャイロX軸データ保存	
     * @param[in] X軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetGyroX(short gyroX);
    /**
     * @brief	ジャイロY軸データ保存	
     * @param[in] Y軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetGyroY(short gyroY);
    /**
     * @brief	ジャイロZ軸データ保存	
     * @param[in] Z軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetGyroZ(short gyroZ);
    /**
     * @brief	地磁気タイムスタンプ保存	
     * @param[in] タイムスタンプ 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetCompTime(unsigned short time);
    /**
     * @brief	地磁気X軸データ保存	
     * @param[in] X軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetCompX(short CompX);
    /**
     * @brief	地磁気Y軸データ保存	
     * @param[in] Y軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetCompY(short CompY);
    /**
     * @brief	地磁気Z軸データ保存	
     * @param[in] Z軸データ
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetCompZ(short CompZ);
    /**
     * @brief	通信方式保存	
     * @param[in] 通信方式(0=SingleBT, 1=CAN-Master, 2=CAN-Slave) 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetRole(short role);
    /**
     * @brief	通信周期保存	
     * @param[in] 通信周期(1〜1000(10〜10000[msec]) 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetPeriod(short period);
    /**
     * @brief	加速度計測範囲保存	
     * @param[in] 計測範囲(0=± 2[G], 1=± 4[G], 2=± 8[G]) 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetAccRange(IMU_RANGE_ACC acc);
    /**
     * @brief	ジャイロ計測範囲保存	
     * @param[in] 計測範囲(0=± 500[deg/sec], 1=± 2000[deg/sec]) 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetGyroRange(IMU_RANGE_GYRO gyro);
    /**
     * @brief	地磁気計測範囲保存	
     * @param[in] 計測範囲(0=± 0.7[Ga], 1=± 1.0[Ga], 2=± 1.5[Ga], 3=± 2.0[Ga], 4=± 3.2[Ga], 5=± 3.8[Ga], 6=± 4.5[Ga]) 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetCompRange(IMU_RANGE_COMP comp);
    /**
     * @brief	バッテリ電圧保存	
     * @param[in] バッテリ電圧 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetImuBatt(short batt);
    /**
     * @brief	フォーマット保存	
     * @param[in] フォーマット 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetBinary(short binary);
    /**
     * @brief	ハードウェアバージョン保存	
     * @param[in] ハードウェアバージョン 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetHard(short hard);
    /**
     * @brief	ファームウェアバージョン保存	
     * @param[in] ファームウェアバージョン 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetFirm(short firm);
    /**
     * @brief	Bluetoothアドレス保存	
     * @param[in] 配列のindex
     * @param[in] Bluetoothアドレス 
     * @retval	0 成功
     * @retval  0以外 不成功
     */
    void SetBtAdr(int index, unsigned char adr);
	

private:
    ACC_INF	_accData;	/*!< 加速度データ */
    GYRO_INF	_gyroData;	/*!< ジャイロデータ */
    COMP_INF	_compData;	/*!< 地磁気データ */
    IMU_STATUS  _imuStatus;	/*!< IMUステータス */
    IMU_DEVICE_PROFILE _imuDeviceProfile; /*!< デバイスプロファイル */
};
    }
}

#endif /* IMUINFORMATION_H_ */
