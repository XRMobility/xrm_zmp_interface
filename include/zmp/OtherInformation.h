/**
 * @file	OtherInformation.h
 * @brief	その他情報クラスヘッダファイル
 *
 * @date	2012/02/15
 * @author	m.ota
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */
#ifndef OTHERINFORMATION_H_
#define OTHERINFORMATION_H_

#include "HEVConst.h"

namespace zmp
{
	namespace minivan
	{
/**
 * @brief 各種情報を保持するためのクラス
 *
 * @author	m.ota
 * @date	2012/02/15
 */
class OtherInformation
{
public:
	/**
	 * @brief コンストラクタ
	 */
	OtherInformation();
	/**
	 * @brief デストラクタ
	 */
	virtual ~OtherInformation();
	
	/**
	 * @brief ドアの開閉状態を取得する
	 * @retval ドア開閉状態(0x00=CLOSE, 0x04=後部ドアOPEN, 0x40=助手席OPEN, 0x80=運転席OPEN)
	 */
	int GetDoorsStateFromOBD() const;
	/**
	 * @brief ガソリン残量[ℓ ]を取得する
	 * @retval ガソリン残量[ℓ ](0〜40)
	 */
	int GetGasLevelFromOBD() const;
	/**
	 * @brief ライト点灯状態を取得する
	 * @retval 0x00 OFF
	 * @retval 0x10 スモールライト点灯
	 * @retval 0x30 ライト点灯 
	 * @retval 0x38 ハイビーム点灯 
	 */
	int GetLightStateFromOBD() const;
		
	/**
	 * @brief ドアの開閉状態を保存する 
	 * @param[in] ドアの開閉状態(0x00=CLOSE, 0x04=後部OPEN, 0x40=助手席OPEN, 0x80=運転席OPEN) 
	 */
	void SetDoorsStateFromOBD(int doorsState);
	/**
	 * @brief ガソリン残量を保存する 
	 * @param[in] ガソリン残量[ℓ ](0〜40)
	 */
	void SetGasLevelFromOBD(int gasLevel);
	/**
	 * @brief ライト点灯状態を保存する 
	 * @param[in] ライト点灯状態(0x00=OFF, 0x10=スモールライト, 0x30=ライト, 0x38=ハイビーム) 
	 */
	void SetLightStateFromOBD(int lightState);

private:
	int	_light_state;	/*!< @brief ヘッドライト状態 */
	int	_doors_state;	/*!< @brief ドア警告 */
	unsigned int	_gas_level;	/*!< @brief ガス残量 */
};
	}
}

#endif /* OTHERINFORMATION_H_ */
