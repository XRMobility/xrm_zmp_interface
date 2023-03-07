/**
 * @file	HevControlCommon.h
 * @brief	共通ユーティリティクラスヘッダファイル
 * @date	2012/02/15
 * @author	m.ota
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */

#ifndef HEVCONTROLCOMMON_H_
#define HEVCONTROLCOMMON_H_

#include "HEVConst.h"

namespace zmp
{
	namespace minivan
	{

/**
 * @brief 共通ユーティリティクラス
 *
 * @author		m.ota
 * @date		2012/02/15
 */
class HevControlCommon
{
public:
	HevControlCommon();
	virtual ~HevControlCommon();

	/**
 	 * @brief	CANIDからMESSAGE_GROUP, MESSAGE_IDを取得する
 	 *
 	 * @param[in]		id CAN ID
 	 * @param[out]		group_id MESSAGE_GROUPのID
 	 * @param[out]		message_id MESSAGE_ID
	 */
	static void ParseID(int id, int& group_id, int& message_id);

	/**
 	 * @brief	2byteを結合する
 	 *
 	 * @param[in]		c_h 上位バイト
 	 * @param[in]		c_l 下位バイト
 	 *
 	 * @return			結合された値
	 */
	static short MkShort(unsigned char c_h, unsigned char c_l);

	/**
 	 * @brief	2byteを結合する
 	 *
 	 * @param[in]		c_h 上位バイト
 	 * @param[in]		c_l 下位バイト
 	 *
 	 * @return			結合された値
	 */
	static unsigned short MkUShort(unsigned char c_h, unsigned char c_l);

	/**
 	 * @brief	12bit signedの2byteを結合する
 	 *
 	 * @param[in]		c_h 上位バイト
 	 * @param[in]		c_l 下位バイト
 	 *
 	 * @return			結合された値
	 */
	static short Signed12ToShort(unsigned char c_h, unsigned char c_l);

};

	} /* namespace minivan */
} /* namespace zmp */
#endif /* HEVCONTROLCOMMON_H_ */
