/**
 * @file	BrakeInformation.h
 * @brief	ブレーキ情報クラスヘッダファイル
 * 
 * @date	2012/02/15
 * @author	m.ota
 * @par		Copyright:
 *		2012 XXX All rights reserved.
 */
#ifndef BRAKEINFORMATION_H_
#define BRAKEINFORMATION_H_

namespace zmp
{
    namespace minivan
    {
/**
 * @brief ブレーキ情報を保持するためのクラス
 *
 * @author	m.ota
 * @date	2012/02/15
 */
class BrakeInformation
{
public:
    /**
     * @brief コンストラクタ
     */
    BrakeInformation();
    /**
     * @brief デストラクタ 
     */
    virtual ~BrakeInformation();

    /**
     * @brief ブレーキストロークの現在値を取得する
     *
     * @retval ブレーキストローク現在値 
     */
    int GetActualBrakeStroke() const;
    /**
     * @brief ブレーキペダルからの入力値を取得する
     *
     * @retval ブレーキペダルからの入力値 
     */
    int GetInputBrakeStroke() const;
    /**
     * @brief ブレーキストローク目標値を取得する
     *
     * @retval ブレーキストローク目標値 
     */
    int GetTargetBrakeStroke() const;
    /**
     * @brief OBDから出力されたブレーキストローク現在値を取得する
     *
     * @retval OBDから出力されたブレーキストローク現在値 
     */
    float GetBrakeStrokeFromOBD() const;
    /**
     * @brief OBDから出力されたブレーキプレス状態を取得する
     *
     * @retval OBDから出力されたブレーキプレス状態 
     */
    bool GetBrakePressedFromOBD() const;

    unsigned char GetBrakeLamp() const;
    unsigned char GetBlinkerLeft() const;
    unsigned char GetBlinkerRight() const;
    unsigned char GetBrakeMode() const;

    /**
     * @brief ブレーキストロークの現在値を保存する
     *
     * @param[in]	brake_stroke コントロールボードから受信したブレーキストローク現在値 
     */
    void SetActualBrakeStroke(int brake_stroke);
    /**
     * @brief ブレーペダルからの入力値を保存する
     *
     * @param[in]	brake_stroke コントロールボードから受信したブレーキペダル入力値 
     */
    void SetInputBrakeStroke(int brake_stroke);
    /**
     * @brief ブレーキストロークの目標値を保存する
     *
     * @param[in]	brake_stroke コントロールボードから受信したブレーキストローク目標値 
     */
    void SetTargetBrakeStroke(int target_brake_stroke);
    /**
     * @brief OBDから受信したブレーキストロークの現在値を保存する
     *
     * @param[in]	brake_stroke OBDから受信したブレーキストローク現在値 
     */
    void SetBrakeStrokeFromOBD(float stroke);
    /**
     * @brief OBDから受信したブレーキストロークが一定値を越えているかを保存する
     *
     * @param[in]	brake_stroke OBDから受信したブレーキストロークが一定値を越えているか 
     */
    void SetBrakePressedFromOBD(bool pressed);

    void SetBrakeLamp(unsigned char lamp);
    void SetBlinkerLeft(unsigned char blink_left);
    void SetBlinkerRight(unsigned char blink_right);
    void SetBrakeMode(unsigned char mode);

private:
    int	_target_brake_stroke;	/*!< @brief ブレーキペダルストローク指令値 */
    int	_actual_brake_stroke;	/*!< @brief ブレーキペダルストローク値 */
    int	_input_brake_stroke;	/*!< @brief ブレーキペダルストローク値 */
    float	_brake_stroke_fromObd;
    bool _brake_pressed;	/*!< @brief ブレーキのONOFF */
    unsigned char _brake_lamp;
    unsigned char _blinker_left;
    unsigned char _blinker_right;
    unsigned char _brake_mode;

};
    }
}

#endif /* BRAKEINFORMATION_H_ */
