/**
 * @file		DemoSensorInformation.cpp
 * @brief		その他情報クラスヘッダファイル
 *
 * @date		2012/05/16
 * @author		m.ota
 * @par			Copyright:
 *				2012 XXX All rights reserved.
 */
#ifndef DEMO_SENSOR_INFORMATION_H_
#define DEMO_SENSOR_INFORMATION_H_

#include "HEVConst.h"

namespace zmp
{
	namespace minivan
	{
		// #ifdef USE_DEMO

#define N_OFZ 4
#define N_AVERAGE 12

		/**
		 * @brief 各種情報を保持するためのクラス
		 *
		 * @author		m.ota
		 * @date		2012/02/15
		 */
		class DemoSensorInformation
		{
		public:
			DemoSensorInformation();
			virtual ~DemoSensorInformation();

			int GetOfzValue(int index, float *v);
			float GetSeatSensor() const;

			void SetOfzValue(int index, const float *v);
			void SetSeatSensor(float v);

		private:
			float _seat_sensor;
			float _ofz_value[N_OFZ][N_AVERAGE];
		};
		// #endif
	}
}

#endif /* OTHERINFORMATION_H_ */
