#include <vector>
#include <cmath>
#include "vractolib/utils/units/vector3d.h"
#include "vractolib/utils/units/pose.h"
#include "vractolib/algos/odom/trackingwheel.h"
#include "vractolib/algos/odom/imu.h"

#ifndef ODOM_MANAGER_H
#define ODOM_MANAGER_H

namespace vractolib {
	class OdomManager {
		private:
		std::vector<TrackingWheel> horizWheels;
		std::vector<TrackingWheel> vertWheels;
		IMU imu;

		vunits::Pose pose; //x, y, heading in radians
		vunits::Pose LastDelta; //last delta values (for mcl)

		public:
		OdomManager(std::vector<TrackingWheel> hWheels, std::vector<TrackingWheel> vWheels, IMU imu) : horizWheels(hWheels), vertWheels(vWheels), imu(imu), pose{0.0, 0.0, 0.0} { };

		void init();
		void reset();
		void update();

		//send reference for efficiency
		const vunits::Pose& getPose() const {
			return pose;
		}
	};
}

#endif