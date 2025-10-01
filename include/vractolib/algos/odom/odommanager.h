#include <vector>
#include "vractolib/utils/units/vector3d.h"
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

		vunits::Vector3D<double> pose; //x, y, heading in radians
		vunits::Vector3D<double> LastDelta; //last delta values (for mcl)

		public:
		OdomManager(std::vector<TrackingWheel> hWheels, std::vector<TrackingWheel> vWheels, IMU imu) : horizWheels(hWheels), vertWheels(vWheels), imu(imu), pose{0.0, 0.0, 0.0} { };

		void update();

		//send reference for efficiency
		const vunits::Vector3D<double>& getPose() const {
			return pose;
		}
	};
}

#endif