#include "vractolib/algos/odom/odommanager.h"
#include <string>
#include <cmath>

namespace vractolib {
	OdomManager::OdomManager(std::vector<TrackingWheel> hWheels, std::vector<TrackingWheel> vWheels, IMU imu) : horizWheels(hWheels), vertWheels(vWheels), imu(imu), pose{0.0, 0.0, 0.0} { }

	void OdomManager::init() {
		OdomManager::reset();
	}

	void OdomManager::reset() {
		for (auto &wheel: horizWheels) {
			wheel.reset();
		}
		for (auto &wheel: vertWheels) {
			wheel.reset();
		}
		imu.reset();
		pose = vunits::Pose();
	}

	void OdomManager::update() {
		std::vector<std::pair<double, double>> horiz, vert;
		for (auto &wheel: horizWheels) {
			horiz.push_back({wheel.getDelta(),wheel.getOffset()});
		}

		/* TEMP CODE */
		if (horiz.size() == 0) {
			horiz.push_back({0.0,0.0});
		}
		/* END TEMP CODE */
		

		for (auto &wheel: vertWheels) {
			vert.push_back({wheel.getDelta(),wheel.getOffset()});
		}
		
		double dTheta = imu.getDelta();
		if (std::isinf(dTheta) || std::isnan(dTheta)) { //nan check
			dTheta = 0.0;
		}
		
		// if we have 2 horizontal/2 vertical wheels, we can employ a more accurate method of calculating heading, but since we do not plan to do this, we will forgo it for now

		vunits::Pose local = vunits::Pose(horiz[0].first, vert[0].first, 0.0);

		if (dTheta > 0.05) { // 0.05 is arbitrary for now
			local += vunits::Pose(horiz[0].first/dTheta, vert[0].first/dTheta, 0.0);
			local *= 2.0 * std::sin(dTheta/2);
		}
		
		pose += local.rotatedBy(pose.theta+dTheta/2);
		vunits::Pose change = local.rotatedBy(pose.theta+dTheta/2);
		
		pose.theta += dTheta;
	}
}