#include "vractolib/algos/odom/imu.h"
#include <cmath>

namespace vractolib {
    double IMU::getHeading() {
        double heading;
        if (useYaw) {
            heading = imu.get_heading();
        } else {
            heading = imu.get_rotation();
        }
        return vunits::wrapToSignedRadians(vunits::degToRad(heading) - offset);
    }

    double IMU::getDelta() {
        double currentHeading = getHeading();

        if (!std::isfinite(currentHeading)) { //nan check
			currentHeading = 0.0;
		}
        if (!std::isfinite(lastHeading)) {
            lastHeading = currentHeading;
        }

        double delta = currentHeading - lastHeading;

        if (delta >= vunits::PI) delta -= vunits::TAU;
        if (delta < -vunits::PI) delta += vunits::TAU;

        lastHeading = currentHeading;
        return delta;
    }

    vunits::Vector3D<double> IMU::getLinearAccel() {
        auto rawAccel = imu.get_accel();
        return vunits::Vector3D<double>(rawAccel.x, rawAccel.y, rawAccel.z);
    }

    void IMU::calibrate() {
        imu.reset();
        lastHeading = getHeading();
    }

    void IMU::reset() {
        IMU::calibrate();
    }
}