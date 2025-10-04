#include "vractolib/algos/odom/imu.h"

namespace vractolib {
    double IMU::getHeading() {
        double heading;
        if (useYaw) {
            heading = imu.get_heading();
        } else {
            heading = imu.get_rotation();
        }
        return vunits::degToRad(heading) - offset;
    }

    double IMU::getDelta() {
        double currentHeading = getHeading();
        double delta = currentHeading - lastHeading;
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