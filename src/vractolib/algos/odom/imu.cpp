#include "vractolib/algos/odom/imu.h"

namespace vractolib {
    double IMU::getHeading() {
        if (useYaw) {
            return imu.get_heading();
        } else {
            return imu.get_rotation();
        }
    }

    double IMU::getDeltaHeading() {
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
}