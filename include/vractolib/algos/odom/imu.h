#include "pros/imu.hpp"
#include "vractolib/utils/units/vector3d.h"
#include "vractolib/utils/units/angles.h"

#ifndef IMU_H
#define IMU_H

namespace vractolib {
    class IMU {
    private:
        pros::IMU &imu;
        bool useYaw; //true: use heading(x), false: use position(z)
        double lastHeading;
    public:
        IMU(pros::IMU &imuSensor) : imu(imuSensor), useYaw(true), lastHeading(0) { };
        IMU(pros::IMU &imuSensor, bool useYaw) : imu(imuSensor), useYaw(useYaw), lastHeading(0) { };

        double getHeading();
        double getDelta();
        vunits::Vector3D<double> getLinearAccel();
        void calibrate();
    };
}

#endif