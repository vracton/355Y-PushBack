#include "pros/imu.hpp"
#include "vractolib/utils/units/vector3d.h"
#include "vractolib/utils/units/angles.h"

#ifndef IMU_H
#define IMU_H

namespace vractolib {
    class IMU {
    private:
        pros::IMU &imu;
        double offset; //heading offset in radians, +CCW

        bool useYaw; //true: use heading(x), false: use position(z)
        double lastHeading;
    public:
        IMU(pros::IMU &imuSensor, double offset) : imu(imuSensor), offset(offset), useYaw(true), lastHeading(0.0) { };
        IMU(pros::IMU &imuSensor, double offset, bool useYaw) : imu(imuSensor), offset(offset), useYaw(useYaw), lastHeading(0.0) { };

        /*
            Gets the current heading of the IMU in radians.
        */
        double getHeading();
        double getDelta();
        vunits::Vector3D<double> getLinearAccel();
        void calibrate();
        void reset();
    };
}

#endif
