#ifndef TRACKING_WHEEL_H
#define TRACKING_WHEEL_H

#include "main.h"
#include "pros/rotation.hpp"

namespace vractolib {
    class TrackingWheel {
    private:
        pros::Rotation &encoder;
        double diameter;
        double offset;

        double lastTotal;

        /*
            Gets the current position of the tracking wheel in radians.
        */
        double getPos();
    public:
        TrackingWheel(pros::Rotation &enc, double diam);

        double getDelta();
        double getOffset();
        void reset();
        void calibrate();
    };
}

#endif