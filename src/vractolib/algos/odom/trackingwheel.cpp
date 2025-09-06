#include "vractolib/algos/odom/trackingwheel.h"

namespace vractolib {
    TrackingWheel::TrackingWheel(pros::Rotation &enc, double diam) 
        : encoder(enc), diameter(diam), lastTotal(0) {}

    double TrackingWheel::getPosDegrees() {
        return encoder.get_position()/100.0; //centidegs to degs
    }

    double TrackingWheel::getDelta() {
        double cur = getPosDegrees();
        double delta = cur - lastTotal;
        lastTotal = cur;
        return (delta / 360.0) * (diameter * M_PI);
    }

    double TrackingWheel::getOffset() {
        return offset;
    }

    void TrackingWheel::reset() {
        encoder.reset_position();
    }
}