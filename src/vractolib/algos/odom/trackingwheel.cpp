#include "vractolib/algos/odom/trackingwheel.h"

namespace vractolib {
    TrackingWheel::TrackingWheel(pros::Rotation &enc, double diam) 
        : encoder(enc), diameter(diam), lastTotal(0) {}

    double TrackingWheel::getPos() {
        return vunits::centiToRad(encoder.get_position());
    }

    double TrackingWheel::getDelta() {
        double cur = getPos();
        double delta = cur - lastTotal;
        lastTotal = cur;
        return delta / 2 * diameter;
    }

    double TrackingWheel::getOffset() {
        return offset;
    }

    void TrackingWheel::calibrate() {
        TrackingWheel::reset();
    }

    void TrackingWheel::reset() {
        encoder.reset_position();
        lastTotal = 0;
    }
}