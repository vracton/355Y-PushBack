#include "vractolib/algos/odom/trackingwheel.h"

namespace vractolib {
    TrackingWheel::TrackingWheel(pros::Rotation &enc, double diam, double offset, bool reversed) 
        : encoder(enc), diameter(diam), offset(offset), lastTotal(0), reversed(reversed) {}

    double TrackingWheel::getPos() {
        return vunits::centiToRad(encoder.get_position());
    }

    double TrackingWheel::getDelta() {
        double cur = (reversed ? -1.0 : 1.0) * getPos();
        double delta = cur - lastTotal;
        lastTotal = cur;
        return delta * (diameter / 2.0);
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