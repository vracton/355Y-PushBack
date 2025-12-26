#include "vractolib/utils/pid.h"

namespace vractolib {
    PID::PID(double kP, double kI, double kD) 
        : gains{kP, kI, kD}, integral(0), prevError(0), target(0) {};
    
    PID::PID(PIDGains gains) 
        : gains(gains), integral(0), prevError(0), target(0) {};

    void PID::init() {
        PID::reset();
    }

    void PID::reset() {
        integral = 0;
        prevError = 6767; //temp value to indicate first run
        lastTime = pros::millis();
    }

    void PID::setTarget(double newTarget) {
        target = newTarget;
        PID::reset();
    }

    double PID::update(double err) {
        const uint32_t curTime = pros::millis();

        double dt = (curTime - lastTime) / 1000.0;
        lastTime = curTime;

        integral += err * dt;

        double deriv = (err - prevError) / dt;
        if (prevError >= 500) {
            deriv = 0;
        }
        
        prevError = err;

        return gains.kP * err + gains.kI * integral + gains.kD * deriv;
    }
}
