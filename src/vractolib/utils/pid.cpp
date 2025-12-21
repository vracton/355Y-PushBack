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
        prevError = 0;
        lastTime = pros::millis();
    }

    void PID::setTarget(double newTarget) {
        target = newTarget;
        PID::reset();
    }

    double PID::update(double at) {
        const uint32_t curTime = pros::millis();
        const double error = target - at;

        double dt = (curTime - lastTime) / 1000.0;
        lastTime = curTime;

        integral += error * dt;

        double deriv = (error - prevError) / dt;
        if (prevError = 0) {
            deriv = 0;
        }
        
        prevError = error;

        return gains.kP * error + gains.kI * integral + gains.kD * deriv;
    }
}
