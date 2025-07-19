#include "vractolib/pid.h"
#include "pros/rtos.hpp"

namespace vractolib {
    PID::PID(double kP, double kI, double kD) 
        : gains{kP, kI, kD}, integral(0), prevError(0), target(0) {};
    
    PID::PID(PIDGains gains) 
        : gains(gains), integral(0), prevError(0), target(0) {};

    void PID::reset() {
        integral = 0;
        prevError = 0;
    }

    void PID::setTarget(double NewTarget) {
        target = NewTarget;
        reset();
    }

    double PID::updated(double at) {
        const uint32_t curTime = pros::millis();
        const double error = target - at;

        double dt = (curTime - lastTime) / 1000.0;
        lastTime = curTime;

        integral += error * dt;

        double deriv = (error - prevError) / dt;
        prevError = error;

        return gains.kP * error + gains.kI * integral + gains.kD * deriv;
    }
}
