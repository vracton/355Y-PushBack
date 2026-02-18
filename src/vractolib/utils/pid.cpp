#include "vractolib/utils/pid.h"

namespace vractolib {
    PID::PID(double kP, double kI, double kD) 
        : gains{kP, kI, kD}, integral(0), prevError(0), starting(true) {};
    
    PID::PID(PIDGains gains) 
        : gains(gains), integral(0), prevError(0), starting(true) {};

    void PID::init() {
        PID::reset();
    }

    void PID::setGains(PIDGains newGains) {
        gains = newGains;
    }

    PIDGains PID::getGains() const {
        return gains;
    }

    void PID::reset() {
        integral = 0;
        prevError = 0;
        starting = true;
        lastTime = pros::millis();
    }

    double PID::update(double err) {
        const uint32_t curTime = pros::millis();

        double dt = (curTime - lastTime) / 1000.0;
        lastTime = curTime;

        if (dt <= 0) {
            dt = 1e-6;
        }

        integral += err * dt;

        double deriv = 0.0;
        if (!starting) { // avoid large spike on first cycle
            deriv = (err - prevError) / dt;
        }

        prevError = err;
        starting = false;

        return gains.kP * err + gains.kI * integral + gains.kD * deriv;
    }
}
