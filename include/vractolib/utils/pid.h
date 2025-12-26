#include <cstdint>
#include "pros/rtos.hpp"

#ifndef PID_H
#define PID_H

namespace vractolib {
    struct PIDGains {
        double kP;
        double kI;
        double kD;
    };

    class PID {
        private:
            PIDGains gains;

            uint32_t lastTime = 0;
            double integral = 0;
            double prevError = 0;
            bool starting = true;

        public:
            PID(double kP = 0, double kI = 0, double kD = 0);
            PID(PIDGains gains);

            void reset();
            void init();

            double update(double err);
    };
}

#endif