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
            double target = 0;

        public:
            PID(double kP = 0, double kI = 0, double kD = 0) : gains{kP, kI, kD} {};

            PID(PIDGains gains) : gains(gains) {};

            void reset();

            void setTarget(double NewTarget);

            double updated(double at);
    };
}

#endif