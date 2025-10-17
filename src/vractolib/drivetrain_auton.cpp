#include "vractolib/drivetrain.h"

namespace vractolib {
	void Drivetrain::turnTo(double angle, int timeout, int settleTime, int maxVolt) {
        const double absTarget = vunits::degToRad(angle);

        // find nearest target angle
        const double k = std::round((odom.getPose().theta - absTarget) / vunits::TAU);
        const double normTarget = absTarget + vunits::TAU * k;

        const double aErr = vunits::degToRad(3.0); // acceptable error
        int settledTicks = 0;
        int elapsedTicks = 0;

        turnPID.setTarget(normTarget);

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const double curHeading = odom.getPose().theta;
            const double pidOut = turnPID.update(curHeading);
            
            int volt = static_cast<int>(std::round(pidOut));
            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;

            rightMotors.move_voltage(volt);
            leftMotors.move_voltage(-volt);

            const double err = std::fabs(normTarget - curHeading);
            settledTicks += (err <= aErr && std::abs(volt) <= maxVolt * 0.15);
            
            pros::delay(vconfig::updateRate);
            elapsedTicks += 1;
        }

        rightMotors.brake();
        leftMotors.brake();
    }
}