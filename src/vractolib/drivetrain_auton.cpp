#include "vractolib/drivetrain.h"
#include <string>

namespace vractolib {
	void Drivetrain::turnTo(double angle, int timeout, int settleTime, int maxVolt) {
        const double normTarget = vunits::degToRad(angle);

        double cur = odom->getPose().theta;
        int k = cur / vunits::TAU;
        double absTarget = k * vunits::TAU + normTarget;
        if (absTarget - cur > vunits::PI) absTarget -= vunits::TAU;

        const double aErr = vunits::degToRad(0.67); // acceptable error
        int settledTicks = 0;
        int elapsedTicks = 0;

        turnPID.setTarget(absTarget);

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const double curHeading = odom->getPose().theta;
            const double pidOut = turnPID.update(curHeading);
            printf("%f\n", pidOut);
            int volt = static_cast<int>(std::round(pidOut) * 250);
            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;
            
            rightMotors.move_velocity(volt);
            leftMotors.move_velocity(-volt);

            const double err = std::fabs(absTarget - curHeading);
            printf("%f\n", err);
            settledTicks += (err <= aErr && std::abs(volt) <= maxVolt * 0.15);
            
            pros::delay(vconfig::updateRate);
            elapsedTicks += 1;
        }

        rightMotors.brake();
        leftMotors.brake();
    }

    void Drivetrain::move(double distance, int timeout, int settleTime, int maxVolt) {
        const vunits::Pose startPose = odom->getPose();
        const double targetDist = distance;

        const double aErr = 0.2; // acceptable error

        int settledTicks = 0;
        int elapsedTicks = 0;

        latPID.setTarget(targetDist);

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const vunits::Pose curPose = odom->getPose();
            double curDist = std::sqrt(std::pow(curPose.x - startPose.x, 2) + std::pow(curPose.y - startPose.y, 2));//(curPose.x - startPose.x) * std::cos(startPose.theta) + (curPose.y - startPose.y) * stdx::sin(startPose.theta); - need to change tot his eventually, but im getting fried by this
            if (distance < 0) {
                curDist = -curDist;
            }
            const double pidOut = latPID.update(curDist);

            printf("pidout: %f, dist: %f\n", pidOut, curDist);

            int volt = static_cast<int>(std::round(pidOut) * 100);
            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;

            rightMotors.move_voltage(volt);
            leftMotors.move_voltage(volt);

            const double err = std::fabs(targetDist - curDist);
            settledTicks += (err <= aErr && std::abs(volt) <= maxVolt * 0.15);

            pros::delay(vconfig::updateRate);
            elapsedTicks += 1;
        }

        rightMotors.brake();
        leftMotors.brake();
    }
}