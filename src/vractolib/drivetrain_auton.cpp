#include "vractolib/drivetrain.h"
#include <cmath>
#include <string>

namespace vractolib {
    void Drivetrain::turnTo(double angle, int timeout, int settleTime, int maxVolt, int feedforward) {
        const double targetHeading = vunits::wrapToSignedRadians(vunits::degToRad(angle));

        const double aErr = vunits::degToRad(0.5); // acceptable error
        int settledTicks = 0;
        int elapsedTicks = 0;

        turnPID.reset();

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const double curHeading = odom->getPose().theta;
            const double err = vunits::angleDiffRadians(curHeading, targetHeading);
            
            const double pidOut = turnPID.update(err);
            
            int volt = static_cast<int>((pidOut + (std::fabs(err) > aErr ? feedforward * (err > 0 ? 1 : -1) : 0)) * 100);
            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;
			// printf("out: %d\n", volt);
            rightMotors.move_voltage(-volt);
            leftMotors.move_voltage(volt);

            settledTicks += (std::fabs(err) <= aErr && std::abs(volt) <= maxVolt * 0.25) ? 1 : 0;
			printf("err: %f, pidout: %f, volt: %d\n", vunits::radToDeg(err), pidOut, volt);
            pros::delay(vconfig::updateRate);
            elapsedTicks += 1;
        }

        rightMotors.brake();
        leftMotors.brake();
        printf("final heading: %f, elapsed: %d, settled: %d\n", vunits::radToDeg(odom->getPose().theta), elapsedTicks, settledTicks);
    }

    void Drivetrain::move(double distance, int timeout, int settleTime, int maxVolt, int feedforward) {
        const vunits::Pose startPose = odom->getPose();
        const double targetDist = distance;

        const double aErr = 0.2; // acceptable error

        int settledTicks = 0;
        int elapsedTicks = 0;

        latPID.reset();

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const vunits::Pose curPose = odom->getPose();

            double curDist = std::sqrt(std::pow(curPose.x - startPose.x, 2) + std::pow(curPose.y - startPose.y, 2));
            if (distance < 0) {
                curDist = -curDist;
            }
            
            const double err = targetDist - curDist;
            const double pidOut = latPID.update(err);

            printf("pidout: %f, err: %f, dist: %f\n", pidOut, err, curDist);

            int volt = static_cast<int>((pidOut + (std::fabs(err) > aErr ? feedforward * (err > 0 ? 1 : -1) : 0)) * 100);
            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;

            rightMotors.move_voltage(volt);
            leftMotors.move_voltage(volt);

            settledTicks += (std::fabs(err) <= aErr && std::abs(volt) <= maxVolt * 0.15) ? 1 : 0;

            pros::delay(vconfig::updateRate);
            elapsedTicks += 1;
        }

        rightMotors.brake();
        leftMotors.brake();
        const vunits::Pose curPose = odom->getPose();

        double curDist = std::sqrt(std::pow(curPose.x - startPose.x, 2) + std::pow(curPose.y - startPose.y, 2));
        if (targetDist < 0) {
            curDist = -curDist;
        }

        printf("final dist: %f, elapsed: %d, settled: %d\n", curDist, elapsedTicks, settledTicks);
    }
}