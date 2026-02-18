#include "vractolib/drivetrain.h"
#include <cmath>
#include <string>

namespace vractolib {
    void Drivetrain::turnTo(double angle, int timeout, int settleTime, int maxVolt, int feedforward) {
        const double targetHeading = vunits::wrapToSignedRadians(vunits::degToRad(angle));

        const double aErr = vunits::degToRad(0.5); // acceptable error
        const double minVoltErr = vunits::degToRad(2.0); // apply stiction floor only when farther from target
        int settledTicks = 0;
        int elapsedTicks = 0;

        turnPID.reset();

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const double curHeading = odom->getPose().theta;
            const double err = vunits::angleDiffRadians(curHeading, targetHeading);
            
            const double pidOut = turnPID.update(err);
            int volt = static_cast<int>(std::lround(pidOut * 100.0));

            // use feedforward as a tapered minimum voltage floor to overcome stiction
            const int minTurnVolt = std::abs(feedforward) * 100;
            const int nearTurnVolt = std::max(100, minTurnVolt / 8);
            const double absErr = std::fabs(err);
            if (absErr > aErr && std::abs(volt) < minTurnVolt) {
                const double rawScale = (absErr - aErr) / (minVoltErr - aErr);
                const double floorScale = (rawScale < 0.0) ? 0.0 : ((rawScale > 1.0) ? 1.0 : rawScale);
                const int taperedFloor = static_cast<int>(std::lround(minTurnVolt * floorScale));
                const int appliedFloor = (absErr >= minVoltErr) ? minTurnVolt : std::max(nearTurnVolt, taperedFloor);
                volt = (err > 0 ? 1 : -1) * appliedFloor;
            }

            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;

            rightMotors.move_voltage(-volt);
            leftMotors.move_voltage(volt);

            if (std::fabs(err) <= aErr && std::abs(volt) <= maxVolt * 0.25) {
                settledTicks += 1;
            } else {
                settledTicks = 0;
            }

			// printf("err: %f, pidout: %f, volt: %d\n", vunits::radToDeg(err), pidOut, volt);

            pros::delay(vconfig::updateRate);
            elapsedTicks += 1;
        }

        rightMotors.brake();
        leftMotors.brake();
        // printf("final heading: %f, elapsed: %d, settled: %d\n", vunits::radToDeg(odom->getPose().theta), elapsedTicks, settledTicks);
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

            double curDist = vunits::Pose::distance(startPose, curPose);
            if (distance < 0) {
                curDist = -curDist;
            }
            
            const double err = targetDist - curDist;
            const double pidOut = latPID.update(err);

            // printf("pidout: %f, err: %f, dist: %f\n", pidOut, err, curDist);

            int volt = static_cast<int>((pidOut + (std::fabs(err) > aErr ? feedforward * (err > 0 ? 1 : -1) : 0)) * 100);
            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;

            rightMotors.move_voltage(volt);
            leftMotors.move_voltage(volt);

            if (std::fabs(err) <= aErr && std::abs(volt) <= maxVolt * 0.15) {
                settledTicks += 1;
            } else {
                settledTicks = 0;
            }

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

        // printf("final dist: %f, elapsed: %d, settled: %d\n", curDist, elapsedTicks, settledTicks);
    }

    void Drivetrain::pointTowards(vunits::Pose pose, int timeout, int settleTime, int maxVolt, int feedforward) {
        const double targetAngle = std::atan2(pose.x - odom->getPose().x, pose.y - odom->getPose().y); //swapped because we don't use standard unit circle for heading

        Drivetrain::turnTo(vunits::radToDeg(targetAngle), timeout, settleTime, maxVolt, feedforward);
    }

    void Drivetrain::moveToPoint(vunits::Pose pose, bool isBackwards, int timeout, int settleTime, int maxVolt, int feedforward) {
        const double maxDistErr = 0.2;

        int settledTicks = 0;
        int elapsedTicks = 0;

        latPID.reset();
        turnPID.reset();

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const vunits::Pose curPose = odom->getPose();

            double targetAngle = std::atan2(pose.x - curPose.x, pose.y - curPose.y); //swapped becauuse we don't use standard unit circle for heading

            if (isBackwards) {
                targetAngle = vunits::wrapToSignedRadians(targetAngle + vunits::PI);
            }

            double distErr = vunits::Pose::distance(curPose, pose) * std::cos(vunits::angleDiffRadians(curPose.theta, targetAngle));

            if (isBackwards) {
                distErr = -distErr;
            }

            double headErr = vunits::angleDiffRadians(curPose.theta, targetAngle);
            
            const double latPIDOut = latPID.update(distErr);
            double turnPIDOut = turnPID.update(headErr);

            // printf("latPIDout: %f, err: %f, dist: %f\n", latPIDOut, distErr, vunits::Pose::distance(curPose, pose));
            // printf("curPose: %f, %f, %f | targetPose: %f, %f, %f\n\n", curPose.x, curPose.y, vunits::radToDeg(curPose.theta), pose.x, pose.y, vunits::radToDeg(pose.theta));

            // double turnScaler = std::clamp(vunits::Pose::distance(curPose, pose) / 6.7, 0.0, 1.0);
            // turnPIDOut *= turnScaler;

            if (std::fabs(vunits::Pose::distance(curPose, pose)) < 4.0) {
                turnPIDOut = 0;
            }

            double latVolt = (latPIDOut + (std::fabs(distErr) > maxDistErr ? feedforward * (distErr > 0 ? 1 : -1) : 0)) * 100;
            double turnVolt = turnPIDOut * 100;
            double left = latVolt + turnVolt;
            double right = latVolt - turnVolt;
            double max = std::max(std::fabs(left), std::fabs(right));

            if (max > maxVolt) {
                left *= maxVolt / max;
                right *= maxVolt / max;
            }

            rightMotors.move_voltage(static_cast<int>(right));
            leftMotors.move_voltage(static_cast<int>(left));

            if (std::fabs(distErr) <= maxDistErr && std::fabs(max) <= maxVolt * 0.15) {
                settledTicks += 1;
            } else {
                settledTicks = 0;
            }

            pros::delay(vconfig::updateRate);
            elapsedTicks += 1;
        }

        rightMotors.brake();
        leftMotors.brake();
    }
}
