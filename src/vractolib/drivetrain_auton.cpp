#include "vractolib/drivetrain.h"
#include <algorithm>
#include <cmath>
#include <string>

namespace vractolib {
    int Drivetrain::applyFeedforwardFloor(int pidVolt, double err, double acceptableErr, double fullFloorErr, int feedforward, int minNearVolt, int nearDivisor) const {
        const int minFloorVolt = std::abs(feedforward) * 100;
        if (minFloorVolt <= 0 || nearDivisor <= 0 || fullFloorErr <= acceptableErr) return pidVolt;

        const double absErr = std::fabs(err);
        if (absErr <= acceptableErr || std::abs(pidVolt) >= minFloorVolt) return pidVolt;

        const int nearFloorVolt = std::max(minNearVolt, minFloorVolt / nearDivisor);
        const double rawScale = (absErr - acceptableErr) / (fullFloorErr - acceptableErr);
        const double floorScale = (rawScale < 0.0) ? 0.0 : ((rawScale > 1.0) ? 1.0 : rawScale);
        const int taperedFloor = static_cast<int>(std::lround(minFloorVolt * floorScale));
        const int appliedFloor = (absErr >= fullFloorErr) ? minFloorVolt : std::max(nearFloorVolt, taperedFloor);
        return (err > 0 ? 1 : -1) * appliedFloor;
    }

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
            int volt = applyFeedforwardFloor(static_cast<int>(std::lround(pidOut * 100.0)), err, aErr, minVoltErr, feedforward);

            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;

            rightMotors.move_voltage(-volt);
            leftMotors.move_voltage(volt);

            if (std::fabs(err) <= aErr && std::abs(volt) <= maxVolt * 0.25) {
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

    void Drivetrain::move(double distance, int timeout, int settleTime, int maxVolt, int feedforward) {
        const vunits::Pose startPose = odom->getPose();
        const double targetDist = distance;

        const double aErr = 0.25; // acceptable error
        const double minVoltErr = 4.0; // apply full floor at and beyond this error

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

            int volt = applyFeedforwardFloor(static_cast<int>(std::lround(pidOut * 100.0)), err, aErr, minVoltErr, feedforward);
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
