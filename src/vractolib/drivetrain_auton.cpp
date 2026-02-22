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
            int volt = applyFeedforwardFloor(
                static_cast<int>(std::lround(pidOut * 100.0)),
                err,
                aErr,
                minVoltErr,
                feedforward
            );

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

            int volt = applyFeedforwardFloor(
                static_cast<int>(std::lround(pidOut * 100.0)),
                err,
                aErr,
                minVoltErr,
                feedforward
            );
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
        const double distAErr = 0.20;
        const double distFullFloorErr = 1.00;
        const double settleDistErr = 0.25;
        const double turnRampDist = 10.0;
        const double minTurnScale = 0.20;

        int settledTicks = 0;
        int elapsedTicks = 0;

        latPID.reset();
        turnPID.reset();

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const vunits::Pose curPose = odom->getPose();

            const double dx = pose.x - curPose.x;
            const double dy = pose.y - curPose.y;
            const double dist = std::hypot(dx, dy);
            double targetAngle = std::atan2(dx, dy); // swapped because we do not use standard unit-circle heading

            if (isBackwards) {
                targetAngle = vunits::wrapToSignedRadians(targetAngle + vunits::PI);
            }

            double distErr = dist * std::cos(vunits::angleDiffRadians(curPose.theta, targetAngle));

            if (isBackwards) {
                distErr = -distErr;
            }

            double headErr = vunits::angleDiffRadians(curPose.theta, targetAngle);
            
            const double latPIDOut = latPID.update(distErr);
            double turnPIDOut = turnPID.update(headErr);

            //gradually slow turning as we get closer to target
            const double turnScaleRaw = dist / turnRampDist;
            const double turnScale = std::clamp(turnScaleRaw, minTurnScale, 1.0);
            turnPIDOut *= turnScale;

            const int latVolt = applyFeedforwardFloor(
                static_cast<int>(std::lround(latPIDOut * 100.0)),
                distErr,
                distAErr,
                distFullFloorErr,
                feedforward
            );
            const int turnVolt = static_cast<int>(std::lround(turnPIDOut * 100.0));
            double left = latVolt + turnVolt;
            double right = latVolt - turnVolt;
            double max = std::max(std::fabs(left), std::fabs(right));

            if (max > maxVolt) {
                left *= maxVolt / max;
                right *= maxVolt / max;
            }

            rightMotors.move_voltage(static_cast<int>(right));
            leftMotors.move_voltage(static_cast<int>(left));

            if (dist <= settleDistErr && std::fabs(max) <= maxVolt * 0.15) {
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

    void Drivetrain::moveToPose(vunits::Pose pose, double lead, int timeout, int settleTime, int maxVolt, int feedforward) {
        const double distAErr = 0.20;
        const double distFullFloorErr = 1.00;
        const double settleDistErr = 0.25;
        const double settleHeadErr = vunits::degToRad(2.0);
        const double turnRampDist = 10.0;
        const double minTurnScale = 0.20;
        const double headingAErr = vunits::degToRad(0.5);
        const double headingFullFloorErr = vunits::degToRad(2.0);
        const double maxLookahead = 18.0;

        lead = std::clamp(lead, 0.0, 1.0);

        int settledTicks = 0;
        int elapsedTicks = 0;

        latPID.reset();
        turnPID.reset();

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const vunits::Pose curPose = odom->getPose();

            const double dx = pose.x - curPose.x;
            const double dy = pose.y - curPose.y;
            const double dist = std::hypot(dx, dy);
            const double finalHeadErr = vunits::angleDiffRadians(curPose.theta, pose.theta);

            const bool atPosition = dist <= settleDistErr;
            const bool atHeading = std::fabs(finalHeadErr) <= settleHeadErr;

            double left = 0.0;
            double right = 0.0;

            if (!atPosition) {
                //target a point behind final pose along final heading
                const double hX = std::sin(pose.theta);
                const double hY = std::cos(pose.theta);
                const double lookahead = std::clamp(lead * dist, 0.0, maxLookahead);

                double carrotX = pose.x - hX * lookahead;
                double carrotY = pose.y - hY * lookahead;
                if (dist <= 1e-6) {
                    carrotX = pose.x;
                    carrotY = pose.y;
                }

                const double targetAngle = std::atan2(carrotX - curPose.x, carrotY - curPose.y); // swapped for this heading convention
                const double headErr = vunits::angleDiffRadians(curPose.theta, targetAngle);
                const double driveErr = dist * std::cos(headErr);

                const double latPIDOut = latPID.update(driveErr);
                double turnPIDOut = turnPID.update(headErr);

                const double turnScale = std::clamp(dist / turnRampDist, minTurnScale, 1.0);
                turnPIDOut *= turnScale;

                const int latVolt = applyFeedforwardFloor(
                    static_cast<int>(std::lround(latPIDOut * 100.0)),
                    driveErr,
                    distAErr,
                    distFullFloorErr,
                    feedforward
                );
                const int turnVolt = static_cast<int>(std::lround(turnPIDOut * 100.0));

                left = latVolt + turnVolt;
                right = latVolt - turnVolt;
            } else {
                // if position is good, finish heading in place
                const double turnPIDOut = turnPID.update(finalHeadErr);
                const int turnVolt = applyFeedforwardFloor(
                    static_cast<int>(std::lround(turnPIDOut * 100.0)),
                    finalHeadErr,
                    headingAErr,
                    headingFullFloorErr,
                    feedforward
                );
                left = turnVolt;
                right = -turnVolt;
            }

            double max = std::max(std::fabs(left), std::fabs(right));
            if (max > maxVolt) {
                left *= maxVolt / max;
                right *= maxVolt / max;
                max = maxVolt;
            }

            rightMotors.move_voltage(static_cast<int>(right));
            leftMotors.move_voltage(static_cast<int>(left));

            if (atPosition && atHeading && max <= maxVolt * 0.15) {
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
