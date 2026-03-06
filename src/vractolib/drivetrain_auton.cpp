#include "vractolib/drivetrain.h"
#include <algorithm>
#include <cmath>
#include <string>

namespace vractolib {
    static void mixLatTurnPrioritizeTurn(double latCmd, double turnCmd, int maxVolt, double &leftOut, double &rightOut) {
        // Preserve turn authority at high drive demand by clipping lateral command
        // to the remaining voltage budget after turn.
        const double turn = std::clamp(turnCmd, -static_cast<double>(maxVolt), static_cast<double>(maxVolt));
        const double maxLat = std::max(0.0, static_cast<double>(maxVolt) - std::fabs(turn));
        const double lat = std::clamp(latCmd, -maxLat, maxLat);
        leftOut = lat - turn;
        rightOut = lat + turn;
    }

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

            rightMotors.move_voltage(volt);
            leftMotors.move_voltage(-volt);

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
        const double targetAngle = std::atan2(pose.y - odom->getPose().y, pose.x - odom->getPose().x);

        Drivetrain::turnTo(vunits::radToDeg(targetAngle), timeout, settleTime, maxVolt, feedforward);
    }

    void Drivetrain::moveToPoint(vunits::Pose pose, bool isBackwards, int timeout, int settleTime, int maxVolt, int feedforward) {
        const double distAErr = 0.20;
        const double distFullFloorErr = 1.00;
        const double settleDistErr = 0.50;
        const double turnRampDist = 10.0;
        const double minTurnScale = 0.0;
        const double endTurnFadeDist = 3.0;
        const double turnDisableDist = 0.50;

        int settledTicks = 0;
        int elapsedTicks = 0;

        latPID.reset();
        turnPID.reset();

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const vunits::Pose curPose = odom->getPose();

            const double dx = pose.x - curPose.x;
            const double dy = pose.y - curPose.y;
            const double dist = std::hypot(dx, dy);
            double targetAngle = std::atan2(dy, dx);

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

            // Gradually reduce turning as we approach target to prevent late heading snap.
            const double turnScaleRaw = dist / turnRampDist;
            const double turnScaleBase = std::clamp(turnScaleRaw, minTurnScale, 1.0);
            const double endTurnScale = std::clamp(
                (dist - turnDisableDist) / std::max(1e-6, endTurnFadeDist - turnDisableDist),
                0.0,
                1.0
            );
            const double turnScale = turnScaleBase * endTurnScale;
            turnPIDOut *= turnScale;

            const int latVolt = applyFeedforwardFloor(
                static_cast<int>(std::lround(latPIDOut * 100.0)),
                distErr,
                distAErr,
                distFullFloorErr,
                feedforward
            );
            int turnVolt = static_cast<int>(std::lround(turnPIDOut * 100.0));
            if (dist <= turnDisableDist) {
                turnVolt = 0;
            } else {
                const int maxNearTurnVolt = static_cast<int>(std::lround(maxVolt * (0.25 + 0.75 * endTurnScale)));
                if (turnVolt > maxNearTurnVolt) turnVolt = maxNearTurnVolt;
                if (turnVolt < -maxNearTurnVolt) turnVolt = -maxNearTurnVolt;
            }
            double left = 0.0;
            double right = 0.0;
            mixLatTurnPrioritizeTurn(latVolt, turnVolt, maxVolt, left, right);
            const double max = std::max(std::fabs(left), std::fabs(right));

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

    void Drivetrain::moveToPose(vunits::Pose pose, double lead, bool isBackwards, int timeout, int settleTime, int maxVolt, int feedforward) {
        const double distAErr = 0.20;
        const double distFullFloorErr = 1.00;
        const double settleDistErr = 0.25;
        const double settleHeadErr = vunits::degToRad(3.0);
        const double turnRampDist = 10.0;
        const double minTurnScale = 0.20;
        const double headingAErr = vunits::degToRad(1.0);
        const double headingFullFloorErr = vunits::degToRad(4.0);
        const double maxLookahead = 18.0;
        const double turnFadeStartDist = 6.0;
        const double turnDisableDist = 0.60;
        const double finishTurnMaxFrac = 0.30;

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
            const double endTurnScale = std::clamp(
                (dist - turnDisableDist) / std::max(1e-6, turnFadeStartDist - turnDisableDist),
                0.0,
                1.0
            );

            double left = 0.0;
            double right = 0.0;

            if (!atPosition) {
                // Target a point along final heading:
                // forward: behind final pose, backward: ahead of final pose.
                const double hX = std::cos(pose.theta);
                const double hY = std::sin(pose.theta);
                const double lookahead = std::clamp(lead * dist, 0.0, maxLookahead);

                const double carrotSign = isBackwards ? 1.0 : -1.0;
                double carrotX = pose.x + carrotSign * hX * lookahead;
                double carrotY = pose.y + carrotSign * hY * lookahead;
                if (dist <= 1e-6) {
                    carrotX = pose.x;
                    carrotY = pose.y;
                }

                const double travelAngle = std::atan2(carrotY - curPose.y, carrotX - curPose.x);
                const double targetAngle = isBackwards
                    ? vunits::wrapToSignedRadians(travelAngle + vunits::PI)
                    : travelAngle;
                const double headErr = vunits::angleDiffRadians(curPose.theta, targetAngle);
                double driveErr = dist * std::cos(headErr);
                if (isBackwards) {
                    driveErr = -driveErr;
                }

                const double latPIDOut = latPID.update(driveErr);
                double turnPIDOut = turnPID.update(headErr);

                const double turnScale = std::clamp(dist / turnRampDist, minTurnScale, 1.0) * endTurnScale;
                turnPIDOut *= turnScale;

                const int latVolt = applyFeedforwardFloor(
                    static_cast<int>(std::lround(latPIDOut * 100.0)),
                    driveErr,
                    distAErr,
                    distFullFloorErr,
                    feedforward
                );
                int turnVolt = static_cast<int>(std::lround(turnPIDOut * 100.0));
                const int movingTurnCap = static_cast<int>(std::lround(
                    maxVolt * (finishTurnMaxFrac + (1.0 - finishTurnMaxFrac) * endTurnScale)
                ));
                if (turnVolt > movingTurnCap) turnVolt = movingTurnCap;
                if (turnVolt < -movingTurnCap) turnVolt = -movingTurnCap;
                if (dist <= turnDisableDist) turnVolt = 0;

                mixLatTurnPrioritizeTurn(latVolt, turnVolt, maxVolt, left, right);
            } else {
                // if position is good, finish heading in place
                const double turnPIDOut = turnPID.update(finalHeadErr);
                int turnVolt = applyFeedforwardFloor(
                    static_cast<int>(std::lround(turnPIDOut * 100.0)),
                    finalHeadErr,
                    headingAErr,
                    headingFullFloorErr,
                    feedforward
                );
                const int finishTurnCap = static_cast<int>(std::lround(maxVolt * finishTurnMaxFrac));
                if (turnVolt > finishTurnCap) turnVolt = finishTurnCap;
                if (turnVolt < -finishTurnCap) turnVolt = -finishTurnCap;
                if (std::fabs(finalHeadErr) <= settleHeadErr) turnVolt = 0;
                left = -turnVolt;
                right = turnVolt;
            }

            const double max = std::max(std::fabs(left), std::fabs(right));

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
