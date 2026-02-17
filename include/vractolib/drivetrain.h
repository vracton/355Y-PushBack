#include "pros/motor_group.hpp"
#include "vractolib/config.h"
#include "vractolib/utils/pid.h"
#include "vractolib/algos/odom/odommanager.h"
#include "vractolib/utils/units/angles.h"

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

namespace vractolib {
    class Drivetrain {
        private:
        pros::MotorGroup &leftMotors;
        pros::MotorGroup &rightMotors;
        vractolib::PID latPID, turnPID;
        vractolib::OdomManager* odom;

        void arcade_handle_input(int forward, int turn, std::function<int(int)> mappedVolt);
        void cheese_handle_input(int forward, int curve, std::function<int(int)> mappedVolt);
        static int linearVoltMap(int input);

        public:
        Drivetrain(pros::MotorGroup &left, pros::MotorGroup &right, vractolib::PIDGains latGains, vractolib::PIDGains turnGains, vractolib::OdomManager* odomManager);

        void init(vunits::Pose startPose = vunits::Pose{0.0, 0.0, 0.0});

        //setters
        void setBrakeMode(pros::motor_brake_mode_e brakeMode);

        //auton
        void turnTo(double angle, int timeout = 3000, int settleTime = 150, int maxVolt = vconfig::maxVolt, int feedforward = 14);
        void move(double distance, int timeout = 3000, int settleTime = 150, int maxVolt = vconfig::maxVolt, int feedforward = 14);
        void pointTowards(vunits::Pose pose, int timeout = 3000, int settleTime = 150, int maxVolt = vconfig::maxVolt, int feedforward = 14);
        void moveToPoint(vunits::Pose pose, bool isBackwards = false, int timeout = 3000, int settleTime = 150, int maxVolt = vconfig::maxVolt, int feedforward = 14);
        
        void moveToPose(vunits::Pose pose, int timeout = 3000, int settleTime = 150, int maxVolt = vconfig::maxVolt, int feedforward = 14);

        //control methods
        void arcade(pros::Controller controller, std::function<int(int)> mappedVolt = linearVoltMap);
        void arcadeDoubleStick(pros::Controller controller, std::function<int(int)> mappedVolt = linearVoltMap);
        void tank(pros::Controller controller, std::function<int(int)> mappedVolt = linearVoltMap);
        void cheese(pros::Controller controller, std::function<int(int)> mappedVolt = linearVoltMap);
    };
}

#endif