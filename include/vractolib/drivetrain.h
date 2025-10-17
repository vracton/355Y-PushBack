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
        vractolib::OdomManager odom;

        void arcade_handle_input(int forward, int turn, std::function<int(int)> mappedVolt);
        static int linearVoltMap(int input);

        public:
        Drivetrain(pros::MotorGroup &left, pros::MotorGroup &right, vractolib::PIDGains latGains, vractolib::PIDGains turnGains, vractolib::OdomManager odomManager);

        void init();

        //basic auton
        void turnTo(double angle, int maxVolt = vconfig::maxVolt);

        //control methods
        void arcade(pros::Controller controller, std::function<int(int)> mappedVolt = linearVoltMap);
        void arcadeDoubleStick(pros::Controller controller, std::function<int(int)> mappedVolt = linearVoltMap);
        void tank(pros::Controller controller, std::function<int(int)> mappedVolt = linearVoltMap);
    };
}

#endif