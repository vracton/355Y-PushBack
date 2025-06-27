#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "main.h"
#include "pros/motor_group.hpp"

namespace vractoliib {
    class Drivetrain {
        private:
            pros::MotorGroup &leftMotors;
            pros::MotorGroup &rightMotors;

            void arcade_handle_input(int forward, int turn, std::function<int(int)> mappedVolt);
        
        public:
            Drivetrain(pros::MotorGroup &left, pros::MotorGroup &right) 
                : leftMotors(left), rightMotors(right) {}
            
            void arcade(pros::Controller controller, std::function<int(int)> mappedVolt);
            void arcadeDoubleStick(pros::Controller controller, std::function<int(int)> mappedVolt);
            void tank(pros::Controller controller, std::function<int(int)> mappedVolt);
    };
}

#endif