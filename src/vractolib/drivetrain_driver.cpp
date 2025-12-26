#include "vractolib/drivetrain.h"
#include "pros/rtos.hpp"

namespace vractolib {
    Drivetrain::Drivetrain(pros::MotorGroup &left, pros::MotorGroup &right, vractolib::PIDGains latGains, vractolib::PIDGains turnGains, vractolib::OdomManager* odomManager) : leftMotors(left), rightMotors(right), latPID(latGains), turnPID(turnGains), odom(odomManager) { };
    
    int Drivetrain::linearVoltMap(int input) {
        // [-127, 127] -> [-12000, 12000]
        return std::min(input / 127.0 * 12000.0, 12000.0 / 100.0 * vconfig::maxVel);
    }

    void Drivetrain::init(vunits::Pose startPose) {
        latPID.init();
        turnPID.init();
        odom->init(startPose);
        odom->init(startPose);
    }

    //setters
    void Drivetrain::setBrakeMode(pros::motor_brake_mode_e brakeMode) {
        leftMotors.set_brake_mode_all(brakeMode);
        rightMotors.set_brake_mode_all(brakeMode);
    }

    void Drivetrain::arcade_handle_input(int forward, int turn, std::function<int(int)> mappedVolt) {
        if (abs(turn) > vconfig::turnDeadzone || abs(forward) > vconfig::forwardDeadzone) {
            leftMotors.move_voltage(mappedVolt(0.67 * turn + forward));
            rightMotors.move_voltage(-mappedVolt(0.67 * turn - forward));
        } else {
            leftMotors.move_voltage(0);
            rightMotors.move_voltage(0);
        }
    }

    void Drivetrain::arcade(pros::Controller controller, std::function<int(int)> mappedVolt) {
        int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * (forward >= 0 ? -1 : 1);

        arcade_handle_input(forward, turn, mappedVolt);
    }

    void Drivetrain::arcadeDoubleStick(pros::Controller controller, std::function<int(int)> mappedVolt) {
        int forward = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        arcade_handle_input(forward, turn, mappedVolt);
    }

    void Drivetrain::tank(pros::Controller controller, std::function<int(int)> mappedVolt) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        if (abs(leftY) > vconfig::forwardDeadzone) {
            leftMotors.move_voltage(mappedVolt(leftY));
        } else {
            leftMotors.move_voltage(0);
        }

        if (abs(rightY) > vconfig::forwardDeadzone) {
            rightMotors.move_voltage(mappedVolt(rightY));
        } else {
            rightMotors.move_voltage(0);
        }
    }

    void Drivetrain::cheese(pros::Controller controller, std::function<int(int)> mappedVolt) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        cheese_handle_input(leftY, rightX, mappedVolt);
    }
}