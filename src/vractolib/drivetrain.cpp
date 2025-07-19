#include "vractolib/drivetrain.h"
#include "vractolib/config.h"
#include "main.h"

namespace vractolib {
    Drivetrain::Drivetrain(pros::MotorGroup &left, pros::MotorGroup &right) : leftMotors(left), rightMotors(right) { }

    void Drivetrain::arcade_handle_input(int forward, int turn, std::function<int(int)> mappedVolt) {
        if (abs(forward) > vconfig::deadzone) {
            leftMotors.move_voltage(mappedVolt(forward));
            rightMotors.move_voltage(mappedVolt(forward));
        } else if (abs(turn) > vconfig::deadzone) {
            leftMotors.move_voltage(mappedVolt(turn));
            rightMotors.move_voltage(-mappedVolt(turn));
        } else {
            leftMotors.move_voltage(0);
            rightMotors.move_voltage(0);
        }
    }

    void Drivetrain::arcade(pros::Controller controller, std::function<int(int)> mappedVolt) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        arcade_handle_input(leftY, leftX, mappedVolt);
    }

    void Drivetrain::arcadeDoubleStick(pros::Controller controller, std::function<int(int)> mappedVolt) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        arcade_handle_input(leftY, rightX, mappedVolt);
    }

    void Drivetrain::tank(pros::Controller controller, std::function<int(int)> mappedVolt) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        if (abs(leftY) > vconfig::deadzone) {
            leftMotors.move_voltage(mappedVolt(leftY));
        } else {
            leftMotors.move_voltage(0);
        }

        if (abs(rightY) > vconfig::deadzone) {
            rightMotors.move_voltage(mappedVolt(rightY));
        } else {
            rightMotors.move_voltage(0);
        }
    }
}