#include "vractolib/drivetrain.h"
#include "pros/rtos.hpp"

namespace vractolib {
    Drivetrain::Drivetrain(pros::MotorGroup &left, pros::MotorGroup &right, vractolib::PIDGains latGains, vractolib::PIDGains turnGains, vractolib::OdomManager odomManager) : leftMotors(left), rightMotors(right), latPID(latGains), turnPID(turnGains), odom(odomManager) { };
    
    int Drivetrain::linearVoltMap(int input) {
        // [-127, 127] -> [-12000, 12000]
        return std::min(input / 127.0 * 12000.0, 12000.0 / 100.0 * vconfig::maxVel);
    }

    void Drivetrain::init() {
        latPID.init();
        turnPID.init();
        odom.init();
    }

    //control
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

    //basic auton
    void Drivetrain::turnTo(double angle, int timeout, int settleTime, int maxVolt) {
        const double absTarget = vunits::degToRad(angle);

        // find nearest target angle
        const double k = std::round((odom.getPose().theta - absTarget) / vunits::TAU);
        const double normTarget = absTarget + vunits::TAU * k;

        const double aErr = vunits::degToRad(3.0); // acceptable error
        int settledTicks = 0;
        int elapsedTicks = 0;

        turnPID.setTarget(normTarget);

        while (elapsedTicks * vconfig::updateRate < timeout && settledTicks * vconfig::updateRate < settleTime) {
            const double curHeading = odom.getPose().theta;
            const double pidOut = turnPID.update(curHeading);
            
            int volt = static_cast<int>(std::round(pidOut));
            if (volt > maxVolt) volt = maxVolt;
            if (volt < -maxVolt) volt = -maxVolt;

            rightMotors.move_voltage(volt);
            leftMotors.move_voltage(-volt);

            const double err = std::fabs(normTarget - curHeading);
            settledTicks += (err <= aErr && std::abs(volt) <= maxVolt * 0.15);
            
            pros::delay(vconfig::updateRate);
            elapsedTicks += 1;
        }

        rightMotors.brake();
        leftMotors.brake();
    }
}