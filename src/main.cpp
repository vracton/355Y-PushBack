#include "main.h"
using namespace pros;

//drive motors
MotorGroup driveRight({11, 12, -3}, v5::MotorGears::blue, v5::MotorUnits::degrees);
MotorGroup driveleft({-14, -15, 16}, v5::MotorGears::blue, v5::MotorUnits::degrees);

//intake motors
Motor intakeLow(1, v5::MotorGearset::blue, v5::MotorUnits::degrees);
Motor intakeHigh(2, v5::MotorGearset::blue, v5::MotorUnits::degrees);

//controller
Controller master(E_CONTROLLER_MASTER);

vractolib::Drivetrain dt(driveleft, driveRight);


void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		lcd::set_text(2, "I was pressed!");
	} else {
		lcd::clear_line(2);
	}
}

void initialize() {
	lcd::initialize();
	lcd::set_text(1, "Salutations!");

	lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {} //imagine having auton

int mappedVoltage(int controllerVal) {
	// [-127, 127] -> [-12000, 12000]
	return std::min(controllerVal / 127.0 * 12000.0, 12000.0 / 100.0 * vconfig::maxVel);
}

void opcontrol() {
	while (true) {
		//drive
		dt.arcade(master, mappedVoltage);

		//intake
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intakeLow.move_velocity(vconfig::maxVel);
		} else {
			intakeLow.move_velocity(0);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intakeHigh.move_velocity(vconfig::maxVel);
		} else {
			intakeHigh.move_velocity(0);
		}

		delay(20);
	}
}