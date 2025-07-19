#include "main.h"
#include "vractolib.h"
using namespace pros;

//drive motors
MotorGroup driveRight({11, 12, -3}, v5::MotorGears::blue, v5::MotorUnits::degrees);
MotorGroup driveleft({-14, -15, 16}, v5::MotorGears::blue, v5::MotorUnits::degrees);

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
	return controllerVal / 127.0 * 12000.0;
}

void opcontrol() {
	while (true) {
		//drivebase control
		dt.arcade(master, mappedVoltage);

		delay(20);
	}
}