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

//pid gains
vractolib::PIDGains latGains = {1.0, 0.0, 0.8};
vractolib::PIDGains turnGains = {1.0, 0.0, 0.8};

//sensors
Rotation horizEnc(6);
Rotation vertEnc(7);
vractolib::TrackingWheel horiz(horizEnc, 3.25);
vractolib::TrackingWheel vert(vertEnc, 3.25);
std::vector<vractolib::TrackingWheel> horizWheels = {horiz};
std::vector<vractolib::TrackingWheel> vertWheels = {vert};
IMU imuSensor(20);
vractolib::IMU imu(imuSensor, 0.0);

//odom init
vractolib::OdomManager odom(horizWheels, vertWheels, imu);

//drivetrain init
vractolib::Drivetrain dt(driveleft, driveRight, latGains, turnGains, odom);

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

	dt.init();
	Task task{[] {
		while (true) {
			odom.update();
			delay(vconfig::updateRate);
		}
	}};
}

void disabled() {}

void competition_initialize() {}

void autonomous() {} //imagine having auton

void opcontrol() {
	while (true) {
		//drive
		dt.arcade(master);

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