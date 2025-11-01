#include "main.h"
#include <string>
using namespace pros;

//drive motors
MotorGroup driveRight({-18, -17, -16}, v5::MotorGears::blue, v5::MotorUnits::degrees);
MotorGroup driveLeft({-20, -19, -15}, v5::MotorGears::blue, v5::MotorUnits::degrees);

//intake motors
Motor intakeHigh(1, v5::MotorGearset::green, v5::MotorUnits::degrees);
Motor intakeMid(-2, v5::MotorGearset::green, v5::MotorUnits::degrees);
Motor intakeLow(21, v5::MotorGearset::blue, v5::MotorUnits::degrees);

//controller
Controller master(E_CONTROLLER_MASTER);

//pid gains
vractolib::PIDGains latGains = {1.0, 0.0, 0.8};
vractolib::PIDGains turnGains = {1.0, 0.0, 0.8};

//sensors
Optical optical(3);

// Rotation horizEnc(6);
Rotation vertEnc(7);
// vractolib::TrackingWheel horiz(horizEnc, 3.25);
vractolib::TrackingWheel vert(vertEnc, 2, 0.5);
std::vector<vractolib::TrackingWheel> horizWheels = {};
std::vector<vractolib::TrackingWheel> vertWheels = {vert};
IMU imuSensor(4);
vractolib::IMU imu(imuSensor, 0.0);

//odom init
vractolib::OdomManager odom(horizWheels, vertWheels, imu);

//drivetrain init
vractolib::Drivetrain dt(driveLeft, driveRight, latGains, turnGains, odom);

// vars
bool isBlue = false;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		isBlue = !isBlue;
		lcd::set_text(2, isBlue ? "Blue Alliance" : "Red Alliance");
		pros::delay(500);
	}
}

void initialize() {
	lcd::initialize();
	lcd::set_text(1, "Salutations!");

	lcd::set_text(2, isBlue ? "Blue Alliance" : "Red Alliance");
	lcd::set_text(3, "color sort subsytem not running");
	lcd::register_btn1_cb(on_center_button);

	dt.init();
	dt.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

	Task task{[] {
		while (true) {
			odom.update();
			delay(vconfig::updateRate);
		}
	}};
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	dt.move(5);
}

int spinDir = 0;

void opcontrol() {
	pros::lcd::set_text(1, "Hello Driver!");
	while (true) {
		//drive
		dt.arcadeDoubleStick(master);

		//bottom 2 intake rollers
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			//outtake tophigh
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeMid.move_voltage(vconfig::maxVolt);
			spinDir = 1;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			//outtake toplow
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeMid.move_voltage(vconfig::maxVolt);
			spinDir = -1;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) || master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			//outtake bottom
			intakeLow.move_voltage(-vconfig::maxVolt);
			intakeMid.move_voltage(-vconfig::maxVolt);
			spinDir = 0;
		} else {
			intakeLow.move_voltage(0);
			intakeMid.move_voltage(0);
			spinDir = 0;
		}

		if ((optical.get_hue() >= 120 && isBlue) || (optical.get_hue() <= 18 && !isBlue)) {
			intakeHigh.move_voltage(-vconfig::maxVolt * spinDir);
		} else {
			intakeHigh.move_voltage(vconfig::maxVolt * spinDir);
		}
	
		lcd::set_text(3, std::to_string(optical.get_hue()));
		lcd::set_text(4, std::to_string(odom.getPose().x)+", "+std::to_string(odom.getPose().y)+", "+std::to_string(vunits::radToDeg(odom.getPose().theta)));

		delay(20);
	}
}