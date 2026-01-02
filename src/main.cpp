#include "main.h"
#include <string>
using namespace pros;

//drive motors
MotorGroup driveRight({9, 10, 8}, v5::MotorGears::blue, v5::MotorUnits::degrees);
MotorGroup driveLeft({-11, -3, -15}, v5::MotorGears::blue, v5::MotorUnits::degrees);

//intake motors
Motor intakeLow(-12, v5::MotorGearset::blue, v5::MotorUnits::degrees);
Motor intakeHigh(-6, v5::MotorGearset::blue, v5::MotorUnits::degrees);

//pneumatics
adi::DigitalOut middle('a');
adi::DigitalOut arm('b');
adi::DigitalOut tongue('c');


//controller
Controller master(E_CONTROLLER_MASTER);

//pid gains
vractolib::PIDGains latGains = {6.0, 0.0, 1.0};
vractolib::PIDGains turnGains = {58.0, 0.0, 12.67};

//sensors
Optical optical(5);

Rotation horizEnc(17);
Rotation vertEnc(7);
vractolib::TrackingWheel horiz(horizEnc, 2, 1.0);
vractolib::TrackingWheel vert(vertEnc, 2, 0.5);
std::vector<vractolib::TrackingWheel> horizWheels = {horiz};
std::vector<vractolib::TrackingWheel> vertWheels = {vert};
IMU imuSensor(20);
vractolib::IMU imu(imuSensor, 0.0);

//odom init
vractolib::OdomManager odom(horizWheels, vertWheels, imu);

//drivetrain init
vractolib::Drivetrain dt(driveLeft, driveRight, latGains, turnGains, &odom);

// auton manager
vractolib::AutonManager autonManager;

bool isBlue = true;

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
	tongue.set_value(false);
	middle.set_value(true);
	arm.set_value(true);

	intakeHigh.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	autonManager.selectAuton(vractolib::AutonType::QUALS);

	Task task{[] {
		while (true) {
			odom.update();
			lcd::set_text(4, std::to_string(odom.getPose().x)+", "+std::to_string(odom.getPose().y)+", "+std::to_string(vunits::radToDeg(odom.getPose().theta)));
			delay(vconfig::updateRate);
		}
	}};
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	switch (autonManager.getAutonType()) {
		case vractolib::AutonType::NONE:
			return;
		case vractolib::AutonType::QUALS:
			//qual autons
			dt.move(29.5);
			tongue.set_value(true);
			dt.turnTo(90 * autonManager.sign());
			dt.move(8.5);
			intakeLow.move_voltage(vconfig::maxVolt);
			pros::delay(2000);

			//outtake long
			dt.move(-29.5);
			tongue.set_value(false);
			intakeLow.move_voltage(vconfig::maxVolt*0.7);
			intakeHigh.move_voltage(vconfig::maxVolt);
			pros::delay(1500);
			intakeHigh.move_voltage(0);
			intakeLow.move_voltage(0);
			dt.move(20);
			dt.turnTo(-135 * autonManager.sign());

			//mid low
			if (autonManager.getSide() == vractolib::Side::LEFT) {
				intakeLow.move_voltage(vconfig::maxVolt);
				dt.move(37, 4000, 150, 4500);
				pros::delay(500);
				dt.turnTo(-45);
				middle.set_value(false);
				dt.move(-20);
				pros::delay(3000);
				intakeLow.move_voltage(0);
			} else {
				intakeLow.move_voltage(vconfig::maxVolt);
				dt.move(57, 4000, 150, 4500);
				intakeLow.move_voltage(-vconfig::maxVolt*0.5);
				pros::delay(3000);
				intakeLow.move_voltage(0);
			}
			
			break;
		case vractolib::AutonType::SAWP:
			// solo awp - ONLY LEFT
			dt.move(30.75);
			dt.turnTo(-90);
			dt.move(8.5, 500);
			intakeLow.move_voltage(vconfig::maxVolt);
			pros::delay(500);

			//outtake long
			dt.move(-32, 1000);
			intakeLow.move_voltage(vconfig::maxVolt*0.7);
			intakeHigh.move_voltage(vconfig::maxVolt);
			pros::delay(500);
			intakeHigh.move_voltage(0);
			intakeLow.move_voltage(0);
			dt.move(7, 500);
			dt.turnTo(142.5, 750);

			//turn to mid
			intakeLow.move_voltage(vconfig::maxVolt);
			dt.move(23, 1000, 150, 7000);
			dt.turnTo(-180, 750);
			dt.move(52, 1500, 150, 7000);
			intakeLow.move_voltage(0);
			dt.turnTo(45);
			dt.move(20, 750, 150, 7000);
			intakeLow.move_voltage(-vconfig::maxVolt*0.5);
			pros::delay(500);
			intakeLow.move_voltage(0);
			dt.move(-39, 1000);
			dt.turnTo(-90, 500);
			dt.move(-22, 750);
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(vconfig::maxVolt);
			break;
		case vractolib::AutonType::ELIMS:
			//elims auton
			intakeLow.move_voltage(vconfig::maxVolt);
			dt.moveToPoint(vunits::Pose{-12.0, 30.0, vunits::degToRad(0.0)}, 3000, 150, 4500);
			dt.turnTo(-142);
			dt.move(33);

			//matchload
			tongue.set_value(true);
			dt.turnTo(-180); //90
			intakeLow.move_voltage(vconfig::maxVolt);
			dt.move(12.5);
			pros::delay(750);

			//outtake long
			dt.move(-28.5);
			tongue.set_value(false);
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(vconfig::maxVolt);
			pros::delay(1500);
			intakeHigh.move_voltage(0);
			intakeLow.move_voltage(0);

			//push in
			dt.move(8);
			arm.set_value(false);
			dt.turnTo(-123.69); //-135
			dt.move(-11.25);
			dt.turnTo(-180);
			dt.move(-2);
			arm.set_value(true);
			dt.move(-17, 3000, 150, 4500);
			break;
		case vractolib::AutonType::SKILLS:
			//skills auton code
			break;
	}
}

int spinDir = 0;

//[0]=tongue, [1]=middle, [2]=arm
bool wasPressed[3] = {false, false, false};
bool pistonStates[3] = {false, true, true};

int curTick = 0;
int nextDetectTick = 0;
int holdDir = 0;

void opcontrol() {
	//TODO: move optical system to always running loop in init

	pros::delay(3000);
	
	autonomous();

	while (true) {
		//drive
		dt.arcadeDoubleStick(master);

		//bottom 2 intake rollers
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intakeLow.move_voltage(vconfig::maxVolt);
			if (!pistonStates[1]) {
				pistonStates[1] = true;
				middle.set_value(pistonStates[1]);
			}
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			//outtake toplow
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(-vconfig::maxVolt);
			if (pistonStates[1]) {
				pistonStates[1] = false;
				middle.set_value(pistonStates[1]);
			}
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			//outtake bottom
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(vconfig::maxVolt);
			if (!pistonStates[1]) {
				pistonStates[1] = true;
				middle.set_value(pistonStates[1]);
			}
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intakeLow.move_voltage(-vconfig::maxVolt);
			if (!pistonStates[1]) {
				pistonStates[1] = true;
				middle.set_value(pistonStates[1]);
			}
		} else {
			intakeLow.move_voltage(0);
			intakeHigh.move_voltage(0);
			if (!pistonStates[1]) {
				pistonStates[1] = true;
				middle.set_value(pistonStates[1]);
			}
		}

		// if (curTick == nextDetectTick) {
		// 	if (isBlue) {
		// 		if (true){//optical.get_hue() >= 40) {
		// 			lcd::set_text(3, std::to_string(optical.get_hue())+" good");
		// 			intakeHigh.move_voltage(-vconfig::maxVolt * spinDir);
		// 			holdDir = -vconfig::maxVolt * spinDir;
		// 			nextDetectTick+=5;
		// 		} else if (optical.get_hue() <= 20) {
		// 			lcd::set_text(3, std::to_string(optical.get_hue())+" wrong");
		// 			intakeHigh.move_voltage(vconfig::maxVolt * spinDir);
		// 			holdDir = vconfig::maxVolt * spinDir;
		// 			nextDetectTick+=15;
		// 		} else {
		// 			lcd::set_text(3, std::to_string(optical.get_hue())+" waiting");
		// 			intakeHigh.move_voltage(0);
		// 			nextDetectTick++;
		// 		}
		// 	} else {
		// 		if (true) {//optical.get_hue() <= 20) {
		// 			lcd::set_text(3, std::to_string(optical.get_hue())+" good");
		// 			intakeHigh.move_voltage(vconfig::maxVolt * spinDir);
		// 			holdDir = vconfig::maxVolt * spinDir;
		// 			nextDetectTick+=15;
		// 		} else if (optical.get_hue() >= 40) {
		// 			lcd::set_text(3, std::to_string(optical.get_hue())+" wrong");
		// 			intakeHigh.move_voltage(-vconfig::maxVolt * spinDir);
		// 			holdDir = -vconfig::maxVolt * spinDir;
		// 			nextDetectTick+=15;
		// 		} else {
		// 			lcd::set_text(3, std::to_string(optical.get_hue())+" waiting");
		// 			intakeHigh.move_voltage(0);
		// 			nextDetectTick++;
		// 		}
		// 	}
		// } else {
		// 	lcd::set_text(3, std::to_string(optical.get_hue())+" holding");
		// 	intakeHigh.move_voltage(holdDir);
		// 	// intakeMid.move_voltage(0);
		// }

		//TODO: piston class
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if (!wasPressed[0]) {
				pistonStates[0] = !pistonStates[0];
				tongue.set_value(pistonStates[0]);
				wasPressed[0] = true;
			}
		} else {
			wasPressed[0] = false;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			if (!wasPressed[2]) {
				pistonStates[2] = !pistonStates[2];
				arm.set_value(pistonStates[2]);
				wasPressed[2] = true;
			}
		} else {
			wasPressed[2] = false;
		}


		curTick++;
		delay(20);
	}
}