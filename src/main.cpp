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
pros::adi::DigitalOut holdDescore('b');
pros::adi::DigitalOut tongue('a');


//controller
Controller master(E_CONTROLLER_MASTER);

//pid gains
vractolib::PIDGains latGains = {6.0, 0.0, 1.0};
vractolib::PIDGains turnGains = {60.0, 0.0, 9.7};

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
	holdDescore.set_value(false);
	tongue.set_value(true);
	holdDescore.set_value(false);
	tongue.set_value(true);

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
	// //move perp to goal
	// holdDescore.set_value(true);
	// intakeHigh.move_voltage(-vconfig::maxVolt*0.5);
	// intakeLow.move_voltage(vconfig::maxVolt);
	// intakeMid.move_voltage(vconfig::maxVolt);

	// dt.move(-30, 1500, 250, 12000*0.3);//-38.274
	// delay(250);
	// intakeMid.move_voltage(0);
	// dt.turnTo(-128, 1000, 250, 600*0.3);
	// delay(1000);
	// dt.move(17, 1000, 250, 12000*0.2);
	// intakeLow.move_voltage(12000);
	// intakeMid.move_voltage(12000);
	// intakeHigh.move_voltage(12000);
	// delay(2000);
	// // intakeLow.move_voltage(0);
	// intakeMid.move_voltage(0);
	// //go back to long goal
	// dt.move(-52, 1500, 250, 12000*0.6);//50.4589031905
	// delay(250);
	// dt.turnTo(-180, 750, 250, 600*0.3);
	// tongue.set_value(false);
	// delay(1000);
	// intakeLow.move_voltage(12000);//vconfig::maxVolt*0.7);
	// intakeMid.move_voltage(12000);//vconfig::maxVolt*0.7);
	// dt.move(-13.5,250, 12000*0.6);//-33.189

	// //at mathcload
	// delay(1500);
	// intakeLow.move_voltage(0);
	// intakeMid.move_voltage(0);
	// dt.move(10,500,350, 12000*0.4);
	// dt.turnTo(-180, 250, 250, 600*0.3);
	// dt.move(25,1500,350, 12000*0.4);
	// holdDescore.set_value(false);
	// intakeLow.move_voltage(12000);
	// intakeMid.move_voltage(12000);
	// intakeHigh.move_voltage(-12000);
	// delay(2000);
	// intakeMid.move_voltage(0);
	// intakeHigh.move_voltage(0);
}

int spinDir = 0;

//[0]=tongue, [1]=holdDescore
bool wasPressed[2] = {false, false};
bool pistonStates[2] = {false, false};

int curTick = 0;
int nextDetectTick = 0;
int holdDir = 0;

void opcontrol() {
	//move optical system to always running loop in init

	// pros::delay(3000);
	// dt.turnTo(-30);

	while (true) {
		//drive
		dt.arcadeDoubleStick(master);

		//bottom 2 intake rollers
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intakeLow.move_voltage(vconfig::maxVolt);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			//outtake toplow
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(-vconfig::maxVolt);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			//outtake bottom
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(vconfig::maxVolt);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intakeLow.move_voltage(-vconfig::maxVolt);
		} else {
			intakeLow.move_voltage(0);
			intakeHigh.move_voltage(0);
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
	
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			if (!wasPressed[0]) {
				pistonStates[0] = !pistonStates[0];
				tongue.set_value(pistonStates[0]);
				wasPressed[0] = true;
			}
		} else {
			wasPressed[0] = false;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			if (!wasPressed[1]) {
				pistonStates[1] = !pistonStates[1];
				holdDescore.set_value(pistonStates[1]);
				wasPressed[1] = true;
			}
		} else {
			wasPressed[1] = false;
		}


		curTick++;
		delay(20);
	}
}