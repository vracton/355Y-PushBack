#include "main.h"
#include "../include/liblvgl/lvgl.h"
#include "./comic_sans.c"
#include <string>
using namespace pros;

//drive motors
MotorGroup driveRight({9, 10, 8}, v5::MotorGears::blue, v5::MotorUnits::degrees);
MotorGroup driveLeft({-16, -3, -15}, v5::MotorGears::blue, v5::MotorUnits::degrees);

//intake motors
Motor intakeLow(-12, v5::MotorGearset::blue, v5::MotorUnits::degrees);
Motor intakeHigh(-6, v5::MotorGearset::blue, v5::MotorUnits::degrees);

//pneumatics
adi::DigitalOut middleRaw('a');
adi::DigitalOut armRaw('b');
adi::DigitalOut tongueRaw('c');
vractolib::Solenoid middle(middleRaw);
vractolib::Solenoid arm(armRaw);
vractolib::Solenoid tongue(tongueRaw);

//controller
Controller master(E_CONTROLLER_MASTER);

//pid gains
vractolib::PIDGains latGains = {6.0, 0.0, 1.0};
vractolib::PIDGains turnGains = {58.0, 0.0, 12.1};

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
		// lcd::set_text(2, isBlue ? "Blue Alliance" : "Red Alliance");
		pros::delay(500);
	}
}

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = (lv_obj_t *) lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        int cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
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
	tongue.set(false);
	middle.set(true);
	arm.set(false);

	intakeHigh.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	autonManager.selectAuton(vractolib::AutonType::ELIMS);
	autonManager.setSide(vractolib::Side::RIGHT);

	// lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);
	// lv_font_t *my_font = lv_binfont_create("./comic_sans.bin");
    /*Create a white label, set its text and align it to the center*/
    // lv_obj_t * label = lv_label_create(lv_screen_active());
    // lv_label_set_text(label, "hi");
	// lv_obj_set_style_text_font(label, &comic_sans, LV_PART_MAIN);
    // lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);
    // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

	// lv_obj_t * btn = lv_button_create(lv_screen_active());     /*Add a button the current screen*/
    // lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    // lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    // lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    // lv_obj_t * label2 = lv_label_create(btn);          /*Add a label to the button*/
    // lv_label_set_text(label2, "Button");                     /*Set the labels text*/
    // lv_obj_center(label2);

	/* start commenting */
	// static int32_t col_dsc[] = {70, 70, 70, LV_GRID_TEMPLATE_LAST};
    // static int32_t row_dsc[] = {50, 50, LV_GRID_TEMPLATE_LAST};

    // /*Create a container with grid*/
    // lv_obj_t * cont = lv_obj_create(lv_screen_active());
    // lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
    // lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
    // lv_obj_set_size(cont, 300, 220);
    // lv_obj_center(cont);
    // lv_obj_set_layout(cont, LV_LAYOUT_GRID);

    // lv_obj_t * label;
    // lv_obj_t * obj;

    // uint32_t i;
    // for(i = 0; i < 6; i++) {
    //     uint8_t col = i % 3;
    //     uint8_t row = i / 3;

    //     obj = lv_button_create(cont);
    //     /*Stretch the cell horizontally and vertically too
    //      *Set span to 1 to make the cell 1 column/row sized*/
    //     lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_STRETCH, col, 1, LV_GRID_ALIGN_STRETCH, row, 1);

    //     label = lv_label_create(obj);

	// 	if (i == 0) {
	// 		lv_label_set_text_fmt(label, "LEFT", col, row);
	// 	} else {
	// 		switch (i) {
	// 			case 1:
	// 				lv_label_set_text_fmt(label, "NONE", col, row);
	// 				break;
	// 			case 2:
	// 				lv_label_set_text_fmt(label, "QUALS", col, row);
	// 				break;
	// 			case 3:
	// 				lv_label_set_text_fmt(label, "SOLO AWP", col, row);
	// 				break;
	// 			case 4:
	// 				lv_label_set_text_fmt(label, "ELIMS", col, row);
	// 				break;
	// 			case 5:
	// 				lv_label_set_text_fmt(label, "SKILLS", col, row);
	// 				break;
	// 			default:
	// 				lv_label_set_text_fmt(label, ":p", i, col, row);
	// 				break;
	// 		}
	// 	}

	// 	lv_obj_center(label);

	// 	lv_obj_add_event_cb(obj, [](lv_event_t * e) {
	// 		lv_event_code_t code = lv_event_get_code(e);
	// 		lv_obj_t * btn = (lv_obj_t *) lv_event_get_target(e);
	// 		if(code == LV_EVENT_CLICKED) {
	// 			/*Get the first child of the button which is the label and change its text*/
	// 			autonManager.toggleSide();

	// 			lv_obj_t * label = lv_obj_get_child(btn, 0);
	// 			lv_label_set_text_fmt(label, "%s", autonManager.getSide() == vractolib::Side::LEFT ? "LEFT" : "RIGHT");
	// 		}
	// 	}, LV_EVENT_ALL, NULL);
    // }

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
			tongue.set(true);
			dt.turnTo(-90 * autonManager.sign());
			intakeLow.move_voltage(vconfig::maxVolt);
			dt.move(9.5, 1000);
			pros::delay(500);

			//outtake long
			dt.move(-29.5, 1000);
			tongue.set(false);
			intakeLow.move_voltage(vconfig::maxVolt*0.7);
			intakeHigh.move_voltage(vconfig::maxVolt);
			pros::delay(1500);
			intakeHigh.move_voltage(0);
			intakeLow.move_voltage(0);
			dt.move(20);
			dt.turnTo(135 * autonManager.sign());

			//mid low
			// if (autonManager.getSide() == vractolib::Side::LEFT) {
			// 	intakeLow.move_voltage(vconfig::maxVolt);
			// 	dt.move(37, 1500, 150, 4500);
			// 	pros::delay(500);
			// 	dt.turnTo(-45);
			// 	middle.set_value(false);
			// 	dt.move(-20, 1000);
			// 	intakeHigh.move_voltage(vconfig::maxVolt);
			// 	pros::delay(1000);
			// 	dt.move(3);
			// 	intakeLow.move_voltage(0);
			// } else {
			// 	intakeLow.move_voltage(vconfig::maxVolt);
			// 	dt.move(57, 4000, 150, 4500);
			// 	intakeLow.move_voltage(-vconfig::maxVolt*0.5);
			// 	pros::delay(3000);
			// 	intakeLow.move_voltage(0);
			// }
			
			break;
		case vractolib::AutonType::SAWP:
			// solo awp - ONLY LEFT
			dt.move(30.75);
			dt.turnTo(-90);
			dt.move(9.5, 500);
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
			dt.moveToPoint(vunits::Pose{-11.0 * autonManager.sign(), 30.0, vunits::degToRad(0.0)}, false, 3000, 150, 4500);
			dt.turnTo(-142 * autonManager.sign(), 1250);
			dt.move(32.75, 1250);

			//matchload
			tongue.set(true);
			dt.turnTo(-180, 1250); //90
			intakeLow.move_voltage(vconfig::maxVolt);
			dt.move(15.5, 900);
			pros::delay(500);

			//outtake long
			dt.turnTo(-180, 250); //90
			dt.move(-29.5, 1250);
			tongue.set(false);
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(vconfig::maxVolt);
			pros::delay(1750);
			intakeHigh.move_voltage(0);
			intakeLow.move_voltage(0);

			//push in
			dt.move(8.75, 850);
			dt.turnTo(-123.69, 800); //-135
			dt.move(-11.5, 750);
			dt.turnTo(-180, 750);
			arm.set(true);
			dt.move(-6, 500);
			arm.set(false);
			dt.move(-13, 1000, 150, 4500);
			dt.move(8, 1000, 150, 6000);
			break;
		case vractolib::AutonType::SKILLS:
			intakeHigh.move_voltage(vconfig::maxVolt * 0.3);
			// odom.setPose(vunits::Pose{-16.935, -47.94, vunits::degToRad(0.0)});
			// intakeLow.move_voltage(vconfig::maxVolt);
			// dt.moveToPoint(vunits::Pose{-24, -24, vunits::degToRad(0.0)}, false, 3000, 1500, 4500);
			// dt.pointTowards(vunits::Pose{-100.0, -100.0, vunits::degToRad(0.0)});
			// intakeLow.move_voltage(0);
			// middle.set(false);
			// dt.move(-21.21);
			// // dt.moveToPoint(vunits::Pose{-9, -9, vunits::degToRad(0.0)}, true, 3000, 1500, 4500);
			// intakeLow.move_voltage(-vconfig::maxVolt);
			// pros::delay(250);
			// intakeLow.move_voltage(vconfig::maxVolt);
			// pros::delay(500);
			// dt.moveToPoint(vunits::Pose{-48, -50, vunits::degToRad(0.0)}, false, 3000, 1500, 4500);
			// tongue.set(true);
			// dt.pointTowards(vunits::Pose{-48, -72, vunits::degToRad(0.0)});
			// dt.moveToPoint(vunits::Pose{-48, -57, vunits::degToRad(0.0)}, false, 3000, 1500, 4500);
			break;
	}
}

int spinDir = 0;

//[0]=tongue, [1]=middle, [2]=arm
// bool wasPressed[3] = {false, false, false};
// bool pistonStates[3] = {false, true, true};

int curTick = 0;
int nextDetectTick = 0;
int holdDir = 0;

void opcontrol() {
	//TODO: move optical system to always running loop in init

	// pros::delay(3000);
	
	// autonomous();

	while (true) {
		//drive
		dt.arcadeDoubleStick(master);

		//bottom 2 intake rollers
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			// intake
			intakeLow.move_voltage(vconfig::maxVolt);
			middle.set(true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			// outtake middle
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(-vconfig::maxVolt);
			middle.set(false);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			// outtake top
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(vconfig::maxVolt);
			middle.set(true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			// outtake low
			intakeLow.move_voltage(-vconfig::maxVolt);
			middle.set(true);
		} else {
			intakeLow.move_voltage(0);
			intakeHigh.move_voltage(0);
			middle.set(true);
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

		tongue.toggleOn(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1));
		arm.toggleOn(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y));

		curTick++;
		delay(20);
	}
}