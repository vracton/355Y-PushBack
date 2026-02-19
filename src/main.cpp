#include "main.h"
#include "../include/liblvgl/lvgl.h"
#include "./comic_sans.c"
#include <cstdio>
#include <string>
using namespace pros;

//drive motors
MotorGroup driveRight({8, 10, 9}, v5::MotorGears::blue, v5::MotorUnits::degrees);
MotorGroup driveLeft({-15, -3, -16}, v5::MotorGears::blue, v5::MotorUnits::degrees);

//intake motors
Motor intakeLow(12, v5::MotorGearset::blue, v5::MotorUnits::degrees);
Motor intakeHigh(-6, v5::MotorGearset::blue, v5::MotorUnits::degrees);

//pneumatics
adi::DigitalOut intakeSolRaw('a');
adi::DigitalOut armRaw('b');
adi::DigitalOut tongueRaw('c');
adi::DigitalOut outtakeDividerRaw('d');
vractolib::Solenoid intakeSol(intakeSolRaw, true);
vractolib::Solenoid arm(armRaw, true);
vractolib::Solenoid tongue(tongueRaw, true);
vractolib::Solenoid outtakeDivider(outtakeDividerRaw, false);

//controller
Controller master(E_CONTROLLER_MASTER);

//pid gains
vractolib::PIDGains latGains = {27.0, 0.0, 1.4};
vractolib::PIDGains turnGains = {92.0, 0.0, 8.4};

Rotation horizEnc(7);
Rotation vertEnc(20);
vractolib::TrackingWheel horiz(horizEnc, 2, 5.5, true);
vractolib::TrackingWheel vert(vertEnc, 2, 0.25, true);
std::vector<vractolib::TrackingWheel> horizWheels = {horiz};
std::vector<vractolib::TrackingWheel> vertWheels = {vert};
IMU imuSensor(2);
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

	intakeHigh.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	autonManager.selectAuton(vractolib::AutonType::NONE);
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

bool inComp = false;

void competition_initialize() {
	inComp = true;
}

void autonomous() {
	switch (autonManager.getAutonType()) {
		case vractolib::AutonType::NONE:
			return;
		case vractolib::AutonType::QUALS:
			//qual autons
			dt.move(29.5);
			tongue.enable();
			dt.turnTo(-90 * autonManager.sign());
			intakeLow.move_voltage(vconfig::maxVolt);
			dt.move(9.5, 1000);
			pros::delay(500);

			//outtake long
			dt.move(-29.5, 1000);
			tongue.disable();
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
			tongue.enable();
			dt.turnTo(-180, 1250); //90
			intakeLow.move_voltage(vconfig::maxVolt);
			dt.move(15.5, 900);
			pros::delay(500);

			//outtake long
			dt.turnTo(-180, 250); //90
			dt.move(-29.5, 1250);
			tongue.disable();
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
			arm.enable();
			dt.move(-6, 500);
			arm.disable();
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
			// middle.disable();
			// dt.move(-21.21);
			// // dt.moveToPoint(vunits::Pose{-9, -9, vunits::degToRad(0.0)}, true, 3000, 1500, 4500);
			// intakeLow.move_voltage(-vconfig::maxVolt);
			// pros::delay(250);
			// intakeLow.move_voltage(vconfig::maxVolt);
			// pros::delay(500);
			// dt.moveToPoint(vunits::Pose{-48, -50, vunits::degToRad(0.0)}, false, 3000, 1500, 4500);
			// tongue.enable();
			// dt.pointTowards(vunits::Pose{-48, -72, vunits::degToRad(0.0)});
			// dt.moveToPoint(vunits::Pose{-48, -57, vunits::degToRad(0.0)}, false, 3000, 1500, 4500);
			break;
	}
}

void opcontrol() {

	if (!inComp) {
		pros::delay(3000);
	}

	int selectedTurnGain = 0; // 0 = P, 1 = I, 2 = D
	const char* gainNames[3] = {"P", "I", "D"};
	char gainLine[64];

	while (true) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			selectedTurnGain = (selectedTurnGain + 2) % 3;
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			selectedTurnGain = (selectedTurnGain + 1) % 3;
		}

		bool changedTurnGains = false;
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			if (selectedTurnGain == 0) latGains.kP += 1.0;
			else if (selectedTurnGain == 1) latGains.kI += 1.0;
			else latGains.kD += 0.1;
			changedTurnGains = true;
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			if (selectedTurnGain == 0) latGains.kP -= 1.0;
			else if (selectedTurnGain == 1) latGains.kI -= 1.0;
			else latGains.kD -= 0.1;
			changedTurnGains = true;
		}
		if (changedTurnGains) {
			dt.setLatGains(latGains);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			// const double currentHeadingDeg = vunits::radToDeg(odom.getPose().theta);
			// dt.turnTo(vunits::wrapToSignedDegrees(currentHeadingDeg + 60.0));
			// master.print(0, 0, "delta: %.2f", vunits::radToDeg(odom.getPose().theta - vunits::degToRad(currentHeadingDeg)));
			vunits::Pose curPos = odom.getPose();
			dt.move(10.0);
			master.print(0, 0, "delta: %.2f", vunits::Pose::distance(curPos, odom.getPose()));
		}

		std::snprintf(gainLine, sizeof(gainLine), "Sel:%s P:%.1f I:%.1f D:%.1f", gainNames[selectedTurnGain], latGains.kP, latGains.kI, latGains.kD);
		lcd::set_text(5, gainLine);

		//drive
		dt.arcadeDoubleStick(master);

		//bottom 2 intake rollers
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			// intake
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeSol.disable();
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			// outtake middle
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(vconfig::maxVolt);
			outtakeDivider.disable();
			intakeSol.disable();
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			// outtake top
			intakeLow.move_voltage(vconfig::maxVolt);
			intakeHigh.move_voltage(vconfig::maxVolt);
			outtakeDivider.enable();
			intakeSol.disable();
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			// outtake low
			intakeLow.move_voltage(-vconfig::maxVolt);
			intakeSol.enable();
		} else {
			intakeLow.move_voltage(0);
			intakeHigh.move_voltage(0);
		}

		tongue.toggleOn(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1));
		arm.toggleOn(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y));

		delay(20);
	}
}
