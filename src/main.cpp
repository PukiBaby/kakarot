#include "main.h"

double exponential_drive(double input, double controllerDeadband = 3, double drivetrainDeadband = 10, double exponential_gain = 1.019)
{
    double maximumEffectiveInput = 127 - controllerDeadband; // Need to do this so that maximum input maps to 1
    // shift input back by the deadband and then scale it so that it goes from 0 to 1
    double sign;

    if (input > 0) 
    {
        sign = 1;
    }
    else
    {
        sign = -1;
    }

    double normalizedInput = (fabs(input) - controllerDeadband)/maximumEffectiveInput; // This formula only works for positive numbers

    if (fabs(input) < controllerDeadband)
    {
        return 0;
    }

    return sign*(drivetrainDeadband + (127 - drivetrainDeadband)*powf(normalizedInput, exponential_gain));
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({11, 2, 13});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-3, -14, -15});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(exponential_drive(dir - turn));                      // Sets left motor voltage
		right_mg.move(exponential_drive((dir + turn));                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}