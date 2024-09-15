#include "liblvgl/llemu.h"
#include "liblvgl/llemu.hpp"
#include "pros.h"
#include "drivetrain.h"
#include "clamp.h"
#include "pros/misc.h"
#include "objects.h"
#include "autons.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	StratusQuo::dt.initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	StratusQuo::arm.stop();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
	/*TODO: Make auton selector :D */

	pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
	
	while(true) 
	{
		if(pros::lcd::read_buttons() == LCD_BTN_LEFT)
		{
			StratusQuo::auton = StratusQuo::SKILLS;
		}
		else if(pros::lcd::read_buttons() == LCD_BTN_CENTER)
		{
			StratusQuo::auton = StratusQuo::MATCH;
		}
		else if(pros::lcd::read_buttons() == LCD_BTN_RIGHT)
		{
			StratusQuo::auton = StratusQuo::GOAL_RUSH;
		}
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous()
{
	switch (StratusQuo::auton)
	{
		case StratusQuo::SKILLS:
			// do skills
			StratusQuo::autonomous.skills();
		case StratusQuo::MATCH:
			// do match
			StratusQuo::autonomous.match();
		case StratusQuo::GOAL_RUSH:
			// do goal rush
			StratusQuo::autonomous.goal_rush();
		default:
			return;
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		StratusQuo::dt.drive();

		if(StratusQuo::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			StratusQuo::arm.arm_up();
		}
		else if(StratusQuo::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			StratusQuo::arm.arm_down();
		}
		else 
		{
			StratusQuo::arm.stop();
		}

		if(StratusQuo::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			StratusQuo::in.intake();
		}
		else if(StratusQuo::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			StratusQuo::in.outtake();
		}
		else
		{
			StratusQuo::in.stop();
		}

		if(StratusQuo::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			StratusQuo::piston.set_value(false);
			pros::delay(20);
		}
		if(StratusQuo::limit_switch.get_new_press())
		{
			StratusQuo::piston.set_value(true);
		}
		
		pros::delay(20);                               // Run for 20 ms then update
	}
}