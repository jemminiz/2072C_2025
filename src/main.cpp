#include "main.h"
#include "autons.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.h"
#include "subsystems.hpp"

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
void initialize() {
	pros::lcd::initialize();
	/*while (true) { // infinite loop
        // print measurements from the rotation sensor
		pros::lcd::print(0, "Horizontal Rotation Sensor: %i", StratusQuo::horizontal.get_position());
        pros::lcd::print(1, "Vertical Rotation Sensor: %i", StratusQuo::vertical.get_position());
        pros::delay(10); // delay to save resources. DO NOT REMOVE
	}*/ // Commented out for testing purposes - want to see if task works properly
	StratusQuo::robot.chassis.calibrate();
	pros::Task ls_task (StratusQuo::limit_switch_task, (static_cast<void*>(&StratusQuo::robot)), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Limit switch task");
	pros::Task arm_task (StratusQuo::arm_task_fn, (static_cast<void*>(&StratusQuo::robot.arm)), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm task");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
	StratusQuo::auton_selector.focus();
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
	StratusQuo::auton_selector.run_auton();
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
	pros::Task drive_task(StratusQuo::drive_task_fn, ((void*)&StratusQuo::robot), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	pros::Task arm_task(StratusQuo::arm_task_fn, ((void*)&StratusQuo::robot), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	pros::Task intake_task(StratusQuo::intake_task_fn, ((void*)&StratusQuo::robot), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	pros::Task scooper_task(StratusQuo::scooper_task_fn, ((void*)&StratusQuo::robot), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	pros::Task clamp_task(StratusQuo::clamp_task_fn, ((void*)&StratusQuo::robot), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
		/*
		StratusQuo::robot.chassis.tank(StratusQuo::robot.master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), StratusQuo::robot.master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		if(StratusQuo::robot.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			StratusQuo::robot.clamp.toggle();
		}
		if(StratusQuo::robot.master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    	{
      		StratusQuo::robot.arm.up();
    	}
		else if(StratusQuo::robot.master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    	{
      		StratusQuo::robot.arm.down();
    	}
		else
    	{
      		StratusQuo::robot.arm.brake();
    	}

		if(StratusQuo::robot.master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    	{
      		StratusQuo::robot.intake.move(127);
    	}
		else if(StratusQuo::robot.master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    	{
      		StratusQuo::robot.intake.move(-127);
    	}
		else
    	{
      		StratusQuo::robot.intake.brake();
    	}

		if(StratusQuo::robot.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    	{
      		StratusQuo::robot.scooper.toggle();
    	}
		pros::delay(25);                               // Run for 20 ms then update */ // Commented everything out to test tasks.
	}
}