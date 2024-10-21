#include "main.h"
#include "autons.hpp"
#include "pros/misc.h"
#include "subsystems.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  StratusQuo::chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  StratusQuo::chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  StratusQuo::chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Blue Ring AWP", sig_blue_ring_side),
    Auton("Red Ring AWP", sig_red_ring_side),
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  StratusQuo::chassis.initialize();
  ez::as::initialize();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
void competition_initialize() {
  // . . .
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
void autonomous() {
  StratusQuo::chassis.reset_pid_targets(); // Resets PID targets to 0
  StratusQuo::chassis.reset_gyro(); // Reset gyro position to 0
  StratusQuo::chassis.reset_drive_sensor(); // Reset drive sensors to 0
  StratusQuo::chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
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
  // This is preference to what you like to drive on.
  StratusQuo::chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  /*
  void (StratusQuo::Arm::*macro_func)() = &StratusQuo::Arm::macro_task;
  auto bound_function = std::bind(macro_func, &StratusQuo::arm);
  pros::Task macro(bound_function);*/ // For later!

  while (true) {

    StratusQuo::chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
      autonomous();

    if(StratusQuo::limit_switch.get_new_press())
    {
      StratusQuo::clamp.set_value(true);
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      StratusQuo::clamp.toggle();
      pros::delay(100);
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
      StratusQuo::arm.arm_up();
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
      StratusQuo::arm.arm_down();
    }
    else
    {
      StratusQuo::arm.brake();
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
      StratusQuo::intake.move(60);
      pros::delay(100);
      StratusQuo::arm.arm_up();
      pros::delay(500);
      StratusQuo::arm.toggle();
      StratusQuo::arm.arm_down();
      pros::delay(500);
      StratusQuo::arm.brake();
    }

    //I had to change the button to A because Ez-Temp was overriding button X in driver ctrl - Ansh :3
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
      StratusQuo::scooper.toggle();
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
      StratusQuo::intake.move(127);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
      StratusQuo::intake.move(-127);
    }
    else
    {
      StratusQuo::intake.brake();
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

void print()
{
  while(true)
  {
    std::cout << StratusQuo::chassis.get_gyro() << endl;
  }
}