#include "autons.hpp"
#include "main.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

using namespace StratusQuo;

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(20, 0, 100); // Driving :D
  chassis.pid_turn_constants_set(3, 0.05, 20, 15); // Turning :D
  chassis.pid_swing_constants_set(6, 0, 65); // Swing :D

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .

void StratusQuo::sig_red_ring_side()
{

};
void StratusQuo::sig_red_goal_side()
{

};
void StratusQuo::sig_blue_ring_side()
{
  StratusQuo::chassis.pid_drive_set(-40, 90);
  StratusQuo::chassis.pid_wait();
  StratusQuo::clamp.set_value(true);
  StratusQuo::chassis.pid_turn_relative_set(-90, TURN_SPEED);
  /*
  StratusQuo::arm.arm_move(110);
  pros::delay(1250);
  StratusQuo::arm.arm_move(-110);
  pros::delay(1250);
  StratusQuo::arm.brake();

  StratusQuo::chassis.pid_drive_set(-50, 75);
  StratusQuo::chassis.pid_wait();
  StratusQuo::clamp.set_value(true);
  pros::delay(100);
  StratusQuo::chassis.pid_turn_set(-105, TURN_SPEED);
  StratusQuo::chassis.pid_wait();
  StratusQuo::intake.move(127);
  StratusQuo::chassis.pid_drive_set(10, DRIVE_SPEED);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_turn_set(-195, TURN_SPEED);
  StratusQuo::chassis.pid_wait();
  
  StratusQuo::chassis.pid_turn_set(105, TURN_SPEED);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_drive_set(15, DRIVE_SPEED);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_drive_set(-10, DRIVE_SPEED);
  StratusQuo::chassis.pid_wait();*/
  /*
  StratusQuo::arm.arm_move(110);
  pros::delay(1250);
  StratusQuo::arm.arm_move(-110);
  pros::delay(1250);
  StratusQuo::arm.brake();
  StratusQuo::chassis.pid_drive_set(-50000000, 90);
  StratusQuo::chassis.pid_wait();
  StratusQuo::clamp.set_value(true);
  StratusQuo::chassis.pid_turn_set(-160, 127);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_drive_set(12, 127);
  StratusQuo::intake.move(127);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_turn_set(1, 127);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_drive_set(50, 127);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_turn_set(10, 127);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_drive_set(100, 127);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_turn_set(120, 127);
  StratusQuo::chassis.pid_wait();
  StratusQuo::chassis.pid_drive_set(100, 127); // Go to Ladder, might not have time!
  */
};
void StratusQuo::sig_blue_goal_side()
{

};

void StratusQuo::local_red_ring_side()
{

};
void StratusQuo::local_red_goal_side()
{

};
void StratusQuo::local_blue_ring_side()
{
  StratusQuo::chassis.pid_drive_set(-30, 127);
  
};
void StratusQuo::local_blue_goal_side()
{

};