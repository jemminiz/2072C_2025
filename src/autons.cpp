

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


#include "autons.hpp"
#include "subsystems.hpp"
const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;



///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  StratusQuo::chassis.set_slew_min_power(80, 80);
  StratusQuo::chassis.set_slew_distance(7, 7);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.headingPID, 11, 0, 20, 0);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.forward_drivePID, 0.45, 0, 5, 0);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.backward_drivePID, 0.45, 0, 5, 0);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.turnPID, 5, 0.003, 35, 15);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants() {
  StratusQuo::chassis.set_slew_min_power(80, 80);
  StratusQuo::chassis.set_slew_distance(7, 7);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.headingPID, 11, 0, 20, 0);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.forward_drivePID, 0.45, 0, 5, 0);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.backward_drivePID, 0.45, 0, 5, 0);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.turnPID, 5, 0.003, 35, 15);
  StratusQuo::chassis.set_pid_constants(&StratusQuo::chassis.swingPID, 7, 0, 45, 0);
}


void modified_exit_condition() {
  StratusQuo::chassis.set_exit_condition(StratusQuo::chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  StratusQuo::chassis.set_exit_condition(StratusQuo::chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  StratusQuo::chassis.set_exit_condition(StratusQuo::chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}




///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  StratusQuo::chassis.set_drive_pid(24, DRIVE_SPEED, true);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_drive_pid(-12, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_drive_pid(-12, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  StratusQuo::chassis.set_turn_pid(90, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(45, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(0, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
}



///
// Combining Turn + Drive
///
void drive_and_turn() {
  StratusQuo::chassis.set_drive_pid(24, DRIVE_SPEED, true);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(45, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(-45, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(0, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  StratusQuo::chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  StratusQuo::chassis.set_drive_pid(24, DRIVE_SPEED, true);
  StratusQuo::chassis.wait_until(6);
  StratusQuo::chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(45, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(-45, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(0, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  StratusQuo::chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  StratusQuo::chassis.wait_until(-6);
  StratusQuo::chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  StratusQuo::chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  StratusQuo::chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_drive_pid(24, DRIVE_SPEED, true);
  StratusQuo::chassis.wait_until(12);

  StratusQuo::chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  StratusQuo::chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  StratusQuo::chassis.set_drive_pid(24, DRIVE_SPEED, true);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(45, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_turn_pid(0, TURN_SPEED);
  StratusQuo::chassis.wait_drive();

  StratusQuo::chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  StratusQuo::chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    StratusQuo::chassis.set_drive_pid(-12, 127);
    StratusQuo::chassis.wait_drive();

    // If failsafed...
    if (StratusQuo::chassis.interfered) {
      StratusQuo::chassis.reset_drive_sensor();
      StratusQuo::chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees. 
// If interfered, robot will drive forward and then attempt to drive backwards. 
void interfered_example() {
 StratusQuo::chassis.set_drive_pid(24, DRIVE_SPEED, true);
 StratusQuo::chassis.wait_drive();

 if (StratusQuo::chassis.interfered) {
   tug(3);
   return;
 }

 StratusQuo::chassis.set_turn_pid(90, TURN_SPEED);
 StratusQuo::chassis.wait_drive();
}



// . . .
// Make your own autonomous functions here!
// . . .

#pragma region Sig_Autons
void sig_red_ring_side()
{
  StratusQuo::chassis.set_drive_pid(-38, 70);
  StratusQuo::chassis.wait_drive();
  StratusQuo::clamp.set_value(true);
  pros::delay(100);
  StratusQuo::chassis.set_turn_pid(-90, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_turn_pid(-70, DRIVE_SPEED);
  StratusQuo::intake.move(127);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_turn_pid(-155, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(12, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(-5, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_turn_pid(-110, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(6, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::intake.brake();
}
void sig_red_goal_side()
{
  StratusQuo::chassis.set_drive_pid(-40, 70);
  StratusQuo::chassis.wait_drive();
  StratusQuo::clamp.set_value(true);
  pros::delay(100);
}

void sig_blue_ring_side()
{
  // Woah :O
}
void sig_blue_goal_side()
{
  // Woah :O
}
#pragma endregion Sig_Autons