#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "clamp.hpp"
#include "constants.hpp"
#include "main.h" // IWYU pragma: keep
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  StratusQuo::chassis.pid_drive_constants_set(24.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  StratusQuo::chassis.pid_heading_constants_set(3.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  StratusQuo::chassis.pid_turn_constants_set(4.4, 0.05, 20.0, 15.0);     // Turn in place constants
  StratusQuo::chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  StratusQuo::chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  StratusQuo::chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  StratusQuo::chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  StratusQuo::chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  StratusQuo::chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  StratusQuo::chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  StratusQuo::chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  StratusQuo::chassis.pid_turn_chain_constant_set(3_deg);
  StratusQuo::chassis.pid_swing_chain_constant_set(5_deg);
  StratusQuo::chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  StratusQuo::chassis.slew_turn_constants_set(3_deg, 70);
  StratusQuo::chassis.slew_drive_constants_set(3_in, 70);
  StratusQuo::chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  StratusQuo::chassis.odom_turn_bias_set(0.9);

  StratusQuo::chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  StratusQuo::chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  StratusQuo::chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  StratusQuo::chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  StratusQuo::chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  StratusQuo::chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  StratusQuo::chassis.pid_turn_set(90_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(45_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(0_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  StratusQuo::chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(45_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(-45_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(0_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  StratusQuo::chassis.pid_drive_set(24_in, 30, true);
  StratusQuo::chassis.pid_wait_until(6_in);
  StratusQuo::chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(45_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(-45_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(0_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  StratusQuo::chassis.pid_drive_set(-24_in, 30, true);
  StratusQuo::chassis.pid_wait_until(-6_in);
  StratusQuo::chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  StratusQuo::chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  StratusQuo::chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  StratusQuo::chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  StratusQuo::chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(45_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait_quick_chain();

  StratusQuo::chassis.pid_turn_set(-45_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait_quick_chain();

  StratusQuo::chassis.pid_turn_set(0_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  StratusQuo::chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  StratusQuo::chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(45_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_turn_set(0_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    StratusQuo::chassis.pid_drive_set(-12_in, 127);
    StratusQuo::chassis.pid_wait();

    // If failsafed...
    if (StratusQuo::chassis.interfered) {
      StratusQuo::chassis.drive_sensor_reset();
      StratusQuo::chassis.pid_drive_set(-2_in, 20);
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
  StratusQuo::chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();

  if (StratusQuo::chassis.interfered) {
    tug(3);
    return;
  }

  StratusQuo::chassis.pid_turn_set(90_deg, TURN_SPEED);
  StratusQuo::chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  StratusQuo::chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  StratusQuo::chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  StratusQuo::chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  StratusQuo::chassis.pid_wait();

  // Drive to 0, 0 backwards
  StratusQuo::chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  StratusQuo::chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  StratusQuo::chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  StratusQuo::chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  StratusQuo::chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  StratusQuo::chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  StratusQuo::chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  StratusQuo::chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  StratusQuo::chassis.pid_wait();

  StratusQuo::chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  StratusQuo::chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (StratusQuo::chassis.odom_tracker_left != nullptr) StratusQuo::chassis.odom_tracker_left->reset();
  if (StratusQuo::chassis.odom_tracker_right != nullptr) StratusQuo::chassis.odom_tracker_right->reset();
  if (StratusQuo::chassis.odom_tracker_back != nullptr) StratusQuo::chassis.odom_tracker_back->reset();
  if (StratusQuo::chassis.odom_tracker_front != nullptr) StratusQuo::chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    StratusQuo::chassis.pid_targets_reset();
    StratusQuo::chassis.drive_imu_reset();
    StratusQuo::chassis.drive_sensor_reset();
    StratusQuo::chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    StratusQuo::chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = StratusQuo::chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    StratusQuo::chassis.pid_turn_set(target, 63, ez::raw);
    StratusQuo::chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(StratusQuo::chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = StratusQuo::chassis.odom_tracker_left != nullptr ? StratusQuo::chassis.odom_tracker_left->get() : 0.0;
    double r_delta = StratusQuo::chassis.odom_tracker_right != nullptr ? StratusQuo::chassis.odom_tracker_right->get() : 0.0;
    double b_delta = StratusQuo::chassis.odom_tracker_back != nullptr ? StratusQuo::chassis.odom_tracker_back->get() : 0.0;
    double f_delta = StratusQuo::chassis.odom_tracker_front != nullptr ? StratusQuo::chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (StratusQuo::chassis.odom_tracker_left != nullptr) StratusQuo::chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (StratusQuo::chassis.odom_tracker_right != nullptr) StratusQuo::chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (StratusQuo::chassis.odom_tracker_back != nullptr) StratusQuo::chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (StratusQuo::chassis.odom_tracker_front != nullptr) StratusQuo::chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void StratusQuo::blue_ring_side()
{
  using namespace StratusQuo;
  is_red_team.store(false);
  chassis.drive_angle_set(180);
  chassis.pid_drive_set(-20_in, 110);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-8_in, 40);
  chassis.pid_wait();
  set_clamp.store(true);
  pros::delay(200);
  chassis.pid_turn_set(45_deg, 80);
  chassis.pid_wait();
  set_intake(127);
  chassis.pid_drive_set(19.85_in, 110);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(8_in, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 110);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 90);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::LEFT_SWING, 247_deg, 70, 25);
  chassis.pid_wait();
  chassis.pid_drive_set(36_in, 110);
  //intake.toggle_piston();
  chassis.pid_wait();
  hook_voltage.store(127);
  //intake.toggle_piston();
  chassis.pid_drive_set(-3_in, 40);
  chassis.pid_wait();
  chassis.pid_drive_set(6_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();
  /*
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 90);
  chassis.pid_wait();
  set_intake(0);
  chassis.pid_drive_set(4_in, 110);
  chassis.pid_wait();
  pros::delay(500);
  lady_brown.move_absolute(LB_POSITIONS[2]);
  while (!((lady_brown.get_position() < LB_POSITIONS[2] + 5) && (lady_brown.get_position() > LB_POSITIONS[2] - 5))) {
    pros::delay(2);
  }
  chassis.pid_drive_set(-6_in, 110);
  chassis.pid_wait_quick_chain();
  lady_brown.move_absolute(180);
  while (!((lady_brown.get_position() < 180 + 5) && (lady_brown.get_position() > 180 - 5))) {
    pros::delay(2);
  }
  chassis.pid_turn_set(-5_deg, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, 127);
  pros::delay(75);
  lady_brown.move(127); */
}

void StratusQuo::red_ring_side()
{
  using namespace StratusQuo;
  is_red_team.store(true);
  chassis.drive_angle_set(180);
  chassis.pid_drive_set(-20_in, 110);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-8_in, 40);
  chassis.pid_wait();
  set_clamp.store(true);
  pros::delay(200);
  chassis.pid_turn_set(315_deg, 80);
  chassis.pid_wait();
  set_intake(127);
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 270_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, 110);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 90);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 113_deg, 70, 25);
  chassis.pid_wait();
  chassis.pid_drive_set(36_in, 110);
  chassis.pid_wait_until(12_in);
  //intake.toggle_piston();
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, 50);
  hook_voltage.store(127);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 40);
  //intake.toggle_piston();
  chassis.pid_wait();
  /*
  chassis.pid_turn_set(180_deg, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, 110);
  pros::delay(750);
  hook_voltage.set(-20);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-12_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, 80);
  chassis.pid_wait();
  lady_brown.move(127); */
}

void StratusQuo::red_goal_rush()
{
  using namespace StratusQuo;
  is_red_team = true;
  color_sort_is_enabled = false;
  lady_brown.set_zero_position(90);
  lady_brown.move_absolute(100, 200);
  chassis.pid_drive_set(42_in, 90);
  lady_brown.move_absolute(360, 62);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 90);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 75_deg, 70);
  lady_brown.move_absolute(0, 200);
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 110);
  chassis.pid_wait();
  pros::delay(200);
  set_clamp.store(true);
  pros::delay(200);
  chassis.pid_drive_set(27_in, 110);
  set_intake(127);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(-75_deg, 70);
  chassis.pid_wait();
  set_intake(0);
  chassis.pid_drive_set(18_in, 80);
  chassis.pid_wait();
  right_doinker.set(!right_doinker.get());
  pros::delay(300);
  chassis.pid_swing_set(ez::LEFT_SWING, -45_deg, 80);
  chassis.pid_wait();
  left_doinker.set(!left_doinker.get());
  pros::delay(500);
  chassis.pid_drive_set(-48_in, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, 40);
  chassis.pid_wait();
  left_doinker.set(!left_doinker.get());
  right_doinker.set(!right_doinker.get());
}

void StratusQuo::blue_goal_rush()
{
  chassis.odom_theta_flip();
  chassis.odom_x_flip();
  red_goal_rush();
}

void StratusQuo::red_goal_side_basic()
{
  using namespace StratusQuo;
  chassis.drive_angle_set(180_deg);
  chassis.pid_drive_set(-18_in, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in, 40);
  chassis.pid_wait();
  set_clamp.store(true);
  pros::delay(200);
  chassis.pid_turn_set(90_deg, 80);
  set_intake(127);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(150_deg, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, 80);
  set_clamp.store(false);
  chassis.pid_drive_set(48_in, 110);
  chassis.pid_wait();
}

void StratusQuo::blue_goal_side_basic()
{
  using namespace StratusQuo;
  chassis.odom_theta_flip();
  chassis.odom_x_flip();
  red_goal_side_basic();
}

void StratusQuo::blue_goal_NATE()
{
  using namespace StratusQuo;
  chassis.drive_angle_set(140_deg);
  is_red_team.store(false);
  lady_brown.set_zero_position(90);
  
  lady_brown.move_absolute(1450, 200);
  while (!((lady_brown.get_position() < lady_brown.get_target_position() + 5) && (lady_brown.get_position() > lady_brown.get_target_position() - 5)))
  {
    pros::delay(2);
  }
  chassis.pid_turn_set(140_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-33_in, 110);
  chassis.pid_wait_quick();
  set_clamp.store(true);
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, 70);
  set_intake(127);
  chassis.pid_wait();
  chassis.pid_drive_set(28_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(220_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(33_in, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(8_in, 110);
}

void StratusQuo::skills()
{
  using namespace StratusQuo;
  chassis.drive_angle_set(230_deg);
  is_red_team.store(true);
  color_sort_is_enabled.store(false);
  lady_brown.set_zero_position(90);
  
  lady_brown.move_absolute(1420, 600);
  while (!((lady_brown.get_position() < 1420 + 7) && (lady_brown.get_position() > 1420 - 7)))
  {
    pros::delay(2);
  }

  chassis.pid_drive_set(-12_in, 110);
  chassis.pid_wait();
  lady_brown.move_absolute(360, 200);
  chassis.pid_drive_set(-3_in, 50);
  chassis.pid_wait_quick();
  set_clamp.store(true);
  chassis.pid_wait(); // Adding this to ensure full stop. Don't know if it's needed
  lady_brown.brake();

  chassis.pid_turn_set(10_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 110);
  set_intake(127);
  chassis.pid_wait(); // First Ring

  chassis.pid_turn_set(30_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(60_in, 110);
  chassis.pid_wait(); // Second Ring
  
  chassis.pid_turn_set(170_deg, 70);
  pros::delay(300);
  lady_brown.move_absolute(60, 200);
  while (!((lady_brown.get_position() < 60 + 2) && (lady_brown.get_position() > 60 - 2)))
  {
    pros::delay(2);
  }
  chassis.pid_wait();
  chassis.pid_drive_set(30_in, 110);
  chassis.pid_wait();
  
  chassis.pid_turn_set(90_deg, 70);
  chassis.pid_wait();
  pros::delay(500);
  hook_voltage.store(-10);
  lady_brown.move_absolute(905, 200);
  chassis.pid_drive_set(1.5_in, 40);
  while (!((lady_brown.get_position() < lady_brown.get_target_position() + 2) && (lady_brown.get_position() > lady_brown.get_target_position() - 2)))
  {
    pros::delay(2);
  }
  chassis.pid_drive_set(-18_in, 110);
  lady_brown.move(-127);
  pros::delay(100);
  lady_brown.brake();
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 70);
  chassis.pid_wait();
  set_intake(127);
  chassis.pid_drive_set(60_in, 90);
  chassis.pid_wait();
  chassis.pid_turn_set(60_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, 110);
  chassis.pid_wait();
  set_clamp.store(false);
  set_intake(0);
  hook_voltage.store(-127);

  chassis.pid_drive_set(13_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-69_in, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, 50);
  chassis.pid_wait_quick();
  set_clamp.store(true);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 110);
  set_intake(127);
  chassis.pid_wait();
  chassis.pid_turn_set(-30_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(50_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(-165_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 110);
  hook_voltage.store(0);
  lady_brown.move_absolute(174, 200);
  while (!((lady_brown.get_position() < lady_brown.get_target_position() + .5) && (lady_brown.get_position() > lady_brown.get_target_position() - .5)))
  {
    pros::delay(2);
  }
  hook_voltage.store(127);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, 110);
  set_intake(0);
  lady_brown.move_absolute(900, 200);
  while (!((lady_brown.get_position() < lady_brown.get_target_position() + .5) && (lady_brown.get_position() > lady_brown.get_target_position() - .5)))
  {
    pros::delay(2);
  }

  chassis.pid_drive_set(-20_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(-180_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(60_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(-60_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(15_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, 110);
  chassis.pid_wait();
  set_clamp.store(false);
  hook_voltage.store(-127);
  chassis.pid_drive_set(80_in, 110);
  roller_voltage.store(127);
  hook_voltage.store(0);
}