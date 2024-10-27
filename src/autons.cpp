#include "autons.hpp"
#include "subsystems.hpp"
#pragma region Sig_Autons
/*
void sig_red_ring_side()
{
  // Woah :O
  StratusQuo::chassis.set_drive_pid(-45, 70);
  StratusQuo::chassis.wait_drive();
  StratusQuo::clamp.set_value(true);
  pros::delay(100);
  StratusQuo::chassis.set_turn_pid(95, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(26, DRIVE_SPEED);
  StratusQuo::intake.move(127);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_turn_pid(180, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(17, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(-7, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_turn_pid(155, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  pros::delay(200);
  StratusQuo::chassis.set_drive_pid(10, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  pros::delay(500);
  
}
void sig_red_goal_side()
{
  StratusQuo::chassis.wait_drive();
  StratusQuo::clamp.set_value(true);
  pros::delay(100);
  StratusQuo::chassis.set_turn_pid(-95, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(26, DRIVE_SPEED);
  StratusQuo::intake.move(127);
  StratusQuo::chassis.wait_drive();
}
int limit_switch_task(){
  while (true)
  {
    if  (StratusQuo::limit_switch.get_new_press())
    {
      StratusQuo::clamp.set_value(true);
    }
    pros::delay(200);
  }
}
void sig_blue_ring_side()
{
  StratusQuo::chassis.set_drive_pid(-45, 70);
  pros::Task t(limit_switch_task);
  StratusQuo::chassis.wait_drive();
  StratusQuo::clamp.set_value(true);
  pros::delay(100);
  StratusQuo::chassis.set_turn_pid(-95, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(26, DRIVE_SPEED);
  StratusQuo::intake.move(127);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_turn_pid(-180, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(17, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(-7, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_turn_pid(-155, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  pros::delay(200);
  StratusQuo::chassis.set_drive_pid(10, DRIVE_SPEED);
  StratusQuo::chassis.wait_drive();
  pros::delay(500);
  // StratusQuo::chassis.set_turn_pid(-315, 40);
  // pros::delay(500);
  // StratusQuo::chassis.wait_drive();
  // StratusQuo::chassis.set_drive_pid(65, DRIVE_SPEED);
  // StratusQuo::intake.toggle();
  // StratusQuo::chassis.wait_drive();
  // StratusQuo::intake.toggle();
  // pros::delay(250);
  // StratusQuo::clamp.set_value(false);
  // StratusQuo::chassis.set_drive_pid(0, 0);
}
void sig_blue_goal_side()
{
  StratusQuo::chassis.set_drive_pid(-45, 70);
  StratusQuo::chassis.wait_drive();
  StratusQuo::clamp.set_value(true);
  pros::delay(100);
  StratusQuo::chassis.set_turn_pid(95, TURN_SPEED);
  StratusQuo::chassis.wait_drive();
  StratusQuo::chassis.set_drive_pid(26, DRIVE_SPEED);
  StratusQuo::intake.move(127);
  StratusQuo::chassis.wait_drive();
}
#pragma endregion Sig_Autons
*/