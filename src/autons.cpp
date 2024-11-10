#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "subsystems.hpp" // IWYU pragma: keep

lemlib::Pose defaultPose(0, 0, 0);

#pragma region Sig_Autons
void StratusQuo::sig_red_ring_side()
{
  StratusQuo::robot.chassis.setPose(defaultPose);
}
void StratusQuo::sig_red_goal_side()
{
  StratusQuo::robot.chassis.setPose(defaultPose);
}
void StratusQuo::sig_blue_ring_side()
{
  StratusQuo::robot.chassis.setPose(defaultPose);
}
void StratusQuo::sig_blue_goal_side()
{
  using namespace StratusQuo;
  robot.chassis.setPose(5, 12, 180);
  robot.chassis.moveToPoint(5, 28, 1100, {.forwards = false});
  robot.chassis.waitUntilDone();
  pros::delay(100);
  robot.chassis.moveToPoint(13.75, 50.5, 2500, {.forwards = false, .maxSpeed = 60});
  robot.chassis.waitUntilDone();
  pros::delay(500);
  /*
  robot.clamp.extend();
  robot.intake.move(-127);
  pros::delay(400);
  robot.intake.move(127);
  robot.chassis.waitUntilDone();
  robot.chassis.moveToPoint(17, 45, 1000);
  pros::delay(2000);
  robot.chassis.waitUntilDone();
  robot.clamp.retract();
  robot.intake.brake();
  robot.chassis.turnToHeading(270, 1000);
  robot.chassis.moveToPoint(45, 37, 1000, {.forwards = false, .maxSpeed = 60});
  robot.chassis.waitUntilDone();
  robot.intake.move(-127);
  robot.clamp.extend();
  pros::delay(200);
  robot.intake.move(127);
  robot.intake.toggle();
  robot.chassis.moveToPoint(70, 27, 2500);
  robot.chassis.waitUntilDone();
  robot.intake.toggle();
*/

  /*
  robot.chassis.moveToPoint(46, 45, 1000, {.forwards = false, .maxSpeed = 90});
  robot.clamp.retract();
  pros::delay(400);
  robot.clamp.extend();
  */ // Handled incorrectly!
}
#pragma endregion Sig_Autons

#pragma region Local_Autons
void StratusQuo::local_red_ring_side()
{
  StratusQuo::robot.chassis.setPose(defaultPose);
}
void StratusQuo::local_red_goal_side()
{
  StratusQuo::robot.chassis.setPose(defaultPose);
}
void StratusQuo::local_blue_ring_side()
{
  StratusQuo::robot.chassis.setPose(defaultPose);
}
void StratusQuo::local_blue_goal_side()
{
  StratusQuo::robot.chassis.setPose(defaultPose);
}
#pragma endregion Local_Autons