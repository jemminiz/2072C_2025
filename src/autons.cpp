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
  robot.chassis.moveToPoint(5, 36, 1000, {.forwards = false});
  robot.chassis.waitUntilDone();
  pros::delay(100);
  robot.chassis.moveToPoint(20.588, 68, 1000, {.forwards = false, .maxSpeed = 90});
  robot.chassis.waitUntilDone();
  robot.clamp.extend();
  robot.intake.move(-127);
  pros::delay(200);
  robot.intake.move(127);
  robot.chassis.moveToPoint(21, 48, 1000);
  pros::delay(1500);
  robot.clamp.retract();
  robot.intake.brake();
  robot.chassis.moveToPoint(21, 40, 1000);
  robot.chassis.waitUntilDone();
  robot.chassis.moveToPoint(21, 38, 500);
  robot.chassis.moveToPoint(48, 41, 1000);
  robot.intake.move(127);
  robot.chassis.waitUntilDone();
  robot.intake.brake();
  robot.chassis.moveToPoint(50, 43, 1000, {.forwards = false});
  robot.chassis.turnToHeading(180, 1000);
  robot.chassis.waitUntilDone();
  robot.chassis.moveToPoint(50, 55, 1000, {.forwards = false});
  robot.chassis.waitUntilDone();
  robot.clamp.extend();
  pros::delay(200);
  robot.intake.toggle();
  robot.chassis.moveToPoint(65, 36, 1500);
  robot.intake.move(127);
  robot.chassis.waitUntilDone();
  robot.intake.toggle();
  pros::delay(100);
  robot.chassis.moveToPoint(66, 36.5, 500, {.forwards = false});

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