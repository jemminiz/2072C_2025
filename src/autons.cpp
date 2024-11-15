#include "autons.hpp"
#include <cmath>
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
  using namespace StratusQuo;
  robot.chassis.setPose(139, 12, 180);
  robot.chassis.moveToPoint(139, 26, 1100, {.forwards = false});
  robot.chassis.waitUntilDone();
  double dist = std::sqrt((54.58 - 26) * (54.58 - 26) + (126.9 - 139) * (126.9 - 139));
  pros::delay(100);
  robot.chassis.moveToPoint(126.9, 54.58, 2500, {.forwards = false, .maxSpeed = 60});
  robot.chassis.waitUntil(dist);
  robot.clamp.extend();
  robot.intake.move(-127);
  pros::delay(400);
  robot.intake.move(127);
  robot.chassis.waitUntilDone();
  robot.chassis.moveToPoint(122, 48, 1000);
  pros::delay(2000);
  robot.chassis.waitUntilDone();
  robot.clamp.retract();
  robot.intake.brake();
  robot.chassis.turnToHeading(90, 1000);
  robot.chassis.moveToPoint(99, 52, 1000, {.forwards = false, .maxSpeed = 60});
  robot.chassis.waitUntilDone();
  robot.intake.move(-127);
  robot.clamp.extend();
  pros::delay(200);
  robot.intake.move(127);
  robot.intake.toggle();
  robot.chassis.moveToPoint(74, 40, 2500);
  robot.chassis.waitUntilDone();
  robot.intake.toggle();
  robot.chassis.moveToPoint(81, 24.3, 1000, {.forwards = false});
  robot.arm.move(127);
  pros::delay(2000);
  robot.clamp.retract();
  robot.chassis.moveToPoint(91, 40, 2000);
  robot.chassis.moveToPoint(91, 90, 5000);
}
void StratusQuo::sig_blue_ring_side()
{
  StratusQuo::robot.chassis.setPose(defaultPose);
}
void StratusQuo::sig_blue_goal_side()
{
  using namespace StratusQuo;
  robot.chassis.setPose(5, 12, 180);
  robot.chassis.moveToPoint(5, 26, 1100, {.forwards = false});
  robot.chassis.waitUntilDone();
  double dist = std::sqrt((54.58 - 26) * (54.58 - 26) + (17.1 - 5) * (17.1 - 5));
  pros::delay(100);
  robot.chassis.moveToPoint(17.1, 54.58, 2500, {.forwards = false, .maxSpeed = 60});
  robot.chassis.waitUntil(dist);
  robot.clamp.extend();
  robot.intake.move(-127);
  pros::delay(400);
  robot.intake.move(127);
  robot.chassis.waitUntilDone();
  robot.chassis.moveToPoint(22, 48, 1000);
  pros::delay(2000);
  robot.chassis.waitUntilDone();
  robot.clamp.retract();
  robot.intake.brake();
  robot.chassis.turnToHeading(270, 1000);
  robot.chassis.moveToPoint(45, 52, 1000, {.forwards = false, .maxSpeed = 60});
  robot.chassis.waitUntilDone();
  robot.intake.move(-127);
  robot.clamp.extend();
  pros::delay(200);
  robot.intake.move(127);
  robot.intake.toggle();
  robot.chassis.moveToPoint(70, 40, 2500);
  robot.chassis.waitUntilDone();
  robot.intake.toggle();
  robot.chassis.moveToPoint(63, 24.3, 1000, {.forwards = false});
  robot.arm.move(127);
    pros::delay(2000);
  robot.clamp.retract();
  robot.chassis.moveToPoint(53, 40, 2000);
  robot.chassis.moveToPoint(53, 90, 5000);
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