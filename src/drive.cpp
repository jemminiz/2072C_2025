#include "drive.hpp"

StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors) : left(left_motors), right(right_motors)
{}
StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* inertial) : left(left_motors), right(right_motors), imu(inertial)
{}

