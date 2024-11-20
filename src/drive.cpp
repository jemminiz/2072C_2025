#include "drive.hpp"
#include <cstdint>

StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors) : left(left_motors), right(right_motors)
{}
StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* inertial) : left(left_motors), right(right_motors), imu(inertial)
{}

int StratusQuo::Drivetrain::drive_to(float target)
{
    pid.move_to(target, [this](std::int32_t voltage) { return left->move(voltage); });
    pid.move_to(target, [this](std::int32_t voltage) { return right->move(voltage); });
    return 0;
}