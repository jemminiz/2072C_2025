#pragma once

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pid.hpp"
namespace StratusQuo
{
    class Drivetrain
    {
        public:
            Drivetrain() = delete;
            Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors);
            Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* inertial);
            int drive_to(float voltage);
        private:
            pros::MotorGroup* left = nullptr;
            pros::MotorGroup* right = nullptr;
            pros::Imu* imu = nullptr;
            PID pid;
    };
}