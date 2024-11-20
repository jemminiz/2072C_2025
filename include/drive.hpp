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
            ~Drivetrain();
            Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors);
            Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* imu);
            Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* imu, PID pid);
            int set_current_target(int target);
            int get_current_target();
            int drive_to(int voltage);
            int set_drive_voltage(int voltage);
            void task_loop();
        private:
            pros::MotorGroup* left = nullptr;
            pros::MotorGroup* right = nullptr;
            pros::Imu* imu = nullptr;
            PID left_pid;
            PID right_pid;
            pros::Task* drive_pid_task;
            int current_target = 0;
    };
}