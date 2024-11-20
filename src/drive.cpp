#include "drive.hpp"
#include <cstdint>

StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors) : left(left_motors), right(right_motors), drive_pid_task(nullptr)
{
    left_pid = PID();
    right_pid = PID();
}
StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* imu) : left(left_motors), right(right_motors), imu(imu), drive_pid_task(nullptr)
{
    left_pid = PID();
    right_pid = PID();
}
StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* imu, PID pid) : left(left_motors), right(right_motors), imu(imu), left_pid(pid), right_pid(pid),
    drive_pid_task([this, &pid](float target) -> void {
        // Use a lambda that calls the member function
        auto set_drive_voltage_lambda = [this](std::int32_t voltage) -> std::int32_t
        {
            return this->set_drive_voltage(voltage); // Call the member function
        };
    
        // Pass the lambda to move_to
        this->left_pid.move_to(target, set_drive_voltage_lambda);
        this->right_pid.move_to(target, set_drive_voltage_lambda);
    })
{
    if(!pid.canGetPosition())
    {
        uint32_t time = pros::millis();
        pid.setPositionFeedback([this, &time]()-> std::int32_t { return left->get_raw_position(&time); }); // Use left motor, specifically index 0, as default position return
    }
}

int StratusQuo::Drivetrain::drive_to(int target)
{
    left_pid.move_to(target, [this](std::int32_t voltage) { return left->move(voltage); });
    right_pid.move_to(target, [this](std::int32_t voltage) { return right->move(voltage); });
    return 0;
}

int StratusQuo::Drivetrain::set_drive_voltage(int voltage)
{
    left->move(voltage);
    right->move(voltage);
    return 0;
}