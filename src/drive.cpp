#include "drive.hpp"
#include <cstdint>

StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors) : left(left_motors), right(right_motors)
{
    left_pid = PID();
    right_pid = PID();
    drive_pid_task = new pros::Task([this]() {this->task_loop();});
}
StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* imu) : left(left_motors), right(right_motors), imu(imu)
{
    left_pid = PID();
    right_pid = PID();
    drive_pid_task = new pros::Task([this]() {this->task_loop();});
}
StratusQuo::Drivetrain::Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, pros::Imu* imu, PID pid) : left(left_motors), right(right_motors), imu(imu), left_pid(pid), right_pid(pid)
{
    if(!pid.can_get_position())
    {
        uint32_t time = pros::millis();
        pid.set_position_feedback([this, &time]()-> std::int32_t { return left->get_raw_position(&time); }); // Use left motor, specifically index 0, as default position return
    }
    drive_pid_task = new pros::Task([this]() {this->task_loop();});
}

int StratusQuo::Drivetrain::set_current_target(int target)
{
    current_target = target;
    return 0;
}
int StratusQuo::Drivetrain::get_current_target()
{
    return current_target;
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

void StratusQuo::Drivetrain::task_loop() {
    auto set_drive_voltage_lambda = [this](std::int32_t voltage) -> std::int32_t {
        return this->set_drive_voltage(voltage);
    };

    while (true) {
        this->left_pid.move_to(current_target, set_drive_voltage_lambda);
        this->right_pid.move_to(current_target, set_drive_voltage_lambda);
        pros::Task::delay(10);
    }
}

StratusQuo::Drivetrain::~Drivetrain()
{
    if(drive_pid_task != nullptr)
    {
        delete (drive_pid_task);
        drive_pid_task = nullptr;
    }
}