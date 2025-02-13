#include "lady_brown.hpp"
#include "EZ-Template/util.hpp"
#include "pros/motors.h"

StratusQuo::Lady_Brown::Lady_Brown(const int port, const ez::PID pid, const pros::Rotation rotation) : _motor(port), _pid(pid), _rotation(rotation)
{
    _motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
};

int StratusQuo::Lady_Brown::move(int8_t voltage)
{
    _motor.move(voltage);
    return 0;
}

int StratusQuo::Lady_Brown::move_to(int position)
{
    _pid.target_set(position);
    while (_pid.exit_condition({_motor}, false) == ez::RUNNING)
    {
        _motor.move(_pid.compute(_rotation.get_position()));
        pros::delay(ez::util::DELAY_TIME);
    }
    return 0;
}

int StratusQuo::Lady_Brown::tare_position()
{
    _rotation.reset_position();
    return 0;
}

int StratusQuo::Lady_Brown::brake()
{
    _motor.brake();
    return 0;
}

double StratusQuo::Lady_Brown::compute(double current)
{
    return _pid.compute(current);
}

double StratusQuo::Lady_Brown::get_position()
{
    return _rotation.get_position() / (36.0/12);
}

void StratusQuo::Lady_Brown::pid_wait()
{
    while(_pid.exit_condition({_motor}) == ez::RUNNING) pros::delay(ez::util::DELAY_TIME);
}

void StratusQuo::Lady_Brown::set_exit_conditions(double small_error_time, double small_error, double big_error_time, double big_error, int velocity_exit_time, int mA_timeout)
{
    _pid.exit_condition_set(small_error_time, small_error, big_error_time, big_error, velocity_exit_time, mA_timeout);
}

void StratusQuo::Lady_Brown::target_set(int target)
{
    _pid.target_set(target);
}