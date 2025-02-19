#include "lady_brown.hpp"
#include "EZ-Template/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "robodash.hpp"

StratusQuo::Lady_Brown::Lady_Brown(const int port, const ez::PID pid, const pros::Rotation rotation) : _motor(port, pros::MotorGearset::green, pros::MotorUnits::rotations), _pid(pid), _rotation(rotation)
{
    _motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
};

int StratusQuo::Lady_Brown::move(int8_t voltage)
{
    _motor.move(voltage);
    return 0;
}

int StratusQuo::Lady_Brown::move_to(double position)
{
    _motor.move_absolute(position, 200);
    return 0;
}

int StratusQuo::Lady_Brown::compute_error(double error, double current)
{
    return _pid.compute_error(error, current);
}

int StratusQuo::Lady_Brown::tare_position()
{
    _rotation.reset_position();
    _motor.tare_position();
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
    return _motor.get_position();
}

ez::exit_output StratusQuo::Lady_Brown::get_exit_condition()
{
    return _pid.exit_condition();
}

double StratusQuo::Lady_Brown::get_voltage()
{
    return _motor.get_voltage();
}

void StratusQuo::Lady_Brown::pid_wait()
{
    while(_pid.exit_condition({_motor}) == ez::RUNNING) pros::delay(ez::util::DELAY_TIME);
}

void StratusQuo::Lady_Brown::set_exit_conditions(double small_error_time, double small_error, double big_error_time, double big_error, int velocity_exit_time, int mA_timeout)
{
    _pid.exit_condition_set(small_error_time, small_error, big_error_time, big_error, velocity_exit_time, mA_timeout);
}

void StratusQuo::Lady_Brown::target_set(double target)
{
    _pid.target_set(target);
}

double StratusQuo::Lady_Brown::get_target_position()
{
    return _motor.get_target_position();
}