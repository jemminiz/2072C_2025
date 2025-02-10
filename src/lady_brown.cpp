#include "lady_brown.hpp"

StratusQuo::Lady_Brown::Lady_Brown(const int port, const ez::PID pid, const pros::Rotation rotation) : _motor(port), _pid(pid), _rotation(rotation)
{};

int StratusQuo::Lady_Brown::move(int8_t voltage)
{
    _motor.move(voltage);
    return 0;
}

int StratusQuo::Lady_Brown::move_to(int position)
{
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