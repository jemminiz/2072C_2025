#include "intake.hpp"

StratusQuo::Intake::Intake(uint8_t roller_port, uint8_t hook_port, uint8_t piston_port) : _roller_motor(roller_port),
                                                                                          _hook_motor(hook_port),
                                                                                          _piston(piston_port)
{};

int StratusQuo::Intake::move_rollers(int8_t voltage)
{
    _roller_motor.move(voltage);
    return 0;
}
int StratusQuo::Intake::move_hooks(int8_t voltage)
{
    _hook_motor.move(voltage);
    return 0;
}
int StratusQuo::Intake::move(int8_t voltage)
{
    move_rollers(voltage);
    move_hooks(voltage);
    return 0;
}
int StratusQuo::Intake::toggle_piston()
{
    _piston.set(!_piston.get());
    return 0;
}
int StratusQuo::Intake::button_toggle(int button)
{
    _piston.button_toggle(button);
    return 0;
}
int StratusQuo::Intake::brake()
{
    _roller_motor.brake();
    _hook_motor.brake();
    return 0;
}