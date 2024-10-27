#include "intake.hpp"

StratusQuo::Intake::Intake(int motor_port, uint8_t pneumatic_port, bool starting_config) : motor(motor_port), piston(pneumatic_port, starting_config)
{};

void StratusQuo::Intake::intake()
{
    motor.move(127);
}
void StratusQuo::Intake::outtake()
{
    motor.move(-127);
}
void StratusQuo::Intake::move(int voltage)
{
    motor.move(voltage);
}
void StratusQuo::Intake::toggle()
{
    piston.toggle();
}
void StratusQuo::Intake::set_value(bool toggle)
{
    piston.set_value(toggle);
}
void StratusQuo::Intake::brake()
{
    motor.brake();
}