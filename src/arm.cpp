#include "arm.hpp"

StratusQuo::Arm::Arm(int motor_port, uint8_t pneumatic_port, bool starting_config) : motor(motor_port), piston(pneumatic_port, starting_config)
{};

void StratusQuo::Arm::up()
{
    motor.move(127);
}
void StratusQuo::Arm::down()
{
    motor.move(-127);
}
void StratusQuo::Arm::move(int voltage)
{
    motor.move(voltage);
}
void StratusQuo::Arm::brake()
{
    motor.brake();
}