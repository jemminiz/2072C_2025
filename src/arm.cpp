#include "arm.hpp"

StratusQuo::Arm::Arm(int motor_port, uint8_t pneumatic_port, bool starting_config, int sensor_port) : motor(motor_port), piston(pneumatic_port, starting_config), sensor(sensor_port)
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
void StratusQuo::Arm::toggle()
{
    piston.toggle();
}
void StratusQuo::Arm::initialize()
{
    sensor.reset();
    sensor.reset_position();
}

double StratusQuo::Arm::get_rotation()
{
    return sensor.get_angle();
}