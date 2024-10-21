#include "intake.hpp"
#include <cstdio>

StratusQuo::Intake::Intake(int motor_port, uint8_t pneumatic_port, bool init_state) : motor(motor_port),
    pneumatic(pneumatic_port, init_state), state(init_state)
{
}

void StratusQuo::Intake::move(int voltage)
{
    motor.move(voltage);
}

void StratusQuo::Intake::brake()
{
    motor.brake();
}

void StratusQuo::Intake::toggle()
{
    pneumatic.set_value(!state);
    state = !state;
}