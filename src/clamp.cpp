#include "clamp.h"

StratusQuo::Clamp::Clamp(uint8_t pneumatic_port) : clamp_pneumatics(pneumatic_port, false)
{};

void StratusQuo::Intake::outake()
{
    intake_motor.move(-127);
}
void StratusQuo::Intake::intake()
{
    intake_motor.move(127);
}
void StratusQuo::Intake::intake_auton(int voltage)
{
    intake_motor.move(voltage);
}
