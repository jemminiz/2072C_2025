#include "clamp.h"

StratusQuo::Clamp::Clamp(uint8_t pneumatic_port) : clamp_pneumatics(pneumatic_port, false)
{};

void StratusQuo::Clamp::clamp()
{
    clamp_pneumatics.toggle();
}
void StratusQuo::Clamp::clamp_setvalue(bool toggle)
{
    clamp_pneumatics.set_value(toggle);
}
