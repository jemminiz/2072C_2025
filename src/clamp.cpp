#include "clamp.hpp"

StratusQuo::Clamp::Clamp(uint8_t pneumatic_port, bool starting_config) : piston(pneumatic_port, starting_config)
{};

void StratusQuo::Clamp::toggle()
{
    piston.toggle();
    current_val = !current_val;
}
void StratusQuo::Clamp::extend()
{
    piston.extend();
}
void StratusQuo::Clamp::retract()
{
    piston.retract();
}