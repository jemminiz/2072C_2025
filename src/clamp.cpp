#include "clamp.hpp"

StratusQuo::Clamp::Clamp(uint8_t pneumatic_port, bool starting_config) : piston(pneumatic_port, starting_config)
{};

void StratusQuo::Clamp::toggle()
{
    piston.toggle();
}
void StratusQuo::Clamp::set_value(bool toggle)
{
    piston.set_value(toggle);
}