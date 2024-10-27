#include "scooper.hpp"
#include "pros/adi.hpp"

StratusQuo::Scooper::Scooper(uint8_t pneumatic_port, bool starting_config) : piston(pneumatic_port, starting_config)
{};

void StratusQuo::Scooper::toggle()
{
    piston.toggle();
}