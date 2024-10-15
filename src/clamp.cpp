#include "clamp.hpp"

StratusQuo::Clamp::Clamp(uint8_t clamp_port, bool init_state) : clamp(clamp_port, init_state), state(init_state)
{}

void StratusQuo::Clamp::toggle()
{
    clamp.set_value(!state);
    state = !state;
}

void StratusQuo::Clamp::set_value(bool toggle)
{
    clamp.set_value(toggle);
    state = toggle;
}