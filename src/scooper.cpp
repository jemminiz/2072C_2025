#include "scooper.hpp"

StratusQuo::Scooper::Scooper(uint8_t port, bool init_state) : scoop(port, init_state), state(init_state)
{}

void StratusQuo::Scooper::toggle()
{
    scoop.set_value(!state);
    state = !state;
}