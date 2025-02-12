#include "clamp.hpp"

StratusQuo::Clamp::Clamp(uint8_t clamp_port) : _clamp_piston(clamp_port)
{};

int StratusQuo::Clamp::set(bool val)
{
    _clamp_piston.set(val);
    return 0;
}

bool StratusQuo::Clamp::get_state()
{
    return _clamp_piston.get();
}