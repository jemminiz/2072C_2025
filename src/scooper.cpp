#include "scooper.h"

StratusQuo::Scooper::Scooper(int8_t port) : scooper(port)
{};

void StratusQuo::Scooper::toggle()
{
    scooper.set_value(!state);
    state = !state;
}