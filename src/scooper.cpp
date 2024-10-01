#include "scooper.h"

StratusQuo::Scooper::Scooper(int8_t port, bool start_extended) : scooper(port, start_extended)
{};

void StratusQuo::Scooper::toggle()
{
    scooper.toggle();
}