#include "limit_switch.hpp"

StratusQuo::Limit_Switch::Limit_Switch(uint8_t port) : limit_switch(port)
{}

bool StratusQuo::Limit_Switch::get_new_press()
{
    return limit_switch.get_new_press();
}