#include "clamp.h"
#include "constants.h"
#include "pros/adi.hpp"

StratusQuo::Clamp::Clamp(pros::adi::DigitalOut p) : piston(p)
{
    piston.set_value(false);
    currentState = false;
};
void StratusQuo::Clamp::toggle()
{
    currentState = !currentState;
    piston.set_value(currentState);
}
bool StratusQuo::Clamp::get_value()
{
    return currentState;
}
void StratusQuo::Clamp::set_value(bool in)
{
    piston.set_value(in);
    currentState = in;
}