#include "scoop.h"

StratusQuo::Scoop::Scoop(uint8_t pneumatic_port) : scoop_pneumatics(pneumatic_port, false)
{};

void StratusQuo::Scoop::scoop()
{
    scoop_pneumatics.toggle();
}
void StratusQuo::Scoop::scoop_setvalue(bool toggle)
{
    scoop_pneumatics.set_value(toggle);
}