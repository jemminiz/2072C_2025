#include "basket.h"

StratusQuo::Basket::Basket(uint8_t port, bool start_extended) : basket_pneumatic(port, start_extended)
{

};
void StratusQuo::Basket::toggle()
{
    basket_pneumatic.toggle();
}