#include "position.h"

StratusQuo::Position::Position()
{};

float StratusQuo::Position::getX()
{
    return x;
};

float StratusQuo::Position::getY()
{
    return y;
};

void StratusQuo::Position::changeX(float delta)
{
    x += delta;
};

void StratusQuo::Position::changeY(float delta)
{
    y += delta;
};