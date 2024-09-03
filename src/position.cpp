#include "position.h"

StratusQuo::Position::Position()
{};

double StratusQuo::Position::getX()
{
    return x;
};

double StratusQuo::Position::getY()
{
    return y;
};

void StratusQuo::Position::changeX(double delta)
{
    x += delta;
};

void StratusQuo::Position::changeY(double delta)
{
    y += delta;
};