#ifndef OBJECTS_H
#define OBJECTS_H

#include "clamp.h"
#include "drivetrain.h"

namespace StratusQuo
{
    extern pros::Controller master;
	extern pros::adi::DigitalOut p;

    extern StratusQuo::Drivetrain dt;
    extern StratusQuo::Clamp piston;
}

#endif