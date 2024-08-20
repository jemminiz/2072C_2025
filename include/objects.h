#ifndef OBJECTS_H
#define OBJECTS_H

#include "clamp.h"
#include "drivetrain.h"
#include "intake.h"

namespace StratusQuo
{
    extern pros::Controller master;
	extern pros::adi::DigitalOut p;
    extern pros::v5::Imu imu;

    extern StratusQuo::Drivetrain dt;
    extern StratusQuo::Clamp piston;
    extern StratusQuo::Intake in;
}

#endif