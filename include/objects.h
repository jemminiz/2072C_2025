#ifndef OBJECTS_H
#define OBJECTS_H

#include "autons.h"
#include "clamp.h"
#include "drivetrain.h"
#include "intake.h"

namespace StratusQuo
{
    extern pros::Controller master;
	extern pros::adi::DigitalOut p;
    extern pros::v5::Imu imu;

    extern Drivetrain dt;
    extern Clamp piston;
    extern Intake in;
    extern Auton autonomous;
    extern auton_type auton;
}

#endif