#ifndef OBJECTS_H
#define OBJECTS_H

#include "arm.h"
#include "autons.h"
#include "basket.h"
#include "clamp.h"
#include "drivetrain.h"
#include "intake.h"
#include "scooper.h"

namespace StratusQuo
{
    extern pros::Controller master;
    extern pros::adi::DigitalIn limit_switch;
    extern pros::adi::DigitalOut intake_piston;
    extern pros::v5::Imu imu;

    extern Drivetrain dt;
    extern Clamp piston;
    extern Intake in;
    extern Auton autonomous;
    extern auton_type auton;
    extern Arm arm;
    extern Scooper scoop;
    extern Basket basket;
}

#endif