#include "pros.h"
#include "objects.h"
#include "constants.h"

namespace StratusQuo
{
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::adi::DigitalOut p(1);

    Drivetrain dt (left_motors, right_motors, IMU_PORT);
    Clamp piston(p);
}