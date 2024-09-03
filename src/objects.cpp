#include "objects.h"
#include "constants.h"

namespace StratusQuo
{
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::adi::DigitalOut p(CLAMP_PORT);
    pros::adi::DigitalIn limit_switch(LIMIT_SWITCH_PORT);
    pros::adi::DigitalOut intake_piston(INTAKE_PISTON_PORT);
    pros::v5::Imu imu(IMU_PORT);

    Drivetrain dt (left_motors, right_motors, IMU_PORT, HORIZONTAL_WHEEL_PORT, VERTICAL_WHEEL_PORT);
    Clamp piston (p);
    Intake in;
    Auton autonomous;
    auton_type auton;
    Arm arm;
}