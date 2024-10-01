#include "objects.h"
#include "constants.h"

namespace StratusQuo
{
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::adi::DigitalIn limit_switch(LIMIT_SWITCH_PORT);
    pros::adi::DigitalOut intake_piston(INTAKE_PISTON_PORT);
    pros::v5::Imu imu(IMU_PORT);

    Drivetrain dt (left_motors, right_motors, IMU_PORT, HORIZONTAL_WHEEL_PORT, VERTICAL_WHEEL_PORT);
    Clamp piston (CLAMP_PORT);
    Intake in (INTAKE_PORT, INTAKE_PISTON_PORT, INTAKE_STARTING_PISTON_STATE);
    Auton autonomous;
    auton_type auton = auton_type::GOAL_RUSH;
    Arm arm;
    Scooper scoop(SCOOP_PORT, SCOOP_START_EXTENDED);
    Basket basket(BASKET_PORT, BASKET_START_EXTENDED);
}