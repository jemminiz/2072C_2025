#pragma once
#include "EZ-Template/drive/drive.hpp"
#include "arm.hpp"
#include "clamp.hpp"
#include "constants.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "scooper.hpp"

namespace StratusQuo
{
    inline pros::Motor intake(INTAKE_PORT);
    inline pros::ADIDigitalOut intake_pneumatic(INTAKE_PISTON_PORT, INTAKE_INIT_STATE);
    
    inline StratusQuo::Arm arm(ARM_PORT, ARM_PNEUMATICS_PORT, ARM_ROTATION_PORT);

    inline StratusQuo::Clamp clamp(CLAMP_PORT, CLAMP_INIT_STATE);

    inline StratusQuo::Scooper scooper(SCOOP_PORT, SCOOP_INIT_STATE);

    inline pros::ADIDigitalIn limit_switch(LIMIT_SWITCH_PORT);

    inline Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {LEFT_MOTOR_BACK_PORT, LEFT_MOTOR_MID_PORT, LEFT_MOTOR_FRONT_PORT},     // Left Chassis Ports (negative port will reverse it!)
    {RIGHT_MOTOR_BACK_PORT, RIGHT_MOTOR_MID_PORT, RIGHT_MOTOR_FRONT_PORT},  // Right Chassis Ports (negative port will reverse it!)

    IMU_PORT,      // IMU Port
    WHEEL_DIAMETER,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    RPM,
    GEAR_RATIO);

}