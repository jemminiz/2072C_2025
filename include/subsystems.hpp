#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "arm.h"
#include "constants.hpp"

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

namespace StratusQuo
{
    inline pros::Motor intake(INTAKE_PORT);
    inline pros::adi::Pneumatics intake_pneumatic(INTAKE_PISTON_PORT, INTAKE_STARTING_PISTON_STATE);
    
    inline StratusQuo::Arm arm(ARM_PORT, ARM_PNEUMATICS_PORT, ARM_ROTATION_PORT);

    inline pros::adi::Pneumatics clamp(CLAMP_PORT, false);

    inline pros::adi::Pneumatics scooper(SCOOP_PORT, SCOOP_START_EXTENDED);

    inline pros::adi::DigitalIn limit_switch(LIMIT_SWITCH_PORT);

    inline pros::v5::Imu imu(IMU_PORT);

    inline ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {LEFT_MOTOR_BACK_PORT, LEFT_MOTOR_MID_PORT, LEFT_MOTOR_FRONT_PORT},     // Left Chassis Ports (negative port will reverse it!)
    {RIGHT_MOTOR_BACK_PORT, RIGHT_MOTOR_MID_PORT, RIGHT_MOTOR_FRONT_PORT},  // Right Chassis Ports (negative port will reverse it!)

    IMU_PORT,      // IMU Port
    WHEEL_DIAMETER,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    RPM);   // Wheel RPM

}