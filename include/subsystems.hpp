#pragma once
#include "arm.hpp"
#include "clamp.hpp"
#include "intake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "constants.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "scooper.hpp"
namespace StratusQuo
{
    inline pros::MotorGroup left_motors (left_motor_ports, pros::MotorGearset::blue);
    inline pros::MotorGroup right_motors (right_motor_ports, pros::MotorGearset::blue);
    inline lemlib::Drivetrain drive(&left_motors, &right_motors, 12.9, lemlib::Omniwheel::NEW_325, 450, 2);
    inline pros::Imu imu (IMU_PORT);
    inline pros::Rotation vertical (VERTICAL_PORT);
    inline pros::Rotation horizontal (HORIZONTAL_PORT);
    inline lemlib::TrackingWheel vertical_wheel (&vertical, lemlib::Omniwheel::NEW_275, -1);
    inline lemlib::TrackingWheel horizontal_wheel (&horizontal, lemlib::Omniwheel::NEW_275, -1);
    inline lemlib::OdomSensors sensors (&vertical_wheel, nullptr, &horizontal_wheel, nullptr, &imu);
    // lateral PID controller
    inline lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
    );

    // angular PID controller
    inline lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
    );

    inline lemlib::Chassis chassis (drive, lateral_controller, angular_controller, sensors);


    inline pros::Controller master(pros::E_CONTROLLER_MASTER);

    inline StratusQuo::Arm arm(ARM_PORT, ARM_PNEUMATICS_PORT, JACKS_INIT_STATE);
    inline StratusQuo::Clamp clamp(CLAMP_PORT, CLAMP_INIT_STATE);
    inline StratusQuo::Intake intake(INTAKE_PORT, INTAKE_PISTON_PORT, INTAKE_INIT_STATE);
    inline StratusQuo::Scooper scooper(SCOOP_PORT, SCOOP_INIT_STATE);
}