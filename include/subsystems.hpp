#pragma once
#include "arm.hpp"
#include "clamp.hpp"
#include "intake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "constants.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "limit_switch.hpp"
#include "pros/misc.h"
#include "scooper.hpp"
namespace StratusQuo
{
    inline pros::MotorGroup left_motors (left_motor_ports, pros::MotorGearset::blue);
    inline pros::MotorGroup right_motors (right_motor_ports, pros::MotorGearset::blue);
    inline lemlib::Drivetrain drive(&left_motors, &right_motors, 11.8, lemlib::Omniwheel::NEW_325, 450, 2);
    inline pros::Imu imu (IMU_PORT);
    inline pros::Rotation vertical (VERTICAL_PORT);
    inline pros::Rotation horizontal (HORIZONTAL_PORT);
    inline lemlib::TrackingWheel vertical_wheel (&vertical, lemlib::Omniwheel::NEW_2, .5);
    inline lemlib::TrackingWheel horizontal_wheel (&horizontal, lemlib::Omniwheel::NEW_2,  8.75 - (14.8125 / 2));
    inline lemlib::OdomSensors sensors (&vertical_wheel, nullptr, &horizontal_wheel, nullptr, &imu);
    // lateral PID controller
    inline lemlib::ControllerSettings lateral_controller(34, // proportional gain (kP) +3 31
                                              0, // integral gain (kI)
                                              210, // derivative gain (kD) +10  200
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
    );

    // angular PID controller
    inline lemlib::ControllerSettings angular_controller(8, // proportional gain (kP) +3 8
                                              0, // integral gain (kI)
                                              70, // derivative gain (kD) +10 60
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
    );
    
    enum Side
    {
        RED,
        BLUE
    };

    typedef struct robo_t
    {
        StratusQuo::Arm arm;
        StratusQuo::Clamp clamp;
        StratusQuo::Intake intake;
        StratusQuo::Scooper scooper;
        StratusQuo::Limit_Switch limit_switch;
        pros::Controller master;
        lemlib::Chassis chassis;
        Side side = BLUE;


        robo_t() : arm(ARM_PORT, ARM_PNEUMATICS_PORT, JACKS_INIT_STATE, ARM_ROTATION_PORT),
                   clamp(CLAMP_PORT, CLAMP_INIT_STATE),
                   intake(INTAKE_PORT, INTAKE_PISTON_PORT, INTAKE_INIT_STATE),
                   scooper(SCOOP_PORT, SCOOP_INIT_STATE),
                   limit_switch(LIMIT_SWITCH_PORT),
                   master(pros::E_CONTROLLER_MASTER),
                   chassis(drive, lateral_controller, angular_controller, sensors)
        {
        }
    } Robot;

    inline Robot robot;

    inline void limit_switch_task(void* params)
    {
        auto robot = ((StratusQuo::Robot*)params);
        uint32_t time;
        while(true)
        {
            if(robot->limit_switch.get_new_press())
            {
                time = pros::millis();
                robot->clamp.extend();
                pros::Task::delay_until(&time, 400);
            }
            if(robot->master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
            {
                time = pros::millis();
                pros::Task::delay_until(&time, 300);
            }
        }
    }
    inline void arm_task_fn(void* params)
    {
        auto robot = ((StratusQuo::Robot*)params);
        uint32_t time;
        while(true)
        {
            if(robot->master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) // Macro :D
            {
                time = pros::millis();
                robot->arm.move(127);
                pros::Task::delay_until(&time, 600); // Find a better timing for this
                robot->arm.toggle();
                robot->arm.move(-127);
                time = pros::millis();
                pros::Task::delay_until(&time, 600); // Same as above
                robot->arm.brake();
            }
            if(robot->master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) robot->arm.move(127);
            else if(robot->master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) robot->arm.move(-127);
            else robot->arm.move(0); // User Control
            time = pros::millis();
            pros::Task::delay_until(&time, 20);
        }
    }
    inline void drive_task_fn(void* params)
    {
        auto robot = ((Robot*)params);
        uint32_t time;
        while(true)
        {
            time = pros::millis();
            robot->chassis.tank(robot->master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), robot->master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
            pros::Task::delay_until(&time, 20);
        }
    } 
    inline void intake_task_fn(void* params)
    {
        auto robot = ((Robot*)params);
        uint32_t time;
        while(true)
        {
            time = pros::millis();
            if(robot->master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) robot->intake.move(127);
            else if(robot->master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) robot->intake.move(-127);
            else robot->intake.move(0);
            if(robot->master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) robot->intake.toggle();
            pros::Task::delay_until(&time, 20);
        }
    }
    inline void scooper_task_fn(void* params)
    {
        auto robot = ((Robot*)params);
        uint32_t time;
        while(true)
        {
            time = pros::millis();
            if(robot->master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) robot->scooper.toggle();
            pros::Task::delay_until(&time, 20);
        }
    }
    inline void clamp_task_fn(void* params)
    {
        auto robot = ((Robot*) params);
        uint32_t time;
        while(true)
        {
            time = pros::millis();
            if(robot->master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
            {
                robot->clamp.toggle();
                
            }
        }
    }
}