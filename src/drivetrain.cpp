/*
 * How do I want odom to work?
 * Run a loop that continually checks your position to where it wants to be on an (x, y) "graph" of the field, with starting position being 0,0.
 * Use tracking wheels to ensure that we're still on track to get where we want to go. 
 * Use inertial sensor to check global orientation
 */


#include "drivetrain.h"
#include "constants.h"
#include "objects.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cstdint>
#include <cmath>

StratusQuo::Drivetrain::Drivetrain(std::vector<int8_t> left_mg, std::vector<int8_t> right_mg, uint8_t imuPort, int8_t horizontal, int8_t vertical) : 
left_motor_group(left_mg), right_motor_group(right_mg), imu(imuPort), horizontal_wheel(horizontal),
vertical_wheel(vertical), pos(), right_back(right_mg[0]),right_mid (right_mg[1]), right_front (right_mg[2]),
left_back(left_mg[0]), left_mid(left_mg[1]), left_front(left_mg[2]), drive_task([this] {this->dt_task();})
{};

void StratusQuo::Drivetrain::drive()
{
    // Issue with left_back, left_mid

    // Tank Drive
    //left_motor_group.move(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    left_back.move(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    left_mid.move(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    left_front.move(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    //right_motor_group.move(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    right_back.move(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    right_mid.move(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    right_front.move(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    
}

void StratusQuo::Drivetrain::suspend_task()
{
    drive_task.suspend();
}
void StratusQuo::Drivetrain::driveTo(double heading)
{
    //left_motor_group.move_relative(heading, 127);
    //right_motor_group.move_relative(heading, 127);
    left_back.move_relative(heading, 127);
    left_mid.move_relative(heading, 127);
    left_front.move_relative(heading, 127);
    right_back.move_relative(heading, 127);
    right_mid.move_relative(heading, 127);
    right_front.move_relative(heading, 127);
    
}

void StratusQuo::Drivetrain::pid_drive(double target)
{
    while(left_front.get_position() != target)
    {
        left_pid.target_set(target);
        right_pid.target_set(target);
        setLeftVoltage(left_pid.compute(left_front.get_position()));
        setRightVoltage(right_pid.compute(right_front.get_position()));
        dt_wait();
    }
}

void StratusQuo::Drivetrain::turn(double theta)
{
    while(left_front.get_position() != theta)
    {
        left_pid.target_set(theta);
        right_pid.target_set(-theta);
        setLeftVoltage(left_pid.compute(left_front.get_position()));
        setRightVoltage(right_pid.compute(right_front.get_position()));
        dt_wait();
    }
}

void StratusQuo::Drivetrain::dt_wait()
{
    while(left_pid.exit_condition({left_front, right_front}, true) == RUNNING)
    {
        pros::delay(10);
    }
}

void StratusQuo::Drivetrain::dt_task()
{
    pros::delay(2000);
    while(true)
    {
        this->setLeftVoltage(left_pid.compute(left_front.get_position()));
        this->setRightVoltage(right_pid.compute(right_front.get_position()));
        pros::delay(10);
    }
}
void StratusQuo::Drivetrain::set_target(double target)
{
    left_pid.target_set(target);
    right_pid.target_set(target);
}

double StratusQuo::Drivetrain::get_heading()
{
    return imu.get_heading();
}
void StratusQuo::Drivetrain::setLeftVoltage(int voltage)
{
    //left_motor_group.move_voltage(voltage);
    //right_motor_group.move_voltage(voltage);
    left_back.move_voltage(voltage);
    left_mid.move_voltage(voltage);
    left_front.move_voltage(voltage);    
}
void StratusQuo::Drivetrain::setRightVoltage(int voltage)
{
    right_back.move_voltage(voltage);
    right_mid.move_voltage(voltage);
    right_front.move_voltage(voltage);
}

float StratusQuo::Drivetrain::calc_delta_orientation()
{
    float delta = this->get_rotation() - previous_global_orientation;
    previous_global_orientation = this->get_rotation();
    return delta;
}; // Should work properly

float StratusQuo::Drivetrain::get_delta_x()
{
    float delta = previous_x_position + pos.getX();
    previous_x_position = pos.getX();
    return delta;
};

float StratusQuo::Drivetrain::get_delta_y()
{
    float delta = previous_y_position + pos.getY();
    previous_y_position = pos.getY();
    return delta;
};

void StratusQuo::Drivetrain::calc_delta_position()
{
    float delta_x = this->get_delta_x();
    float delta_y = this->get_delta_y();
    
    pos.changeX(delta_x);
    pos.changeY(delta_y);
};

void StratusQuo::Drivetrain::odom_drive(double x_heading, double y_heading, double orientation)
{
    //imu.get_rotation() is absolute!!!
    //absolute turning algorithm for orientation?
    // change orientation to face a specific point (in inches)

    this->face_direction(x_heading, y_heading);
    driveTo(sqrt(((x_heading - pos.getX())* (x_heading - pos.getX())) + (y_heading - pos.getY()) * (y_heading - pos.getY())));
};

void StratusQuo::Drivetrain::face_direction(double x, double y)
{
    double angle = atan2(y, x);
    if(angle > 180)
    {
        while(this->get_rotation() != angle)
        {
            left_motor_group.move_voltage(127);
            right_motor_group.move_voltage(-127);
        }
        left_motor_group.brake();
        right_motor_group.brake();
    }
    else
    {
        while(this->get_rotation() != angle)
        {
            left_motor_group.move_voltage(-127);
            right_motor_group.move_voltage(127);
        }
        left_motor_group.brake();
        right_motor_group.brake();
    }
};

void StratusQuo::Drivetrain::initialize()
{
    left_front.tare_position();
    right_front.tare_position();
    imu.reset();
    left_pid.constants_set(kP, kI, kD);
    right_pid.constants_set(kP, kI, kD);
    left_pid.exit_condition_set(80, 50, 300, 150, 500, 500);
    right_pid.exit_condition_set(80, 50, 300, 150, 500, 500);
};

double StratusQuo::Drivetrain::get_rotation()
{
    return (std::fmod(imu.get_rotation(), M_2_PI));
};