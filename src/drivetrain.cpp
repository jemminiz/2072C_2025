/*
 * How do I want odom to work?
 * Run a loop that continually checks your position to where it wants to be on an (x, y) "graph" of the field, with starting position being 0,0.
 * Use tracking wheels to ensure that we're still on track to get where we want to go. 
 * Use inertial sensor to check global orientation
 */


#include "drivetrain.h"
#include "constants.h"
#include "objects.h"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cstdint>
#include <cmath>

StratusQuo::Drivetrain::Drivetrain(std::vector<int8_t> left_mg, std::vector<int8_t> right_mg, uint8_t imuPort, int8_t horizontal, int8_t vertical) : 
left_motor_group(left_mg), right_motor_group(right_mg), imu(imuPort), horizontal_wheel(horizontal),
vertical_wheel(vertical), pos(), right_back(right_mg[0]),right_mid (right_mg[1]), right_front (right_mg[2]),
left_back(left_mg[0]), left_mid(left_mg[1]), left_front(left_mg[2])
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

void StratusQuo::Drivetrain::pidDriveHelper(double heading)
{
    double desired_heading = -heading;
    pidSensorCurrentValue = -imu.get_gyro_rate().z;

    pidError = pidSensorCurrentValue - desired_heading;

    if(kI != 0)
    {
        if(std::abs(pidError) < PID_INTEGRAL_LIMIT)
        {
            pidIntegral += pidError;
        }
        else pidIntegral = 0;
    }
    else pidIntegral = 0;

    pidDerivative = pidError - pidLastError;
    pidLastError = pidError;

    pidDrive = kP * pidError + kI * pidIntegral + kD * pidDerivative;
    if(pidDrive > PID_DRIVE_MAX) pidDrive = PID_DRIVE_MAX;
    if(pidDrive < PID_DRIVE_MIN) pidDrive = PID_DRIVE_MIN;

    this->setVoltage(pidDrive);
}
void StratusQuo::Drivetrain::pid_drive(double heading)
{
    while(-imu.get_gyro_rate().z != heading)
    {
        driveTo(heading);
        pros::delay(20);
    }
}
double StratusQuo::Drivetrain::get_heading()
{
    return imu.get_heading();
}
void StratusQuo::Drivetrain::setVoltage(int voltage)
{
    //left_motor_group.move_voltage(voltage);
    //right_motor_group.move_voltage(voltage);
    left_back.move_voltage(voltage);
    left_mid.move_voltage(voltage);
    left_front.move_voltage(voltage);
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
    imu.reset();
};

double StratusQuo::Drivetrain::get_rotation()
{
    return (std::fmod(imu.get_rotation(), M_2_PI));
};