#include "drivetrain.h"
#include "constants.h"
#include "objects.h"
#include "pros/misc.h"

StratusQuo::Drivetrain::Drivetrain(std::vector<int8_t> left_mg, std::vector<int8_t> right_mg, int imuPort, int8_t horizontal, int8_t vertical) : 
left_motor_group(left_mg), right_motor_group(right_mg), imu(imuPort), horizontal_wheel(horizontal),
vertical_wheel(vertical)
{};

void StratusQuo::Drivetrain::drive()
{
    // Tank Drive
    left_motor_group.move_voltage(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    right_motor_group.move_voltage(StratusQuo::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
}
void StratusQuo::Drivetrain::driveTo(double heading)
{
    pidSensorCurrentValue = this->get_heading();

    pidError = pidSensorCurrentValue - heading;

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
double StratusQuo::Drivetrain::get_heading()
{
    return this->get_heading();
}
void StratusQuo::Drivetrain::setVoltage(int voltage)
{
    left_motor_group.move_voltage(voltage);
    right_motor_group.move_voltage(voltage);
}

float StratusQuo::Drivetrain::calc_delta_orientation()
{
    return previous_global_orientation;
}

void StratusQuo::Drivetrain::odom_drive(double heading, double orientation)
{
    
};