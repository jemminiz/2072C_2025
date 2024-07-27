#include "PID.h"
#include "constants.h"

StratusQuo::PID::PID() : drivetrain(left_motors, right_motors, IMU_PORT)
{
};

void StratusQuo::PID::DriveTo(double heading)
{
    pidSensorCurrentValue = drivetrain.get_heading();

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

    drivetrain.setVoltage(pidDrive);
};