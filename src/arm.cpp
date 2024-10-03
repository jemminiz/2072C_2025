#include "arm.h"
#include "constants.h"
#include "pros/motors.h"

StratusQuo::Arm::Arm() : motor(ARM_PORT)
{
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
};

void StratusQuo::Arm::arm_down()
{
    motor.move(-127);
}
void StratusQuo::Arm::arm_up()
{
    motor.move(127);
}
void StratusQuo::Arm::stop()
{
    motor.move(0);
}