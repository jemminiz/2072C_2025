#include "arm.h"
#include "constants.h"
#include "pros/motors.h"

StratusQuo::Arm::Arm() : motor1(ARM_PORT_1), motor2(ARM_PORT_2)
{
    motor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    motor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
};

void StratusQuo::Arm::arm_up()
{
    motor1.move(42);
    motor2.move(42);
}
void StratusQuo::Arm::arm_down()
{
    motor1.move(-42);
    motor2.move(-42);
}