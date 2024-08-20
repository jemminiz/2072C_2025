#include "intake.h"
#include "constants.h"
#include "pros/motors.h"

StratusQuo::Intake::Intake() : motor(StratusQuo::INTAKE_PORT)
{
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
};

void StratusQuo::Intake::intake()
{
    motor.move(127);
}
void StratusQuo::Intake::outtake()
{
    motor.move(-127);
}