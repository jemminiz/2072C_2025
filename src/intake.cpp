#include "intake.h"
#include "constants.h"
#include "pros/motors.h"

StratusQuo::Intake::Intake() : motor(StratusQuo::INTAKE_PORT), piston(StratusQuo::INTAKE_PISTON_PORT, true), current_piston_state(true)
{
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); // Could be an issue later
};

void StratusQuo::Intake::intake()
{
    motor.move(127);
};
void StratusQuo::Intake::outtake()
{
    motor.move(-127);
};

void StratusQuo::Intake::move()
{
    piston.set_value(!current_piston_state);
    current_piston_state = !current_piston_state;
};