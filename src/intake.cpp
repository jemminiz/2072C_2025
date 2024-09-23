#include "intake.h"
#include "pros/motors.h"

StratusQuo::Intake::Intake(uint8_t motor_port, uint8_t piston_port, bool starting_piston_state) : motor(motor_port), piston(piston_port, starting_piston_state)
{
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); // Could be an issue later
};

void StratusQuo::Intake::intake()
{
    motor.move_velocity(200);
};
void StratusQuo::Intake::outtake()
{
    motor.move_velocity(-600);
};
void StratusQuo::Intake::stop()
{
    motor.move_velocity(0);
}

void StratusQuo::Intake::toggle()
{
    piston.toggle();
};

