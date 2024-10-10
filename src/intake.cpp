#include "intake.h"

StratusQuo::Intake::Intake(uint8_t motor_port,
            uint8_t pneumatic_port) : intake_motor(motor_port), intake_pneumatics(pneumatic_port, false), intake_PID(0.45, 0, 0, 0, "Intake")
{
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
};

void StratusQuo::Intake::outake()
{
    intake_motor.move(-127);
}
void StratusQuo::Intake::intake()
{
    intake_motor.move(127);
}
void StratusQuo::Intake::intake_auton(int voltage)
{
    intake_motor.move(voltage);
}
