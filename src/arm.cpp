#include "arm.hpp"

StratusQuo::Arm::Arm(uint8_t motor_port,
            uint8_t pneumatic_port,
            uint8_t sensor_port) : arm_motor(motor_port), arm_pneumatics(pneumatic_port, false), arm_sensor(sensor_port), arm_PID(0.45, 0, 0, 0, "Arm")
{
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
};

void StratusQuo::Arm::arm_down()
{
    arm_motor.move(-127);
}
void StratusQuo::Arm::arm_up()
{
    arm_motor.move(127);
}
void StratusQuo::Arm::arm_move(int voltage)
{
    arm_motor.move(voltage);
}
double StratusQuo::Arm::get_rotation()
{
    return arm_sensor.get_position();
}
void StratusQuo::Arm::toggle()
{
    arm_pneumatics.set_value(!pneumatic_state);
    pneumatic_state = !pneumatic_state;
}
void StratusQuo::Arm::brake()
{
    arm_motor.brake();
}