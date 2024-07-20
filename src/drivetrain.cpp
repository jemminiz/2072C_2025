#include "drivetrain.h"
#include "constants.h"
#include "pros.h"
#include "pros/misc.h"

Drivetrain::Drivetrain(std::vector<int8_t> left_mg, std::vector<int8_t> right_mg, std::optional<int> imuPort) : left_motor_group(left_mg), right_motor_group(right_mg),
imu(imuPort.value_or(0))
{};

void Drivetrain::drive()
{
    // Tank Drive
    left_motor_group.move_voltage(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    right_motor_group.move_voltage(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
}