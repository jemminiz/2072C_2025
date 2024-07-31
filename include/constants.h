#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "pros.h"
#include "pros/misc.h"
#include <vector>
#include "drivetrain.h"

namespace StratusQuo
{
    inline constexpr int LEFT_MOTOR_BACK_PORT (1);
    inline constexpr int LEFT_MOTOR_MID_PORT (1);
    inline constexpr int LEFT_MOTOR_FRONT_PORT (1);
    inline constexpr int RIGHT_MOTOR_BACK_PORT (1);
    inline constexpr int RIGHT_MOTOR_MID_PORT (1);
    inline constexpr int RIGHT_MOTOR_FRONT_PORT (1);

    inline constexpr int IMU_PORT (0);

    inline constexpr int PID_DRIVE_MAX (127);
    inline constexpr int PID_DRIVE_MIN (-127);

    inline constexpr int PID_INTEGRAL_LIMIT (50);

    inline constexpr float kP (2.00);
    inline constexpr float kI (0.04);
    inline constexpr float kD (0.);

    extern pros::Controller master;

    extern std::vector<int8_t> left_motors;
    extern std::vector<int8_t> right_motors;

    extern StratusQuo::Drivetrain dt;
};
#endif