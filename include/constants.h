#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <cstdint>
#include <vector>

namespace StratusQuo
{
    inline constexpr int LEFT_MOTOR_BACK_PORT (1);
    inline constexpr int LEFT_MOTOR_MID_PORT (2);
    inline constexpr int LEFT_MOTOR_FRONT_PORT (3);
    inline constexpr int RIGHT_MOTOR_BACK_PORT (-4);
    inline constexpr int RIGHT_MOTOR_MID_PORT (-5);
    inline constexpr int RIGHT_MOTOR_FRONT_PORT (-6);

    inline constexpr int IMU_PORT (0);
    inline constexpr int CLAMP_PORT (0);
    inline constexpr int8_t INTAKE_PORT (10);

    inline constexpr int PID_DRIVE_MAX (127);
    inline constexpr int PID_DRIVE_MIN (-127);

    inline constexpr int PID_INTEGRAL_LIMIT (50);

    inline constexpr float kP (2.00);
    inline constexpr float kI (0.04);
    inline constexpr float kD (0.);

    extern std::vector<int8_t> left_motors;
    extern std::vector<int8_t> right_motors;
};
#endif