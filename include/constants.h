#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <cstdint>
#include <vector>

namespace StratusQuo
{
    //Unused motor ports: 13, 14, 15, 16, 17, 18, 19, 20, 21
    
    inline constexpr int LEFT_MOTOR_BACK_PORT (5);
    inline constexpr int LEFT_MOTOR_MID_PORT (-3);
    inline constexpr int LEFT_MOTOR_FRONT_PORT (-8);
    inline constexpr int RIGHT_MOTOR_BACK_PORT (-1);
    inline constexpr int RIGHT_MOTOR_MID_PORT (2);
    inline constexpr int RIGHT_MOTOR_FRONT_PORT (11);
    inline constexpr uint8_t IMU_PORT (12);
    inline constexpr int HORIZONTAL_WHEEL_PORT (13);
    inline constexpr int VERTICAL_WHEEL_PORT (14);
    inline constexpr int ARM_PORT_1 (10);
    inline constexpr int ARM_PORT_2 (-9);
    inline constexpr int INTAKE_PORT (6);

    inline constexpr uint8_t CLAMP_PORT ('g');
    inline constexpr uint8_t INTAKE_PISTON_PORT ('b');
    inline constexpr uint8_t LIMIT_SWITCH_PORT ('h');
    inline constexpr uint8_t SCOOP_PORT ('f');

    inline constexpr int PID_DRIVE_MAX (127);
    inline constexpr int PID_DRIVE_MIN (127);

    inline constexpr int PID_INTEGRAL_LIMIT (50);

    inline constexpr float kP (2.0f);
    inline constexpr float kI (0.0f);
    inline constexpr float kD (0.0f);

    extern std::vector<int8_t> left_motors;
    extern std::vector<int8_t> right_motors;
};
#endif