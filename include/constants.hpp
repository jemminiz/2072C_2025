#pragma once
#include <cstdint>

namespace StratusQuo
{
    #pragma region Smart_Ports
    inline constexpr int LEFT_MOTOR_BACK_PORT (5);
    inline constexpr int LEFT_MOTOR_MID_PORT (-3);
    inline constexpr int LEFT_MOTOR_FRONT_PORT (-8);
    inline constexpr int RIGHT_MOTOR_BACK_PORT (-1);
    inline constexpr int RIGHT_MOTOR_MID_PORT (2);
    inline constexpr int RIGHT_MOTOR_FRONT_PORT (11);
    inline constexpr int ARM_PORT (10);
    inline constexpr int INTAKE_PORT (7);
    #pragma endregion Smart_Ports

    #pragma region Sensor_Ports
    inline constexpr uint8_t IMU_PORT (12);
    inline constexpr uint8_t ARM_ROTATION_PORT (38);
    #pragma endregion Sensor_Ports
    
    #pragma region Piston_Ports
    inline constexpr uint8_t CLAMP_PORT ('g');
    inline constexpr uint8_t INTAKE_PISTON_PORT ('b');
    inline constexpr uint8_t LIMIT_SWITCH_PORT ('h');
    inline constexpr uint8_t SCOOP_PORT ('f');
    inline constexpr uint8_t BASKET_PORT ('a');
    inline constexpr uint8_t ARM_PNEUMATICS_PORT ('c');
    #pragma endregion Piston_Ports

    #pragma region Const_Vals
    inline constexpr bool BASKET_START_EXTENDED (false);
    inline constexpr bool INTAKE_STARTING_PISTON_STATE (true);
    inline constexpr bool SCOOP_START_EXTENDED (false);
    inline constexpr float WHEEL_DIAMETER (3.25);
    inline constexpr int RPM (450);
    #pragma endregion Const_Vals
}