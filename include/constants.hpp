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
    inline constexpr uint8_t ARM_ROTATION_PORT (21);
    #pragma endregion Sensor_Ports
    
    #pragma region Piston_Ports
    inline constexpr uint8_t CLAMP_PORT ('g');
    inline constexpr uint8_t INTAKE_PISTON_PORT ('e');
    inline constexpr uint8_t LIMIT_SWITCH_PORT ('h');
    inline constexpr uint8_t SCOOP_PORT ('f');
    inline constexpr uint8_t ARM_PNEUMATICS_PORT ('a');
    #pragma endregion Piston_Ports

    #pragma region Const_Vals
    inline constexpr bool BASKET_INIT_STATE (false);
    inline constexpr bool INTAKE_INIT_STATE (false);
    inline constexpr bool SCOOP_INIT_STATE (false);
    inline constexpr bool CLAMP_INIT_STATE (false);
    inline constexpr float WHEEL_DIAMETER (3.25);
    inline constexpr int RPM (450);
    inline constexpr float GEAR_RATIO (0.75);
    #pragma endregion Const_Vals
}