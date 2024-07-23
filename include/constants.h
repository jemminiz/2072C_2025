#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "pros.h"
#include <vector>

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

extern pros::Controller master;
extern float kP;
extern float kI ;
extern float kD;

extern std::vector<int8_t> left_motors;
extern std::vector<int8_t> right_motors;
#endif