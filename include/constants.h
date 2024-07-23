#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "pros.h"
#include <vector>

#define LEFT_MOTOR_BACK_PORT 1
#define LEFT_MOTOR_MID_PORT 1
#define LEFT_MOTOR_FRONT_PORT 1
#define RIGHT_MOTOR_BACK_PORT 1
#define RIGHT_MOTOR_MID_PORT 1
#define RIGHT_MOTOR_FRONT_PORT 1

#define IMU_PORT 0

#define PID_DRIVE_MAX 127
#define PID_DRIVE_MIN (-127)

inline constexpr int PID_INTEGRAL_LIMIT (50);

extern pros::Controller master;
extern float kP;
extern float kI ;
extern float kD;

extern std::vector<int8_t> left_motors;
extern std::vector<int8_t> right_motors;
#endif