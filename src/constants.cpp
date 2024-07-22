#include "constants.h"

float kP = 2.0;
float kI = 0.04;
float kD = 0.0;

pros::Controller master(pros::E_CONTROLLER_MASTER);

std::vector<int8_t> left_motors{LEFT_MOTOR_BACK_PORT, LEFT_MOTOR_MID_PORT, LEFT_MOTOR_FRONT_PORT};
std::vector<int8_t> right_motors{RIGHT_MOTOR_BACK_PORT, RIGHT_MOTOR_MID_PORT, RIGHT_MOTOR_FRONT_PORT};
