#include "constants.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

std::vector<int8_t> left_motors{StratusQuo::LEFT_MOTOR_BACK_PORT, StratusQuo::LEFT_MOTOR_MID_PORT, StratusQuo::LEFT_MOTOR_FRONT_PORT};
std::vector<int8_t> right_motors{StratusQuo::RIGHT_MOTOR_BACK_PORT, StratusQuo::RIGHT_MOTOR_MID_PORT, StratusQuo::RIGHT_MOTOR_FRONT_PORT};

StratusQuo::Drivetrain dt (left_motors, right_motors, StratusQuo::IMU_PORT);