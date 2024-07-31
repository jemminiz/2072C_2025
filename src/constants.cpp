#include "constants.h"

pros::Controller StratusQuo::master(pros::E_CONTROLLER_MASTER);

std::vector<int8_t> StratusQuo::left_motors{StratusQuo::LEFT_MOTOR_BACK_PORT, StratusQuo::LEFT_MOTOR_MID_PORT, StratusQuo::LEFT_MOTOR_FRONT_PORT};
std::vector<int8_t> StratusQuo::right_motors{StratusQuo::RIGHT_MOTOR_BACK_PORT, StratusQuo::RIGHT_MOTOR_MID_PORT, StratusQuo::RIGHT_MOTOR_FRONT_PORT};

StratusQuo::Drivetrain StratusQuo::dt (StratusQuo::left_motors, StratusQuo::right_motors, StratusQuo::IMU_PORT);