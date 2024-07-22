#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <cstdint>
#include <optional>
#include <vector>
#include "pros.h"

class Drivetrain {
    public:
        Drivetrain(std::vector<int8_t> left_mg, std::vector<int8_t> right_mg, std::optional<int> imu = std::nullopt);
        void drive();
        double get_heading();
        void setVoltage(int voltage);
    private:
        pros::MotorGroup left_motor_group;
        pros::MotorGroup right_motor_group;
        pros::Imu imu;
};
#endif