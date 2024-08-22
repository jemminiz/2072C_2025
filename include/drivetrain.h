// USE THE INERTIAL SENSOR AS THE SECOND WHEEL

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <cstdint>
#include <optional>
#include <vector>
#include "pros.h"

namespace StratusQuo
{
    class Drivetrain
    {
        public:
            Drivetrain(std::vector<int8_t> left_mg, std::vector<int8_t> right_mg, int imu, int8_t horizontal, int8_t vertical);
            Drivetrain(const Drivetrain& rhs) = delete;
            Drivetrain& operator=(const Drivetrain& rhs) = delete;

            void drive();
            double get_heading();
            void driveTo(double heading);
            void setVoltage(int voltage);
            void odom_drive(double heading, double orientation);
        private:
            pros::MotorGroup left_motor_group;
            pros::MotorGroup right_motor_group;
            pros::Imu imu;
            pros::v5::Rotation horizontal_wheel;
            pros::v5::Rotation vertical_wheel;

            float calc_delta_orientation();

            float pidSensorCurrentValue = 0.f;

            float pidError = 0.f;
            float pidLastError = 0.f;
            float pidDerivative = 0.f;
            float pidIntegral = 0.f;
            float pidDrive = 0.f;

            float horizontal_wheel_distance;
            float vertical_wheel_distance;
            float imu_distance;
            float previous_global_position;
            float previous_global_orientation;
            float global_orientation_at_reset;
            float deltaOrientation;
    };
}
#endif