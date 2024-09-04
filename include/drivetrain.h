// USE THE INERTIAL SENSOR AS THE SECOND WHEEL

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <cstdint>
#include <vector>
#include "position.h"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/motor_group.hpp"

namespace StratusQuo
{
    class Drivetrain
    {
        public:
            Drivetrain(std::vector<int8_t> left_mg, std::vector<int8_t> right_mg, uint8_t imu, int8_t horizontal, int8_t vertical);
            Drivetrain(const Drivetrain& rhs) = delete;
            Drivetrain& operator=(const Drivetrain& rhs) = delete;

            void drive();
            void driveTo(double heading);
            void setVoltage(int voltage);
            void odom_drive(double x_heading, double y_heading, double orientation);
            void initialize();
            void face_direction(double x, double y);
            double get_rotation();
            double get_heading();
        private:
            pros::MotorGroup left_motor_group;
            pros::MotorGroup right_motor_group;
            pros::Imu imu;
            pros::v5::Rotation horizontal_wheel;
            pros::v5::Rotation vertical_wheel;

            float calc_delta_orientation();
            float get_delta_x();
            float get_delta_y();
            void calc_delta_position();

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
            
            float previous_x_position = 0;
            float previous_y_position = 0;

            Position pos;
    };
}
#endif