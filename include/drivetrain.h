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
            Drivetrain(std::vector<int8_t> left_mg, std::vector<int8_t> right_mg, std::optional<int> imu = std::nullopt);
            Drivetrain(const Drivetrain& rhs) = delete;
            Drivetrain& operator=(const Drivetrain& rhs) = delete;

            void drive();
            double get_heading();
            void driveTo(double heading);
            void setVoltage(int voltage);
        private:
            pros::MotorGroup left_motor_group;
            pros::MotorGroup right_motor_group;
            pros::Imu imu;

            float pidSensorCurrentValue = 0.f;

            float pidError = 0.f;
            float pidLastError = 0.f;
            float pidDerivative = 0.f;
            float pidIntegral = 0.f;
            float pidDrive = 0.f;
    };
}
#endif