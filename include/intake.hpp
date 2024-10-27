#pragma once

#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include <cstdint>
namespace StratusQuo
{
    class Intake
    {
        public:
            Intake(int motor_port, uint8_t pneumatic_port, bool starting_config);
            void intake();
            void outtake();
            void move(int voltage);
            void toggle();
            void set_value(bool toggle);
            void brake();
        private:
            pros::Motor motor;
            pros::adi::Pneumatics piston;
    };
}