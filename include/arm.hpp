#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include <cstdint>
namespace StratusQuo
{
    class Arm
    {
        public:
            Arm(int motor_port, uint8_t pneumatic_port, bool starting_config);
            
            void toggle();
            void up();
            void down();
            void move(int voltage);
            void brake();
        private:
            pros::Motor motor;
            pros::adi::Pneumatics piston;
    };
}