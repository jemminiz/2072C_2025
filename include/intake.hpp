#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include <cstdint>
namespace StratusQuo
{
    class Intake
    {
        public:
            Intake(int motor_port, uint8_t pneumatic_port, bool init_state);
            void toggle();
            void move(int voltage);
            void brake();
        private:
            pros::Motor motor;
            pros::ADIDigitalOut pneumatic;
            bool state;
    };
}