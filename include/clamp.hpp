#pragma once

#include "pros/adi.hpp"
namespace StratusQuo
{
    class Clamp
    {
        public:
            Clamp(uint8_t pneumatic_port, bool starting_config);
            void toggle();
            void set_value(bool toggle);
        private:
            pros::adi::Pneumatics piston;
    };
}