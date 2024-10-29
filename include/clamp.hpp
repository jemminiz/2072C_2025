#pragma once

#include "pros/adi.hpp"
#include "constants.hpp"
namespace StratusQuo
{
    class Clamp
    {
        public:
            Clamp(uint8_t pneumatic_port, bool starting_config);
            void toggle();
            void extend();
            void retract();
        private:
            pros::adi::Pneumatics piston;
            bool current_val = CLAMP_INIT_STATE;
    };
}