#pragma once

#include "pros/adi.hpp"
#include <cstdint>
namespace StratusQuo
{
    class Scooper
    {
        public:
            Scooper(uint8_t pneumatic_port, bool starting_config);
            void toggle();
        private:
            pros::adi::Pneumatics piston;
    };
}