#pragma once

#include "pros/adi.hpp"
#include <cstdint>
namespace StratusQuo
{
    class Scooper
    {
        public:
            Scooper(uint8_t port, bool init_state);
            void toggle();
        private:
            pros::ADIDigitalOut scoop;
            bool state = false;
    };
}