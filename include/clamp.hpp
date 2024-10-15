#pragma once

#include "pros/adi.hpp"
namespace StratusQuo
{
    class Clamp
    {
        public:
            Clamp(uint8_t clamp_port, bool init_state);
            void toggle();
            void set_value(bool toggle);
            bool get_state();
        private:
            pros::ADIDigitalOut clamp;
            bool state = false;
    };
}