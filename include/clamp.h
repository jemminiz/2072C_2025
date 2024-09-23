#ifndef CLAMP_H
#define CLAMP_H

#include "pros/adi.hpp"

namespace StratusQuo
{
    class Clamp
    {
        public: 
            Clamp(uint8_t clamp_port);
            void toggle();
            bool get_value();
            void set_value(bool in);
        private:
            pros::adi::Pneumatics piston;
            bool currentState = false;
    };
}

#endif