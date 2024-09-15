#ifndef CLAMP_H
#define CLAMP_H

#include "pros/adi.hpp"

namespace StratusQuo
{
    class Clamp
    {
        public: 
            Clamp(pros::adi::DigitalOut p);
            void toggle();
            bool get_value();
            void set_value(bool in);
        private:
            pros::adi::DigitalOut piston;
            bool currentState = false;
    };
}

#endif