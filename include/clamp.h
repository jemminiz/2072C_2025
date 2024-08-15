#ifndef CLAMP_H
#define CLAMP_H

#include "pros.h"
#include "pros/adi.h"
#include "pros/adi.hpp"

namespace StratusQuo
{
    class Clamp
    {
        public: 
            Clamp(pros::adi::DigitalOut p);
            void toggle();
        private:
            pros::adi::DigitalOut piston;
            bool currentState = false;
    };
}

#endif