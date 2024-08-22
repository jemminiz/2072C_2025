#ifndef ARM_H
#define ARM_H

#include "pros.h"

namespace StratusQuo
{
    class Arm
    {
        public: 
            Arm();
            void arm_up();
            void arm_down();
        private:
            pros::Motor motor1;
            pros::Motor motor2;
    };
}

#endif