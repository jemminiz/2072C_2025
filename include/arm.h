#ifndef ARM_H
#define ARM_H


#include "pros/motors.hpp"
namespace StratusQuo
{
    class Arm
    {
        public: 
            Arm();
            void arm_up();
            void arm_down();
            void stop();
        private:
            pros::Motor motor;
    };
}

#endif