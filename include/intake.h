#ifndef INTAKE_H
#define INTAKE_H

#include "pros.h"

namespace StratusQuo
{
    class Intake
    {
        public: 
            Intake();
            void intake();
            void outtake();
        private:
            pros::Motor motor;
    };
}

#endif