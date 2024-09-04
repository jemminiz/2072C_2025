#ifndef INTAKE_H
#define INTAKE_H


#include "pros/adi.hpp"
#include "pros/motors.hpp"
namespace StratusQuo
{
    class Intake
    {
        public: 
            Intake();
            void intake();
            void outtake();
            void move();
        private:
            pros::Motor motor;
            pros::adi::DigitalOut piston;
            bool current_piston_state;
    };
}

#endif