#ifndef INTAKE_H
#define INTAKE_H


#include "pros/adi.hpp"
#include "pros/motors.hpp"
namespace StratusQuo
{
    class Intake
    {
        public: 
            Intake(uint8_t motor_port, uint8_t piston_port, bool starting_piston_state);
            void intake();
            void outtake();
            void stop();
            void toggle();
        private:
            pros::Motor motor;
            pros::adi::Pneumatics piston;
    };
}

#endif