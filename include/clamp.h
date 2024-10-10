#pragma once 
#include <cstdint>
#include "EZ-Template/PID.hpp"
#include "pros/adi.hpp"
namespace StratusQuo
{
    class Clamp
    {
        public:
            Clamp(uint8_t pneumatic_port
            );
            void clamp();
            void clamp_auton(bool toggle);

            void set_constants(double kP, double kI, double kD);         
        private:
            pros::adi::Pneumatics clamp_pneumatics;
    };
}