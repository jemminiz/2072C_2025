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
            void clamp_setvalue(bool toggle);
        private:
            pros::adi::Pneumatics clamp_pneumatics;
    };
}