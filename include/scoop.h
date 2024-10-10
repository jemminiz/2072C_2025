#pragma once 
#include <cstdint>
#include "EZ-Template/PID.hpp"
#include "pros/adi.hpp"
namespace StratusQuo
{
    class Scoop
    {
        public:
            Scoop(uint8_t pneumatic_port
            );
            void scoop();
            void scoop_setvalue(bool toggle);
      
        private:
            pros::adi::Pneumatics scoop_pneumatics;
    };
}