#ifndef SCOOPER_H
#define SCOOPER_H

#include "pros/adi.hpp"
#include <cstdint>

namespace StratusQuo
{
    class Scooper
    {
        public:
            Scooper(int8_t port, bool start_extended);
            void toggle();
        private:
            pros::adi::Pneumatics scooper;
            bool state = false;
    };
}
#endif