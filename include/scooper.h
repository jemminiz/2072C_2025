#ifndef SCOOPER_H
#define SCOOPER_H

#include "pros/adi.hpp"
#include <cstdint>

namespace StratusQuo
{
    class Scooper
    {
        public:
            Scooper(int8_t port);
            void toggle();
        private:
            pros::adi::DigitalOut scooper;
            bool state = false;
    };
}
#endif