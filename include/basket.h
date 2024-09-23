#ifndef BASKET_H
#define BASKET_H

#include "pros/adi.hpp"
namespace StratusQuo {
    class Basket
    {
        public:
            Basket(uint8_t port, bool start_extended);
            void toggle();
        private:
            pros::adi::Pneumatics basket_pneumatic;
    };
} // namespace StratusQuo
#endif