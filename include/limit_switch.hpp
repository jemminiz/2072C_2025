#pragma once
#include "pros/adi.hpp"
namespace StratusQuo
{
    class Limit_Switch
    {
        public:
            Limit_Switch(uint8_t port);
            bool get_new_press();
            int limit_switch_task();
        private:
            pros::adi::DigitalIn limit_switch;
    };
}