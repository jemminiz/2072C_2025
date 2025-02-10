#pragma once

#include "EZ-Template/piston.hpp"
#include "pros/motors.hpp"
#include <cstdint>
namespace StratusQuo
{
    class Intake
    {
        public:
            Intake() = delete;
            Intake(uint8_t roller_port, uint8_t hook_port, uint8_t piston_port);

            int move(int8_t voltage);
            int move_rollers(int8_t voltage);
            int move_hooks(int8_t voltage);
            int toggle_piston();
            int button_toggle(int button);
            int brake();
        private:
            pros::Motor _roller_motor;
            pros::Motor _hook_motor;
            ez::Piston _piston;
    };
}