#pragma once

#include "EZ-Template/piston.hpp"
#include "pros/motors.hpp"
#include <cstdint>
#include <atomic>

inline std::atomic_int8_t roller_voltage = 0;
inline std::atomic_int8_t hook_voltage = 0;
inline std::atomic_bool is_red_team = false;
inline bool color_sort_is_enabled = true;

inline void set_intake(std::atomic_int8_t voltage)
{
    roller_voltage.store(voltage);
    hook_voltage.store(voltage);
}

inline void set_hooks(std::atomic_int8_t voltage)
{
    hook_voltage.store(voltage);
}

inline void set_rollers(std::atomic_int8_t voltage)
{
    roller_voltage.store(voltage);
}

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