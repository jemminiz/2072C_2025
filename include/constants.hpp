#pragma once

#include <cstdint>
#include <atomic>

inline std::atomic<bool> is_auto_clamp_enabled(true);
inline std::atomic<bool> is_red_team(false);
inline std::atomic<bool> color_sort_is_enabled(true);
inline std::atomic<uint8_t> hook_voltage(0);
inline std::atomic<uint8_t> roller_voltage(0);

inline void set_intake(uint8_t voltage)
{
    hook_voltage.store(voltage);
    roller_voltage.store(voltage);
}