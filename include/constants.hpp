#pragma once

#include "EZ-Template/PID.hpp"
#include <cstdint>
#include <atomic>

inline const int ROLLER_PORT = 10;
inline const int HOOK_PORT = 11;
inline const int LADY_BROWN_PORT = 12;
inline const uint8_t INTAKE_PISTON_PORT = 'a';
inline const uint8_t CLAMP_PORT = 'b';
inline const uint8_t LEFT_LIMIT_SWITCH_PORT = 'c';
inline const uint8_t RIGHT_LIMIT_SWITCH_PORT = 'd';
inline const ez::PID LADY_BROWN_PID(3.0, 0, 10.0);
inline const pros::Rotation LADY_BROWN_ROTATION(-20);

inline std::atomic<bool> is_auto_clamp_enabled(true);