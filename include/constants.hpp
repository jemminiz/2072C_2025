#pragma once

#include "pros/rotation.hpp"
#include <cstdint>
#include <atomic>

inline const int ROLLER_PORT = 9;
inline const int HOOK_PORT = 8;
inline const int LADY_BROWN_PORT = 10;

inline const uint8_t INTAKE_PISTON_PORT = 'b';
inline const uint8_t CLAMP_PORT = 'a';
inline const uint8_t LEFT_LIMIT_SWITCH_PORT = 'c';
inline const uint8_t RIGHT_LIMIT_SWITCH_PORT = 'd';

inline const pros::Rotation LADY_BROWN_ROTATION(-21);

inline std::atomic<bool> is_auto_clamp_enabled(true);