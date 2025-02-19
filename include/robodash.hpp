#pragma once
#include "autons.hpp"
#include "robodash/api.h" // IWYU pragma: keep

inline rd::Selector auton_selector({
    {"Drive Example", drive_example},
    {"Turn Example", turn_example}
});
inline rd::Console console("Debug");