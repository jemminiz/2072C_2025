#pragma once
#include "robodash/api.h"

inline rd::Selector auton_selector({
    {"First auto", [](){}}
});
inline rd::Console console("Debug");