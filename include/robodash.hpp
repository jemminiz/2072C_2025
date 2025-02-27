#pragma once
#include "autons.hpp"
#include "robodash/api.h" // IWYU pragma: keep

inline rd::Selector auton_selector({
    {"Red Neg", StratusQuo::blue_goal_NATE},
    {"Red Neg", StratusQuo::red_ring_side},
    {"Red Pos", StratusQuo::red_goal_rush},
    {"Blue Neg", StratusQuo::blue_ring_side},
    {"Blue Pos", StratusQuo::blue_goal_rush},
    {"Red Pos Basic", StratusQuo::red_goal_side_basic},
    {"Blue Pos Basic", StratusQuo::blue_goal_side_basic}
});
inline rd::Console debug("Debug");