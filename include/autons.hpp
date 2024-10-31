#pragma once

#include "robodash/views/selector.hpp"

namespace StratusQuo
{
    void sig_red_ring_side();
    void sig_red_goal_side();
    void sig_blue_ring_side();
    void sig_blue_goal_side();
    void local_red_ring_side();
    void local_red_goal_side();
    void local_blue_ring_side();
    void local_blue_goal_side();

    inline rd::Selector::routine_t red_goal("Red Goal Side", StratusQuo::sig_red_goal_side);
    inline rd::Selector::routine_t red_ring("Red Ring Side", StratusQuo::sig_red_ring_side);
    inline rd::Selector::routine_t blue_goal("Blue Goal Side", StratusQuo::sig_blue_goal_side);
    inline rd::Selector::routine_t blue_ring("Blue Ring Side", StratusQuo::sig_blue_ring_side);
    inline rd::Selector auton_selector("Auton Selection", {StratusQuo::red_goal, StratusQuo::red_ring, StratusQuo::blue_goal, StratusQuo::blue_ring});
}