#pragma once

void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

namespace StratusQuo
{
    void red_ring_side();
    void blue_ring_side();
    void red_goal_rush();
    void blue_goal_rush();
    void red_goal_side_basic();
    void blue_goal_side_basic();

    void skills();
}