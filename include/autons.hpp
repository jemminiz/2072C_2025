#pragma once

#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void combining_movements();
void interfered_example();

void sig_red_ring_side();
void sig_red_goal_side();
void sig_blue_ring_side();
void sig_blue_goal_side();
void local_red_ring_side();
void local_red_goal_side();
void local_blue_ring_side();
void local_blue_goal_side();

void default_constants();
void one_mogo_constants();
void two_mogo_constants();