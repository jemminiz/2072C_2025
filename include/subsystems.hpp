#pragma once

#include "EZ-Template/api.hpp" // IWYU pragma: keep
#include "api.h" // IWYU pragma: keep
#include "intake.hpp"
#include "clamp.hpp"
#include "constants.hpp"
#include "lady_brown.hpp"
#include "pros/adi.hpp"

namespace StratusQuo
{
    inline ez::Drive chassis(
    {-4, -7, 18},
    {1, 2, -3},
    20,
    2.75,
    450);

    inline Intake intake(ROLLER_PORT, HOOK_PORT, INTAKE_PISTON_PORT);
    inline Clamp clamp(CLAMP_PORT);
    inline Lady_Brown lady_brown(LADY_BROWN_PORT, LADY_BROWN_PID, LADY_BROWN_ROTATION);
    inline pros::adi::DigitalIn left_limit_switch(LEFT_LIMIT_SWITCH_PORT);
    inline pros::adi::DigitalIn right_limit_switch(RIGHT_LIMIT_SWITCH_PORT);
    inline pros::Optical optical(6);

    inline ez::Piston right_doinker(DOINKER_PORT);
    inline ez::Piston left_doinker(RIGHT_DOINKER_PORT);
}