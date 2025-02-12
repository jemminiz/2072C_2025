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
    {1, 2, 3},
    {4, 5, 6},
    7,
    2.75,
    450);

    inline Intake intake(ROLLER_PORT, HOOK_PORT, INTAKE_PISTON_PORT);
    inline Clamp clamp(CLAMP_PORT);
    inline Lady_Brown lady_brown(LADY_BROWN_PORT, LADY_BROWN_PID, LADY_BROWN_ROTATION);
    inline pros::adi::DigitalIn left_limit_switch(LEFT_LIMIT_SWITCH_PORT);
    inline pros::adi::DigitalIn right_limit_switch(RIGHT_LIMIT_SWITCH_PORT);
}