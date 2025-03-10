#pragma once

#include "EZ-Template/api.hpp" // IWYU pragma: keep
#include "api.h" // IWYU pragma: keep
#include "clamp.hpp"
#include "pros/adi.hpp"

namespace StratusQuo
{
    inline ez::Drive chassis(
    {-4, -7, 18},
    {1, 2, -3},
    20,
    2.75,
    450);

    inline pros::Motor rollers(9);
    inline pros::Motor hooks(8);

    inline pros::Motor lady_brown(10);
    inline const pros::Rotation lady_brown_rotation(16);
    inline ez::PID lady_brown_pid(0.9, 0, 0, 0);
    inline pros::adi::DigitalIn lady_brown_limit_switch('h');

    inline pros::Optical optical(6);
    inline pros::adi::DigitalIn left_limit_switch('c');
    inline pros::adi::DigitalIn right_limit_switch('b');

    inline ez::Piston intake_piston('e');
    inline ez::Piston right_doinker('d');
    inline ez::Piston left_doinker('f');
    
    inline Clamp clamp('a');
}

inline pros::Task lb_task([]() {
    using namespace StratusQuo;
    pros::delay(2000); // Wait for everything to initialize
    while(true)
    {
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
      {
        lady_brown.move(127);
      }
      else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
      {
        if(lady_brown_limit_switch.get_value())
        {
          lady_brown.brake();
          lady_brown_rotation.reset_position();
          continue;
        }
        lady_brown.move(-127);
      }
      else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
      {
        lady_brown.move(lady_brown_pid.compute(lady_brown_rotation.get_position() / 100.0)); // Divide by 100 to account for centidegrees
        // Calculates the voltage to run based on the PID constants and current position
      }
      else lady_brown.brake();
      pros::delay(50);
    }
  });