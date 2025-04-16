#include "main.h"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "devices.hpp"
#include "drivercontrol.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "screen.hpp"

bool is_auto = true;
bool manualClick = false;

void auto_clamp_task(void * params) {
  pros::delay(2000);

  while (true)
  {
    if (!backClamped && clampSensor.get() < 20) {
      backClamp.set_value(true);
      backClamped = true;
      pros::delay(250);
    }
    else if (manualClick) {
      backClamped = !backClamped;
      backClamp.set_value(backClamped);
      manualClick = false;
      pros::delay(750);
    }

    pros::delay(20);
  }
}

pros::Task intake_task([]() {
  pros::delay(2000);
  while (true) {
    if (is_color_sort_enabled.load() && backClamped) {
      optical.set_integration_time(5);
      optical.set_led_pwm(100);

      auto color = optical.get_hue();
      if (is_red_team.load() && color > 210 && color < 250) {
        pros::delay(200);
        hooks = -127;
        pros::delay(300);
      }
      else if (!is_red_team.load() && (color < 5 || color > 350)) {
        pros::delay(200);
        hooks = -127;
        pros::delay(300);
      }
      else {
      optical.set_led_pwm(0);}
   }
   if (stopIntake) {
    optical.set_integration_time(5);
    optical.set_led_pwm(100);

    auto color = optical.get_hue();
    if (!is_red_team.load() && color > 205 && color < 255) {
      hooks = -127;
      pros::delay(3);
      hooks = 0;
      hooks.brake();
      hook_voltage.store(0);
    }
    else if (is_red_team.load() && (color < 10 || color > 355)) {
      hooks = -127;
      pros::delay(3);
      hooks = 0;
      hooks.brake();
      hook_voltage.store(0);
    }
  }
    hooks.move(hook_voltage.load());
    rollers.move(roller_voltage.load());

    if (hook_voltage.load() == 0) {
      hooks.brake();
    }
    if (roller_voltage.load() == 0) {
      rollers.brake();
    }
    pros::delay(20);
  }
});


pros::Task wallStakeTask([]() {
  pros::delay(2000);
  int currentPos = 0;
  while (true) {
    if(!is_auto)
    {
      while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        wallStake.move_voltage(12000);
        pros::delay(40);
        wallStake.brake();
      }
      while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        wallStake.move_voltage(-12000);
        pros::delay(40);
        wallStake.brake();
        if(wallStakeLimitSwitch.get_value())
            {
                wallStake.brake();
                liftSensor.reset_position();
                wallStake.tare_position();
                continue;
            }
      }
          if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
          {
            wallStake.move_absolute(127, 200); // LOAD STATE 1!
          }
    }
    else
    {
        wallStake.move(wallStakePID.compute(liftSensor.get_position()));
        pros::delay(50);
    }

  }
});

// Enter your autons here!
AutonFunction autonFunctions[] = {
    {"Goal Rush Blue",  positiveSideSimpleBlue},
    {"Goal Side Blue corner", positiveSideBlue},  
    {"Goal Side Red corner", positiveSideRed},
    {"Solo AWP Red", soloAwpRed},
    {"Solo AWP Blue", soloAwpBlue},
    {"Drive off line",
     []() { chassis.pid_drive_set(-6, 110, false); }}, // Drive off the line!
    {"Easy negative side blue quals", easyNegativeQualsBlue},
    {"Easy negative side red quals", easyNegativeQualsRed},
    {"Negative side blue quals", negativeSideQualsBlue},
    {"Negative side red quals", negativeSideQualsRed},
    {"Negative no alliance stake quals red", negativeNoAllianceStakeQualsRed},
    {"Negative no alliance stake quals blue", negativeNoAllianceStakeQualsBlue},
    {"Negative alliance stake last blue", negativeAllianceStakeLastBlue},
    {"Negative alliance stake last red", negativeAllianceStakeLastRed}
};

// this is needed for LVGL displaying! Do not touch!
size_t autonCount = sizeof(autonFunctions) / sizeof(autonFunctions[0]);

void initialize() {
  pros::delay(750); // Stop the user from doing anything while legacy ports configure.

  // screen init
  calibrationScreenInit();

  calibrateChassis();

  autonSelectorScreenInit(
      // your auton functions
      autonFunctions,
      // auton count needed for LVGL displaying
      autonCount,
      // customizable color scheme, play around with it!
      LV_COLOR_MAKE(0x00, 0xA6, 0xF5)
  );

  chassisInits();

  wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  liftSensor.reset_position();
  wallStakePID.exit_condition_set(500, 0.5, 0, 0, 3000, 0);

  // piston inits
  intakeRaise.set_value(false);
  backClamp.set_value(false);
  leftDoinker.set_value(false);
  rightDoinker.set_value(false);
  pros::Task autoClampes(auto_clamp_task);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field

 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 */
void autonomous() {
  set_drive_to_hold();

  pros::Task auto_clamp_autonomous([]() {
    while(true)
    {
      while(isAutoClamp)
      {
        if(clampSensor.get() < 20 && !backClamped)
        {
          backClamped = true;
          backClamp.set_value(backClamped);
        }
        pros::delay(250);
      }
    }
  });

  runSelectedAuton(autonFunctions, autonCount);
}

/**
 * Runs the operator control code. This function will be started in its own task
 */
void opcontrol() {
  set_drive_to_coast();
  is_auto = false;

  // task to make sure all motors are plugged in and check the temperature of
  // the drivetrain
  pros::Task motorCheck(checkMotorsAndPrintTemperature);
  wallStakeTask.resume();

  while (true) {
    chassis.opcontrol_tank();
    is_color_sort_enabled = false; //DONT FORGET TO CHANGE
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      hook_voltage.store(127);
      roller_voltage.store(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      hook_voltage.store(-127);
      roller_voltage.store(-127);
    } else {
      hook_voltage.store(0);
      roller_voltage.store(0);
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      manualClick = true;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      rightDoinker.set_value(!rightDoinkered);
      rightDoinkered = !rightDoinkered;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      intakeRaise.set_value(intakeRaised);
      intakeRaised = !intakeRaised;
    }

    if(wallStakeLimitSwitch.get_value())
    {
        wallStake.brake();
        wallStake.tare_position();
        liftSensor.reset_position();
    }
    
    /*if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      wallStake.tare_position();
      liftSensor.reset_position(); // Resets position
    }*/

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}