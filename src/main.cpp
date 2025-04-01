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

pros::Task auto_clamp_task([]() {
  pros::delay(2000);
  bool curr = set_clamp.load();
  bool changed;
  /*
  while(true) // Limit switch code
  {
      // One gets a new press and the other is being pressed
      curr = ((backClampLeftLimitSwitch.get_new_press() &&
  backClampRightLimitSwitch.get_value()) ||
  (backClampLeftLimitSwitch.get_value() &&
  backClampRightLimitSwitch.get_new_press()) && is_auto_clamp_enabled.load());
      backClamp.set_value(curr);
      if(changed && !curr) pros::delay(1000); // Delay 1 second after release
      pros::delay(20);
      changed = curr;
  } */
  while (true) // Distance sensor code
  {
    backClamp.set_value(set_clamp.load());
  }
});

pros::Task intake_task([]() {
  pros::delay(2000);
  while (true) {
    if (is_color_sort_enabled.load()) {
      optical.set_integration_time(5);
      optical.set_led_pwm(100);

      auto color = optical.get_hue();
      if (is_red_team.load() && color > 190 && color < 250) {
        hooks = -127;
        pros::delay(500);
      }
      if (!is_red_team.load() && color < 10) {
        hooks = -127;
        pros::delay(500);
      }
    } else
      optical.set_led_pwm(0);

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

void wallStake_control() {
    if(is_auto) {
        return;
    } else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        wallStakePID.target_set(2820);

    }
}
pros::Task wallStakeTask([]() {
  pros::delay(2000);
  int currentPos = 0;
  while (true) {
    if(!is_auto)
    {
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            wallStake.move_voltage(12000);
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            if(wallStakeLimitSwitch.get_value())
            {
                wallStake.brake();
                liftSensor.reset_position();
                continue;
            }
            wallStake.move_voltage(-12000);
        }
        else
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            wallStakePID.target_set(1980);
            do
            {
                wallStake.move(wallStakePID.compute(liftSensor.get_position())); 
                pros::delay(ez::util::DELAY_TIME);
            } while(wallStakePID.exit_condition(wallStake) == ez::RUNNING);
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            wallStakePID.target_set(3800);
            do
            {
                wallStake.move(wallStakePID.compute(liftSensor.get_position())); 
                pros::delay(ez::util::DELAY_TIME);
            } while (wallStakePID.exit_condition(wallStake) == ez::RUNNING);
        }
        else wallStake.brake();
        pros::delay(50);
    }
    else
    {
        wallStake.move(wallStakePID.compute(liftSensor.get_position()));
        pros::delay(50);
    }

  }
});

pros::Task screenTask([]() {
  while (true) {
    printf("Computed Output Value: %.2f\n",
           wallStakePID.compute(liftSensor.get_position()));
    printf("Sensor Value: %d\n", liftSensor.get_position());
    printf("Target Value: %.2f\n", wallStakePID.target_get());
    printf("ErrorValue: %.2f\n",
           (liftSensor.get_position()) - wallStakePID.target_get());
    pros::delay(500);
  }
});

// Enter your autons here!
AutonFunction autonFunctions[] = {
    {"Solo AWP Red", soloAwpRed},
    {"Solo AWP Blue", soloAwpBlue},
    {"Drive off line",
     []() { chassis.pid_drive_set(-6, 110, false); }}, // Drive off the line!
    {"Negative side blue quals", negativeSideQualsBlue},
    {"Negative side red quals", negativeSideQualsRed},
};

// this is needed for LVGL displaying! Do not touch!
size_t autonCount = sizeof(autonFunctions) / sizeof(autonFunctions[0]);

void initialize() {
  pros::delay(
      750); // Stop the user from doing anything while legacy ports configure.

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
  set_clamp.store(false);
  leftDoinker.set_value(false);
  rightDoinker.set_value(false);
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
  // pros::Task motorCheck(checkMotorsAndPrintTemperature);
  wallStakeTask.resume();

  bool backClamped = set_clamp.load();

  while (true) {
    chassis.opcontrol_tank();
    is_color_sort_enabled = false;
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

    int dist = clampSensor.get();
    if(dist < 20 && !backClamped)
    {
      set_clamp.store(true);
    }
    else if(backClamped && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
      set_clamp.store(false);
    }
    backClamped = set_clamp.load();

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      rightDoinker.set_value(!rightDoinkered);
      rightDoinkered = !rightDoinkered;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      intakeRaise.set_value(intakeRaised);
      intakeRaised = !intakeRaised;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      wallStake.tare_position();
      liftSensor.reset_position(); // Resets position
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}