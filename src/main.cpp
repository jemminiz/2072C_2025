#include "main.h"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "devices.hpp"
#include "drivercontrol.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "screen.hpp"

bool is_auto = true;
std::atomic<bool> manualClick = false;

pros::Task auto_clamp_task([]() {
  pros::delay(2000);

  while (true) {
    if (!is_auto) {
      // Driver control mode
      backClamped = set_clamp.load();
      if (!backClamped && clampSensor.get() < 20) {
        backClamp.set_value(true);
        set_clamp.store(true);
        pros::delay(250);
      } else if (manualClick) {
        bool newState = !set_clamp.load();
        set_clamp.store(newState);
        backClamp.set_value(newState);
        manualClick = false;
        pros::delay(750);
      }
    } else if (isAutoClamp) {
      // Autonomous mode
      if (clampSensor.get() < 20 && !backClamped) {
        backClamped = true;
        set_clamp.store(true);
        backClamp.set_value(true);
        pros::delay(250);
      }
    }

    pros::delay(20);
  }
});

pros::Task color_sort_task([]() {
  pros::delay(2000);
  const int PROX_THRESHOLD_ENTER = 50;
  const int PROX_THRESHOLD_EXIT = 120;
  bool prevRingState = false;
  bool ringDetected = false;
  bool throwing = false;
  uint32_t throwStartTime = 0;
  optical.set_integration_time(10);
  optical.set_led_pwm(100);
  while (true) {
    bool currentRingState = false;
    if (is_color_sort_enabled) {
      if (is_red_team.load()) // Red team - throw blue
      {
        currentRingState = (optical.get_proximity() > PROX_THRESHOLD_ENTER &&
                                optical.get_hue() <= 16 ||
                            optical.get_hue() >= 280);
      } else if (!is_red_team.load()) // Blue team - throw red
      {
        currentRingState =
            (optical.get_proximity() > PROX_THRESHOLD_ENTER &&
             optical.get_hue() >= 100 && optical.get_hue() <= 250);
      }

      if (currentRingState && !prevRingState) {
        ringDetected = true;
      } else if (ringDetected &&
                 optical.get_proximity() < PROX_THRESHOLD_EXIT) {
        hooks.move(-127);
        throwStartTime = pros::millis();
        throwing = true;
        ringDetected = false;
      }
      prevRingState = currentRingState;

      // stop throwing after 200ms
      if (throwing && (pros::millis() - throwStartTime >= 200)) {
        hooks.move(127);
        rollers.move(127);
        throwing = false;
      }
    }
    if (stopIntake) {
      if (is_red_team.load() &&
          (optical.get_hue() >= 100 && optical.get_hue() <= 250)) {
        intake.brake();
      } else if (!is_red_team.load() &&
                 (optical.get_hue() <= 16 || optical.get_hue() >= 280)) {
        intake.brake();
      }
    }
    pros::delay(10);
  }
});

pros::Task wallStakeTask([]() {
  pros::delay(2000);
  int currentPos = 0;
  enum WallStakePositions {
    ZERO = 0,
    LOAD = 110,
    SCORE = 250,
    UNTIP = 330,
    GRAB = 400
  };
  while (true) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      currentPos += 1;
      currentPos %= 3;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      currentPos = (currentPos + 3 - 1) % 3;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      currentPos = 3;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      currentPos = 4;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      // zero lb
      while (!wallStakeLimitSwitch.get_value()) {
        wallStake.move_voltage(-12000);
      }
    }

    switch (currentPos) {
    case 0:
      wallStake.move_absolute(ZERO, 150);
      break;
    case 1:
      wallStake.move_absolute(LOAD, 150);
      break;
    case 2:
      wallStake.move_absolute(SCORE, 150);
      break;
    case 3:
      wallStake.move_absolute(UNTIP, 150);
      break;
    case 4:
      wallStake.move_absolute(GRAB, 150);
      break;
    }
  }
});

// Enter your autons here!
AutonFunction autonFunctions[] = {
    {"+ Goal Rush Blue", positiveSideSimpleBlue},
    {"+ 4 Ring Blue", positiveSideBlue},
    {"+ 4 Ring Red", positiveSideRed},
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
    {"- Elims blue", negativeAllianceStakeLastBlue},
    {"- Elims red", negativeAllianceStakeLastRed},
    {"- Ring Rush Red", negativeRingRushRed},
    {"- Ring Rush Blue", negativeRingRushBlue}};

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
      LV_COLOR_MAKE(0x00, 0xA6, 0xF5));

  chassisInits();

  wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  liftSensor.reset_position();
  wallStakePID.exit_condition_set(500, 0.5, 0, 0, 3000, 0);

  // piston inits
  intakeRaise.set_value(false);
  backClamp.set_value(false);
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
  is_auto = true;
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
  color_sort_task.suspend();
  wallStakeTask.resume();

  while (true) {
    chassis.opcontrol_tank();
    is_color_sort_enabled = false;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move(-127);
    }

    manualClick.store(
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT));

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      rightDoinker.set_value(!rightDoinkered);
      rightDoinkered = !rightDoinkered;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      intakeRaise.set_value(intakeRaised);
      intakeRaised = !intakeRaised;
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}