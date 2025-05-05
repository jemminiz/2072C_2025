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
RingState ringState = IDLE;

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

enum RingColors { NONE, RED, BLUE };

pros::Task color_sort_task([]() {
  pros::delay(2000);
  RingColors currentRing = NONE;
  RingState ringState = IDLE;
  bool currentRingState = false;
  bool prevRingState = false;
  uint32_t throwStartTime = 0;
  uint32_t detectedStartTime = 0;
  bool throwing = false;
  optical.set_led_pwm(100);
  optical.set_integration_time(5);
  while (true) {
    if (is_color_sort_enabled.load()) {
      // 1. Detect if a bad ring is seen based on hue
      if ((optical.get_hue() <= 20 || optical.get_hue() >= 280))
        currentRing = RED;
      else if (optical.get_hue() >= 200 && optical.get_hue() <= 250)
        currentRing = BLUE;
      if (!is_red_team.load()) {
        currentRingState = (currentRing == RED);
      } else {
        currentRingState = (currentRing == BLUE);
      }

      // 2. State machine
      switch (ringState) {
      case IDLE:
        if (currentRingState) {
          detectedStartTime = pros::millis();
          ringState = DETECTED;
        } else if (stopIntake && !currentRingState) {
          ringState = DETECTED;
        }
        break;

      case DETECTED:
        if (currentRingState) {
          if (pros::millis() - detectedStartTime >= 50) {
            ringState = CONFIRMED;
          }
        } else if (stopIntake && !currentRingState) {
          ringState = CONFIRMED;
        } else {
          ringState = IDLE;
        }
        break;

      case CONFIRMED:
        if (currentRingState) {
          // Wait for physical contact
          if (intakeLimitSwitch.get_value()) {
            ringState = READY_TO_LAUNCH;
          }
          // Timeout fallback
          else if (pros::millis() - detectedStartTime > 500) {
            ringState = IDLE;
          }
        } else if (stopIntake) {
          if (intakeLimitSwitch.get_value()) {
            intake = 0;
            ringState = IDLE;
          }
        }
        break;

      case READY_TO_LAUNCH:
        pros::delay(100);
        hooks.move(-127);
        throwStartTime = pros::millis();
        throwing = true;
        ringState = IDLE;
        break;
      }

      prevRingState = currentRingState;

      // Stop hooks after 200ms
      if (throwing && (pros::millis() - throwStartTime >= 200)) {
        hooks.move(0);
        throwing = false;
      }
    }
    pros::delay(20);
  }
});

pros::Task wallStakeTask([]() {
  pros::delay(2000);
  currentPos = 0;
  enum WallStakePositions {
    ZERO = 0,
    LOAD = 130,
    SCORE = 480,
    UNTIP = 900,
    GRAB = 600
  };
  while (true) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      currentPos += 1;
      currentPos %= 3;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      currentPos = (currentPos + 3 - 1) % 3;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      currentPos = 3;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      currentPos = 4;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      // zero lb
      wallStake.move_voltage(-6000); // slower approach
      while (!wallStakeLimitSwitch.get_value())
        pros::delay(10);
      wallStake.move_voltage(0);
      wallStake.tare_position(); // zero out the encoder
      currentPos = 0;
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
    pros::delay(20);
  }
});

// Enter your autons here!
AutonFunction autonFunctions[] = {
    {"+ Goal Rush Blue", positiveSideSimpleBlue},
    {"Red Ring rush solo awp", soloAwpSafeRed},
    {"+ 4 Ring Blue", positiveSideBlue},
    {"+ 4 Ring Red", positiveSideRed},
    {"Solo AWP Red", soloAwpRed},
    {"Solo AWP Blue", soloAwpBlue},
    {"Drive off line",
     []() { chassis.pid_drive_set(-6, 110, false); }}, // Drive off the line!
    {"Universal Blue Left", universalBlueLeft},        // Universal Blue
    {"Universal Blue Right", universalBlueRight},      // Universal Blue
    {"Universal Red Left", universalRedLeft},          // Universal Red
    {"Universal Red Right", universalRedRight},        // Universal Red
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

  pros::Task motorCheck(checkMotorsAndPrintTemperature);
  color_sort_task.suspend();
  wallStakeTask.resume();

  RingState ringState = IDLE;
  bool currentRingState = false;
  bool prevRingState = false;
  uint32_t throwStartTime = 0;
  uint32_t detectedStartTime = 0;
  bool throwing = false;

  optical.set_integration_time(5);

  while (true) {
    chassis.opcontrol_tank();

    if (is_color_sort_enabled.load()) {
      // 1. Detect if a bad ring is seen based on hue
      if (!is_red_team.load()) {
        currentRingState =
            (optical.get_hue() <= 20 || optical.get_hue() >= 280);
      } else {
        currentRingState =
            (optical.get_hue() >= 200 && optical.get_hue() <= 250);
      }

      // 2. State machine
      switch (ringState) {
      case IDLE:
        if (currentRingState) {
          detectedStartTime = pros::millis();
          ringState = DETECTED;
        }
        break;

      case DETECTED:
        if (currentRingState) {
          if (pros::millis() - detectedStartTime >= 50) {
            ringState = CONFIRMED;
          }
        } else {
          ringState = IDLE;
        }
        break;

      case CONFIRMED:
        // Wait for physical contact
        if (intakeLimitSwitch.get_value()) {
          ringState = READY_TO_LAUNCH;
        }
        // Timeout fallback
        else if (pros::millis() - detectedStartTime > 500) {
          ringState = IDLE;
        }
        break;

      case READY_TO_LAUNCH:
        pros::delay(100);
        hooks.move(-127);
        throwStartTime = pros::millis();
        throwing = true;
        ringState = IDLE;
        break;
      }

      prevRingState = currentRingState;

      // Stop hooks after 200ms
      if (throwing && (pros::millis() - throwStartTime >= 200)) {
        hooks.move(0);
        throwing = false;
      }
    }

    // Intake control
    if (!throwing) {
      if (is_color_sort_enabled.load()) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
            !master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
          intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
          intake.move(-127);
          is_color_sort_enabled.store(false);
        } else if ((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
                    master.get_digital_new_press(
                        pros::E_CONTROLLER_DIGITAL_R2)) ||
                   (master.get_digital_new_press(
                        pros::E_CONTROLLER_DIGITAL_R2) &&
                    master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))) {
          is_color_sort_enabled.store(!is_color_sort_enabled.load());
        } else if (ringState == IDLE) {
          intake.move(0);
        }
      } else {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
          intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
          intake.move(-127);
        } else {
          intake.move(0);
        }
      }
    }

    // Manual controls
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

    pros::delay(ez::util::DELAY_TIME);
  }
}
