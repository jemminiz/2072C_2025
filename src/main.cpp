#include "main.h"
#include "autons.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robodash.hpp"
#include "subsystems.hpp"
#include "constants.hpp"
#include <string>

bool is_position_based = false;
bool L1_is_pressed = false;
bool L2_is_pressed = false;
bool R1_is_pressed = false;
bool R2_is_pressed = false;
bool DOWN_is_pressed = false;
int lady_brown_speed = 0;

pros::Task limit_switch_task([]() {
  pros::delay(2000);
  bool changed = false;
  bool curr = set_clamp.load();
  while(true)
  {
    if(((StratusQuo::left_limit_switch.get_new_press() && StratusQuo::right_limit_switch.get_value()) ||
        (StratusQuo::left_limit_switch.get_value() && StratusQuo::right_limit_switch.get_new_press())) && is_auto_clamp_enabled.load())
    {
      set_clamp.store(true);
    }
    curr = set_clamp.load();
    StratusQuo::clamp.set(curr);
    if(curr != changed) pros::delay(1000);
    changed = curr;
    pros::delay(50);
  }
});

pros::Task intake_task([]() {
  pros::delay(2000);
  while(true)
  {
    // Color sort
    if(color_sort_is_enabled)
    {
      StratusQuo::optical.set_integration_time(5);
      StratusQuo::optical.set_led_pwm(100);

      auto color = StratusQuo::optical.get_hue();
      if(is_red_team.load() && color > 190 && color < 250)
      {
        StratusQuo::hooks.move(-127);
        pros::delay(500);
      }
      if(!is_red_team.load() && color < 10)
      {
        StratusQuo::hooks.move(-127);
        pros::delay(500);
      }
    }
    else StratusQuo::optical.set_led_pwm(0);

    StratusQuo::hooks.move(hook_voltage.load());
    StratusQuo::rollers.move(roller_voltage.load());

    if(hook_voltage.load() == 0)
    {
      StratusQuo::hooks.brake();
    }
    if(roller_voltage.load() == 0)
    {
      StratusQuo::rollers.brake();
    }

    pros::delay(20);
  }
});

pros::Task lb_task([]() {
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

pros::Task screen_task([]() {
  pros::delay(2000);
  while(true)
  {
    debug.focus();
    debug.println("Current Position: " + std::to_string(StratusQuo::lady_brown.get_position()));
    pros::delay(75);
    debug.clear();
  }
});

void initialize() {
  pros::delay(500);

  StratusQuo::chassis.opcontrol_curve_buttons_toggle(true);
  StratusQuo::chassis.opcontrol_drive_activebrake_set(0.0);
  StratusQuo::chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  StratusQuo::rollers.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  StratusQuo::hooks.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  StratusQuo::lady_brown.tare_position();
  StratusQuo::lady_brown_rotation.reset_position();
  StratusQuo::lady_brown_pid.exit_condition_set(80, 50, 300, 150, 500, 500);
  StratusQuo::lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  StratusQuo::chassis.pid_tuner_pids.push_back({"Lift", &StratusQuo::lady_brown_pid.constants});

  lb_task.suspend();

  StratusQuo::chassis.drive_imu_calibrate(false);
  StratusQuo::chassis.drive_sensor_reset();
  master.rumble(StratusQuo::chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
  StratusQuo::chassis.pid_targets_reset();
  StratusQuo::chassis.drive_imu_reset();
  StratusQuo::chassis.drive_sensor_reset();
  StratusQuo::chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  StratusQuo::chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  color_sort_is_enabled = true;

  if(auton_selector.get_auton()) auton_selector.run_auton();
  else 
  {
    StratusQuo::skills();//StratusQuo::chassis.pid_drive_set(-6_in, 110); 
  }
}

void pid_tuner() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      StratusQuo::chassis.pid_tuner_toggle();

    if (master.get_digital(DIGITAL_B)) {
      pros::motor_brake_mode_e_t preference = StratusQuo::chassis.drive_brake_get();
      autonomous();
      StratusQuo::chassis.drive_brake_set(preference);
    }

    StratusQuo::chassis.pid_tuner_iterate();
  }

  else {
    if (StratusQuo::chassis.pid_tuner_enabled())
      StratusQuo::chassis.pid_tuner_disable();
  }
}

void opcontrol() {
  using namespace StratusQuo;
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  lb_task.resume();
  lady_brown_pid.target_set(22); // Set target to load position

  while (true) {
    color_sort_is_enabled = false;
    L1_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    L2_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    R1_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    R2_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    DOWN_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    pid_tuner();

    chassis.opcontrol_tank();
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
      hook_voltage.store(127);
      roller_voltage.store(127);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
      hook_voltage.store(-127);
      roller_voltage.store(-127);
    }
    else set_intake(0);

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      set_clamp.store(!StratusQuo::clamp.get_state());
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
      right_doinker.set(!right_doinker.get());
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
      intake_piston.set(!intake_piston.get());
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
      lady_brown.tare_position();
      lady_brown_rotation.reset_position(); // Resets position
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}