#include "main.h"
#include "autons.hpp"
#include "lady_brown.hpp"
#include "pros/misc.h"
#include "robodash.hpp"
#include "subsystems.hpp"

bool is_position_based = false;
bool color_sort_is_enabled = true;
bool L1_is_pressed = false;
bool L2_is_pressed = false;
bool R1_is_pressed = false;
bool R2_is_pressed = false;
bool DOWN_is_pressed = false;
int lady_brown_speed = 0;

pros::Task limit_switch_task([]() {
  pros::delay(2000);
  bool changed = false;
  while(true)
  {
    if(((StratusQuo::left_limit_switch.get_new_press() && StratusQuo::right_limit_switch.get_value()) ||
        (StratusQuo::left_limit_switch.get_value() && StratusQuo::right_limit_switch.get_new_press())) && is_auto_clamp_enabled.load())
    {
      set_clamp.store(true);
    }
    StratusQuo::clamp.set(set_clamp.load());
    if(set_clamp.load() != changed) pros::delay(1000);
    changed = set_clamp.load();
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
        StratusQuo::intake.move_hooks(-127);
        pros::delay(500);
      }
      if(!is_red_team.load() && color < 10)
      {
        StratusQuo::intake.move_hooks(-127);
        pros::delay(500);
      }
    }

    StratusQuo::intake.move_hooks(hook_voltage.load());
    StratusQuo::intake.move_rollers(roller_voltage.load());
    pros::delay(20);
  }
});

pros::Task lady_brown_task([]() {
  pros::delay(2000);
  while(true)
  {
    double currPose = StratusQuo::lady_brown.get_position()/100.0;
    if(DOWN_is_pressed)
    {
      StratusQuo::lady_brown.move_to(1);
    }
    else if(L1_is_pressed)
    {
      StratusQuo::lady_brown.move(127);
    }
    else if(L2_is_pressed)
    {
      StratusQuo::lady_brown.move(-127);
    }
    else StratusQuo::lady_brown.brake();
    
    pros::delay(50);
  }
});

pros::Task screen_task([]() {
  pros::delay(2000);
  while(true)
  {
    //console.focus();
    console.println(std::to_string(StratusQuo::lady_brown.get_position()));
    pros::delay(75);
    console.clear();
  }
});

void initialize() {
  pros::delay(500);

  StratusQuo::chassis.opcontrol_curve_buttons_toggle(true);
  StratusQuo::chassis.opcontrol_drive_activebrake_set(0.0);
  StratusQuo::chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  StratusQuo::lady_brown.tare_position();
  StratusQuo::lady_brown.set_exit_conditions(80, 50, 300, 150, 500, 500);

  StratusQuo::chassis.pid_tuner_pids.push_back({"Lift", &StratusQuo::LADY_BROWN_PID.constants});

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

  if(auton_selector.get_auton()) auton_selector.run_auton();
  else StratusQuo::blue_ring_side(); //StratusQuo::chassis.pid_drive_set(-6_in, 110); 
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
  StratusQuo::chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    if(/*master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)*/ false) is_position_based = !is_position_based;
    L1_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    L2_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    R1_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    R2_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    DOWN_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    pid_tuner();

    StratusQuo::chassis.opcontrol_tank();
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) set_intake(127);
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) set_intake(-127);
    else set_intake(0);

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      set_clamp.store(!StratusQuo::clamp.get_state());
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
      StratusQuo::doinker.set(!StratusQuo::doinker.get());
    }
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}