#include "main.h"
#include "lady_brown.hpp"
#include "robodash.hpp"
#include "subsystems.hpp"

bool is_position_based = false;
bool L1_is_pressed = false;
bool L2_is_pressed = false;
int lady_brown_speed = 0;

pros::Task lady_brown_task([](){
  int current_position = 0;
  while(true)
  {
    if(is_position_based)
    {
      // How should this work
      if(L1_is_pressed) StratusQuo::lady_brown.move_to(StratusQuo::LB_POSITION[(abs(current_position - 1)) % 3]);
      else if(L2_is_pressed) StratusQuo::lady_brown.move_to(StratusQuo::LB_POSITION[(abs(current_position + 1)) % 3]);
    }
    else
    {
      if(L1_is_pressed) StratusQuo::lady_brown.move(127);
      else if(L2_is_pressed) StratusQuo::lady_brown.move(-127);
      else StratusQuo::lady_brown.brake();
    }
    pros::delay(50);
  }
});
void initialize() {
  pros::delay(500);

  StratusQuo::chassis.opcontrol_curve_buttons_toggle(true);
  StratusQuo::chassis.opcontrol_drive_activebrake_set(0.0);
  StratusQuo::chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  StratusQuo::lady_brown.tare_position();

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
  else StratusQuo::chassis.pid_drive_set(-6_in, 110);
}

void pid_tuner() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      StratusQuo::chassis.pid_tuner_toggle();

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
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
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) is_position_based = !is_position_based;
    L1_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    L2_is_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    pid_tuner();

    StratusQuo::chassis.opcontrol_tank();
    
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) StratusQuo::intake.move(127);
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) StratusQuo::intake.move(-127);
    else StratusQuo::intake.brake();

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
