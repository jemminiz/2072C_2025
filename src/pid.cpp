#include "pid.h"
#include "utils.h"
#include <cmath>

void StratusQuo::PID::variables_reset()
{
    output = 0;
    target = 0;
    error = 0;
    prev_error = 0;
    integral = 0;
    time = 0;
    prev_time = 0;
}
StratusQuo::PID::PID()
{
    variables_reset();
    constants_set(0, 0, 0, 0);
}
StratusQuo::PID::Constants StratusQuo::PID::constants_get()
{
    return constants;
}
StratusQuo::PID::PID(double p, double i, double d, double start_i, std::string name) {
  variables_reset();
  constants_set(p, i, d, start_i);
  name_set(name);
}
void StratusQuo::PID::constants_set(double p, double i, double d, double p_start_i) {
  constants.kp = p;
  constants.ki = i;
  constants.kd = d;
  constants.start_i = p_start_i;
}
void StratusQuo::PID::exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout) {
  exit.small_exit_time = p_small_exit_time;
  exit.small_error = p_small_error;
  exit.big_exit_time = p_big_exit_time;
  exit.big_error = p_big_error;
  exit.velocity_exit_time = p_velocity_exit_time;
  exit.mA_timeout = p_mA_timeout;
}
void StratusQuo::PID::target_set(double input) { target = input; }
double StratusQuo::PID::target_get() { return target; }
void StratusQuo::PID::i_reset_toggle(bool toggle) { reset_i_sgn = toggle; }
bool StratusQuo::PID::i_reset_get() { return reset_i_sgn; };
double StratusQuo::PID::compute(double current) {
  error = target - current;
  return compute_error(error, current);
}
double StratusQuo::PID::compute_error(double err, double current) {
  error = err;
  cur = current;

  return raw_compute();
}

double StratusQuo::PID::raw_compute() {
  // calculate derivative on measurement instead of error to avoid "derivative kick"
  // https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control
  derivative = cur - prev_current;

  if (constants.ki != 0) {
    // Only compute i when within a threshold of target
    if (fabs(error) < constants.start_i)
      integral += error;

    // Reset i when the sign of error flips
    if (sgn(error) != sgn(prev_current) && reset_i_sgn)
      integral = 0;
  }

  output = (error * constants.kp) + (integral * constants.ki) - (derivative * constants.kd);

  prev_current = cur;
  prev_error = error;

  return output;
}

void StratusQuo::PID::timers_reset() {
  i = 0;
  k = 0;
  j = 0;
  l = 0;
  m = 0;
  is_mA = false;
}

void StratusQuo::PID::name_set(std::string p_name) {
  name = p_name;
  name_active = name == "" ? false : true;
}

void StratusQuo::PID::velocity_sensor_secondary_toggle_set(bool toggle) { use_second_sensor = toggle; }
bool StratusQuo::PID::velocity_sensor_secondary_toggle_get() { return use_second_sensor; }

void StratusQuo::PID::velocity_sensor_secondary_set(double secondary_sensor) { second_sensor = secondary_sensor; }
double StratusQuo::PID::velocity_sensor_secondary_get() { return second_sensor; }

void StratusQuo::PID::velocity_sensor_main_exit_set(double zero) { velocity_zero_main = zero; }
double StratusQuo::PID::velocity_sensor_main_exit_get() { return velocity_zero_main; }

void StratusQuo::PID::velocity_sensor_secondary_exit_set(double zero) { velocity_zero_secondary = zero; }
double StratusQuo::PID::velocity_sensor_secondary_exit_get() { return velocity_zero_secondary; }

StratusQuo::exit_output StratusQuo::PID::exit_condition(bool print) {
  // If this function is called while all exit constants are 0, print an error
  if (exit.small_error == 0 && exit.small_exit_time == 0 && exit.big_error == 0 && exit.big_exit_time == 0 && exit.velocity_exit_time == 0 && exit.mA_timeout == 0) {
    return ERROR_NO_CONSTANTS;
  }

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (exit.small_error != 0) {
    if (abs(error) < exit.small_error) {
      j += DELAY_TIME;
      i = 0;  // While this is running, don't run big thresh
      if (j > exit.small_exit_time) {
        timers_reset();
        return SMALL_EXIT;
      }
    } else {
      j = 0;
    }
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.  This does not run while small_timeout is running
  else if (exit.big_error != 0 && exit.big_exit_time != 0) {  // Check if this condition is enabled
    if (abs(error) < exit.big_error) {
      i += DELAY_TIME;
      if (i > exit.big_exit_time) {
        timers_reset();
        return BIG_EXIT;
      }
    } else {
      i = 0;
    }
  }

  // If the motor velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0) {  // Check if this condition is enabled
    if (abs(derivative) <= velocity_zero_main) {
      k += DELAY_TIME;
      if (k > exit.velocity_exit_time) {
        timers_reset();
        return VELOCITY_EXIT;
      }
    } else {
      k = 0;
    }
  }

  if (!use_second_sensor)
    return RUNNING;

  // If the secondary sensors velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0) {  // Check if this condition is enabled
    if (abs(second_sensor) <= velocity_zero_secondary) {
      m += DELAY_TIME;
      if (m > exit.velocity_exit_time) {
        timers_reset();
        return VELOCITY_EXIT;
      }
    } else {
      m = 0;
    }
  }

  // printf("j: %i   i: %i   k: %i   m: %i\n", j, i, k, m);

  return RUNNING;
}

StratusQuo::exit_output StratusQuo::PID::exit_condition(pros::Motor sensor, bool print) {
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    if (sensor.is_over_current()) {
      l += DELAY_TIME;
      if (l > exit.mA_timeout) {
        timers_reset();
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition(print);
}

StratusQuo::exit_output StratusQuo::PID::exit_condition(std::vector<pros::Motor> sensor, bool print) {
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    for (auto i : sensor) {
      // Check if 1 motor is pulling too many mA
      if (i.is_over_current()) {
        is_mA = true;
        break;
      }
      // If all of the motors aren't drawing too many mA, keep bool false
      else {
        is_mA = false;
      }
    }
    if (is_mA) {
      l += DELAY_TIME;
      if (l > exit.mA_timeout) {
        timers_reset();
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition(print);
}