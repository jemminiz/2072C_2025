#ifndef PID_H
#define PID_H
#include "constants.h"
#include "pros/motors.hpp"
#include <string>
namespace StratusQuo {
class PID {
public:
  PID();
  PID(double p, double i = 0, double d = 0, double start_i = 0,
      std::string name = "");
  void constants_set(double p, double i = 0, double d = 0,
                     double p_start_i = 0);
  struct Constants {
    double kp;
    double ki;
    double kd;
    double start_i;
  };
  struct exit_condition_ {
    int small_exit_time = 0;
    double small_error = 0;
    int big_exit_time = 0;
    double big_error = 0;
    int velocity_exit_time = 0;
    int mA_timeout = 0;
  };
  void exit_condition_set(int p_small_exit_time, double p_small_error,
                          int p_big_exit_time = 0, double p_big_error = 0,
                          int p_velocity_exit_time = 0, int p_mA_timeout = 0);
  void target_set(double input);
  double compute(double current);
  double compute_error(double err, double current);
  double target_get();
  Constants constants_get();
  void variables_reset();
  Constants constants;
  exit_condition_ exit;
  void velocity_sensor_secondary_set(double secondary_sensor);
  double velocity_sensor_secondary_get();
  void velocity_sensor_secondary_toggle_set(bool toggle);
  bool velocity_sensor_secondary_toggle_get();
  void velocity_sensor_main_exit_set(double zero);
  double velocity_sensor_main_exit_get();
  void velocity_sensor_secondary_exit_set(double zero);
  double velocity_sensor_secondary_exit_get();
  exit_output exit_condition(bool print = false);
  exit_output exit_condition(pros::Motor sensor,
                                         bool print = false);
  exit_output exit_condition(std::vector<pros::Motor> sensor,
                                         bool print = false);
  void name_set(std::string name);
  std::string name_get;
  void i_reset_toggle(bool toggle);
  bool i_reset_get();
  void timers_reset();

  double output = 0.0;
  double cur = 0.0;
  double error = 0.0;
  double target = 0.0;
  double prev_error = 0.0;
  double prev_current = 0.0;
  double integral = 0.0;
  double derivative = 0.0;
  long time = 0;
  long prev_time = 0;

private:
  double velocity_zero_main = 0.05;
  double velocity_zero_secondary = 0.1;
  int i = 0, j = 0, k = 0, l = 0, m = 0;
  bool is_mA = false;
  double second_sensor = 0.0;

  std::string name;
  bool name_active = false;
  void exit_condition_print(exit_output exit_type);
  bool reset_i_sgn = true;
  double raw_compute();
  bool use_second_sensor = false;
};
}; // namespace StratusQuo

#endif