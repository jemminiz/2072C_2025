#pragma once
#include "EZ-Template/api.hpp" // IWYU pragma: keep
#include "EZ-Template/util.hpp"
#include "pros/motors.hpp"
namespace StratusQuo
{
    typedef enum 
    {
        DOWN = 0,
        LOADING = 100,
        SCORING = 3000
    } LB_POSITION;

    inline const ez::PID LADY_BROWN_PID{2.2, 0, .1, 0, "Lift"};;
    class Lady_Brown
    {
        public:
            Lady_Brown() = delete;
            Lady_Brown(int port);
            Lady_Brown(int port, ez::PID pid);
            Lady_Brown(const int port, const ez::PID pid, const pros::Rotation rotation);
            int move(int8_t voltage);
            int move_to(double position);
            double get_position();
            int tare_position();
            int brake();
            double compute(double current);
            double get_target_position();
            void pid_wait();
            void set_constants(double kP = 0.0, double kI = 0.0, double kD = 0.0, double start_i = 0.0);
            void set_exit_conditions(double small_error_time, double small_error, double big_error_time = 0, double big_error = 0, int velocity_exit_time = 0, int mA_timeout = 0);
            void target_set(double target);
            double get_voltage();
            int compute_error(double error, double current);
            ez::exit_output get_exit_condition();
        private:
            int _position = DOWN;
            pros::Motor _motor;
            pros::Rotation _rotation;
            ez::PID _pid;
    };
}