#pragma once
#include "EZ-Template/api.hpp" // IWYU pragma: keep
#include "pros/motors.hpp"
namespace StratusQuo
{
    inline int LB_POSITION[3] = {0, 100, 300};
    class Lady_Brown
    {
        public:
            Lady_Brown() = delete;
            Lady_Brown(int port);
            Lady_Brown(int port, ez::PID pid);
            Lady_Brown(const int port, const ez::PID pid, const pros::Rotation rotation);
            int move(int8_t voltage);
            int move_to(int position);
            int tare_position();
            int brake();
        private:
            int _position = LB_POSITION[0];
            pros::Motor _motor;
            pros::Rotation _rotation;
            ez::PID _pid;
    };
}