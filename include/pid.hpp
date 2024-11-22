#pragma once
#include <cstdint>
#include <functional>
namespace StratusQuo
{
    class PID
    {
        public:
            PID();
            PID(float kP, float kI, float kD);
            PID(float kP, float kI, float kD, std::function<double()> callback);
            int set_position_feedback(std::function<double()> callback);
            const std::function<double()> get_position_feedback();
            int set_kP(float new_kP);
            int set_kI(float new_kI);
            int set_kD(float new_kD);
            int move_to(float target, std::function<std::int32_t(std::int32_t voltage)> set_voltage);
            const bool can_get_position();
        private:
            float _kP = 0.f, _kI = 0.f, _kD = 0.f;
            std::function<double()> _position_feedback;
            float _prev_position = 0.f;
            int _calculate(float target);
    };
}