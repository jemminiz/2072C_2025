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
            int setPositionFeedback(std::function<double()> callback);
            std::function<double()> getPositionFeedback();
            int setKP(float newKP);
            int setKI(float newKI);
            int setKD(float newKD);
            int move_to(float target, std::function<std::int32_t(std::int32_t voltage)> set_voltage);
            bool canGetPosition();
        private:
            float kP = 0.f, kI = 0.f, kD = 0.f;
            std::function<double()> positionFeedback;
            float prev_position = 0.f;
            int calculate(float target);
    };
}