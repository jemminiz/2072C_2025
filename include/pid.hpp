#pragma once
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
            int setKP(float newKP);
            int setKI(float newKI);
            int setKD(float newKD);
            int moveTo(float target, std::function<int(float power)> powerAdjustmentFunc);
        private:
            float kP = 0.f, kI = 0.f, kD = 0.f;
            std::function<double()> positionFeedback;
    };
}