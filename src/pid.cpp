#include "pid.hpp"
#include "pros/rtos.hpp"

StratusQuo::PID::PID()
{}
StratusQuo::PID::PID(float kP, float kI, float kD)
{
    P = kP;
    I = kI;
    D = kD;
    currentFeedback = nullptr;
}
StratusQuo::PID::PID(float kP, float kI, float kD, std::function<double()> callback)
{
    P = kP;
    I = kI;
    D = kD;
    currentFeedback = callback;
}

int StratusQuo::PID::setCurrentFeedback(std::function<double()> callback)
{
    if(!callback) return 1;
    currentFeedback = callback;
    return 0;
}
int StratusQuo::PID::setKP(float newKP)
{
    if(newKP >= 0) P = newKP;
    else return 1;
    return 0;
}
int StratusQuo::PID::setKI(float newKI)
{
    if(newKI >= 0) I = newKI;
    else return 1;
    return 0;
}
int StratusQuo::PID::setKD(float newKD)
{
    if(newKD >= 0) D = newKD;
    else return 1;
    return 0;
}

int StratusQuo::PID::moveTo(float target, std::function<int(float power)> powerAdjustmentFunc)
{
    if(!currentFeedback) return 1;

    bool condition = true;
    float current = currentFeedback();
    float prev_current = 0.f;
    float power = 0.f;
    float dT = 10.f;

    float proportional = target;
    float integral = 0.f;
    float derivative = 0.f;
    while (condition)
    {
        // calculate derivative on measurement instead of error to avoid "derivative kick"
        // https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control
        proportional = current - prev_current; // TODO: Find current 
        integral += proportional;
        if(proportional == 0) integral = 0;
        if (proportional * P >= 127) integral = 0;
        power = proportional * P + integral * I + derivative * D;
        powerAdjustmentFunc(power);
        prev_current = current;
        current = currentFeedback();
        
        pros::delay(dT);
    }
    return 0;
}

/*
 *  // TODO: DELETE ME
 *  moveTo(500, [this](float power) { this->motor->setPower(power);}); -- Lambda function
 *  moveTo(500, std::bind(motor, &Motor::setPower)); -- Binding to not have to use Lambdas
*/