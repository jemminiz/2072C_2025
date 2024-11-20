#include "pid.hpp"

StratusQuo::PID::PID()
{}
StratusQuo::PID::PID(float kP, float kI, float kD)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    positionFeedback = nullptr;
}
StratusQuo::PID::PID(float kP, float kI, float kD, std::function<double()> callback)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    positionFeedback = callback;
}

int StratusQuo::PID::setPositionFeedback(std::function<double()> callback)
{
    if(!callback) return 1;
    positionFeedback = callback;
    return 0;
}
int StratusQuo::PID::setKP(float newKP)
{
    if(newKP >= 0) kP = newKP;
    else return 1;
    return 0;
}
int StratusQuo::PID::setKI(float newKI)
{
    if(newKI >= 0) kI = newKI;
    else return 1;
    return 0;
}
int StratusQuo::PID::setKD(float newKD)
{
    if(newKD >= 0) kD = newKD;
    else return 1;
    return 0;
}

int StratusQuo::PID::move_to(float target, std::function<std::int32_t(std::int32_t voltage)> set_voltage)
{
    set_voltage(calculate(target));
    return 0;
}

int StratusQuo::PID::calculate(float target)
{
    if(!positionFeedback) return 0;

    float position = positionFeedback(); // current position!
    float power = 0.f;

    float set_point = target;
    float error = set_point - position;
    float integral = error;
    float derivative = 0.f;
    if(error == 0) integral = 0;
    if (error * kP >= 127) integral = 0;

    // calculate derivative on measurement instead of error to avoid "derivative kick"
    // https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control
    derivative = position - prev_position;

    power = error * kP + integral * kI + derivative * kD;
    prev_position = position;
    return power;
}

/*
 *  // TODO: DELETE ME
 *  moveTo(500, [this](float power) { this->motor->setPower(power);}); -- Lambda function
 *  moveTo(500, std::bind(motor, &Motor::setPower)); -- Binding to not have to use Lambdas
*/