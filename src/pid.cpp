#include "pid.hpp"

StratusQuo::PID::PID()
{}
StratusQuo::PID::PID(float kP, float kI, float kD)
{
    this->_kP = kP;
    this->_kI = kI;
    this->_kD = kD;
    _position_feedback = nullptr;
}
StratusQuo::PID::PID(float kP, float kI, float kD, std::function<double()> callback)
{
    this->_kP = kP;
    this->_kI = kI;
    this->_kD = kD;
    _position_feedback = callback;
}

int StratusQuo::PID::set_position_feedback(std::function<double()> callback)
{
    if(!callback) return 1;
    _position_feedback = callback;
    return 0;
}
int StratusQuo::PID::set_kP(float new_kP)
{
    if(new_kP >= 0) _kP = new_kP;
    else return 1;
    return 0;
}
int StratusQuo::PID::set_kI(float new_kI)
{
    if(new_kI >= 0) _kI = new_kI;
    else return 1;
    return 0;
}
int StratusQuo::PID::set_kD(float new_kD)
{
    if(new_kD < 0) return 1;
    _kD = new_kD;
    return 0;
}

int StratusQuo::PID::move_to(float target, std::function<std::int32_t(std::int32_t voltage)> set_voltage)
{
    set_voltage(_calculate(target));
    return 0;
}

int StratusQuo::PID::_calculate(float target)
{
    if(!_position_feedback) return 0;

    float _position_ = _position_feedback(); // current position!
    float _power_ = 0.f;

    float _set_point_ = target;
    float _error_ = _set_point_ - _position_;
    float _integral_ = _error_;
    float _derivative_ = 0.f;
    if(_error_ == 0) _integral_ = 0;
    if (_error_ * _kP >= 127) _integral_ = 0;

    // calculate derivative on measurement instead of error to avoid "derivative kick"
    // https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control
    _derivative_ = _position_ - _prev_position;

    _power_ = _error_ * _kP + _integral_ * _kI + _derivative_ * _kD;
    _prev_position = _position_;
    return _power_;
}

const bool StratusQuo::PID::can_get_position()
{
    return static_cast<bool>(_position_feedback);
}

const std::function<double()> StratusQuo::PID::get_position_feedback()
{
    return _position_feedback;
}


/*
 *  // TODO: DELETE ME
 *  moveTo(500, [this](float power) { this->motor->setPower(power);}); -- Lambda function
 *  moveTo(500, std::bind(motor, &Motor::setPower)); -- Binding to not have to use Lambdas
*/