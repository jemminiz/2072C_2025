#pragma once 
#include <cstdint>
#include "EZ-Template/PID.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
namespace StratusQuo
{
    class Intake
    {
        public:
            Intake(uint8_t motor_port,
            uint8_t pneumatic_port
            );
            void intake();
            void outake();
            void intake_auton(int voltage);
            void brake();

            void set_constants(double kP, double kI, double kD);         
        private:
            pros::Motor intake_motor;
            pros::adi::Pneumatics intake_pneumatics;
            ez::PID intake_PID;
    };
}