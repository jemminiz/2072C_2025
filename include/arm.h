#pragma once 
#include <cstdint>
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
namespace StratusQuo
{
    class Arm
    {
        public:
            Arm(uint8_t motor_port,
            uint8_t pneumatic_port,
            uint8_t sensor_port
            );
            void arm_down();
            void arm_up();
            void arm_move(int voltage);            
        private:
            pros::Motor arm_motor;
            pros::adi::Pneumatics arm_pneumatics;
            pros::Rotation arm_sensor;
    };
}