#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <cstdint>
namespace StratusQuo
{
    enum Arm_State
    {
        ZERO, ABOVE_ARM, LOADING, SCORING
    };
    class Arm
    {
        public:
            Arm(int motor_port, uint8_t pneumatic_port, bool starting_config, int sensor_port);
            
            void toggle();
            void up();
            void down();
            void move(int voltage);
            void brake();
            void change_state(Arm_State state);
            void initialize();
            
            double get_rotation();
        private:
            pros::Motor motor;
            pros::adi::Pneumatics piston;
            pros::Rotation sensor;
            Arm_State currentState = ZERO;
    };
}