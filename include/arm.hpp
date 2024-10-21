#pragma once 
#include <cstdint>
#include "EZ-Template/PID.hpp"
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
            double get_rotation();
            void brake();
            void toggle();

            void stop();

            void macro_task();

            void set_constants(double kP, double kI, double kD);         
        private:
            pros::Motor arm_motor;
            pros::ADIDigitalOut arm_pneumatics;
            pros::Rotation arm_sensor;
            PID arm_PID;
            bool pneumatic_state = false;
    };
}