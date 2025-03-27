#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <atomic>

extern pros::Controller master;

extern pros::Motor hooks;
extern pros::Motor rollers;
extern pros::MotorGroup intake;
extern pros::Motor wallStake;


extern pros::MotorGroup left_side_motors;
extern pros::MotorGroup right_side_motors;

extern pros::ADIDigitalOut intakeRaise;
extern pros::ADIDigitalOut backClamp;
extern pros::ADIDigitalOut leftDoinker;
extern pros::ADIDigitalOut rightDoinker;


extern pros::ADIDigitalIn backClampLeftLimitSwitch;
extern pros::ADIDigitalIn backClampRightLimitSwitch;
extern pros::ADIDigitalIn wallStakeLimitSwitch;

extern ez::PID wallStakePID;


extern pros::Rotation liftSensor;
extern pros::Optical optical;
extern pros::Distance clampSensor;

extern bool wingsOut;
extern bool liftUp;

extern Drive chassis;

void checkMotorsAndPrintTemperature();
void calibrateChassis();
void chassisInits();

inline std::atomic<bool> set_clamp = false;
inline std::atomic<bool> is_auto_clamp_enabled = true;
inline std::atomic<bool> is_color_sort_enabled = true;
inline std::atomic<bool> is_red_team = true;
inline std::atomic<double> hook_voltage = 0;
inline std::atomic<double> roller_voltage = 0;

inline bool backClamped = false;
inline bool intakeRaised = false;
inline bool rightDoinkered = false;