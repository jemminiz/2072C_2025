#include "autons.h"
#include "clamp.h"
#include "objects.h"

StratusQuo::Auton::Auton()
{};

void StratusQuo::Auton::skills()
{
    // do skills stuff
    dt.driveTo(5);
};

void StratusQuo::Auton::match()
{
    // do match stuff
    StratusQuo::dt.pid_drive(-10000);
    pros::delay(1000);
    StratusQuo::piston.toggle();
    StratusQuo::dt.pid_drive(500);
    pros::delay(1000);
    StratusQuo::dt.pid_drive(0);
    //StratusQuo::dt.set_target(5000000000);
    //StratusQuo::dt.dt_task(); // Drive forward
    StratusQuo::in.intake();
    pros::delay(1000); // Wait 1s - get ring into goal
    
    StratusQuo::in.stop();
    StratusQuo::dt.turn(-90000);
    pros::delay(300);
    StratusQuo::dt.set_target(10000);
    pros::delay(500);
    StratusQuo::in.intake();
    pros::delay(100);
    StratusQuo::dt.turn(180000);
    pros::delay(500);
    StratusQuo::dt.set_target(10000);
    StratusQuo::dt.pid_drive(10000);
    pros::delay(500);
    StratusQuo::dt.set_target(0);
    StratusQuo::dt.setLeftVoltage(0);
    StratusQuo::dt.setRightVoltage(0);
    StratusQuo::dt.set_target(10000);
    //StratusQuo::dt.pid_drive(10000);
    pros::delay(500);
};

void StratusQuo::Auton::goal_rush()
{
    // do goal rush
    //StratusQuo::dt.set_target(-5000000000000);
    //StratusQuo::dt.dt_task(); // Drive back into goal
    // Should theoretically multitask the limit switch
    StratusQuo::dt.pid_drive(-10000);
    pros::delay(1000);
    StratusQuo::piston.toggle();
    StratusQuo::dt.pid_drive(500);
    pros::delay(1000);
    StratusQuo::dt.pid_drive(0);
    //StratusQuo::dt.set_target(5000000000);
    //StratusQuo::dt.dt_task(); // Drive forward
    StratusQuo::in.intake();
    pros::delay(1000); // Wait 1s - get ring into goal
    
    StratusQuo::in.stop();
    /*
    StratusQuo::dt.turn(90000);
    pros::delay(300);
    StratusQuo::dt.set_target(0);
    StratusQuo::dt.setLeftVoltage(0);
    StratusQuo::dt.setRightVoltage(0);
    StratusQuo::dt.set_target(10000);
    StratusQuo::dt.pid_drive(10000);
    pros::delay(500);
    StratusQuo::dt.set_target(0);
    StratusQuo::dt.setLeftVoltage(0);
    StratusQuo::dt.setRightVoltage(0);
    StratusQuo::dt.set_target(10000);
    StratusQuo::dt.pid_drive(10000);
    pros::delay(500);
    */
    
    //i tried my best
};
// develop more autons as season develops