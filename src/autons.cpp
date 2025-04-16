#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "devices.hpp"
#include "autons.hpp"
#include "drivercontrol.hpp"
#include "pros/rtos.hpp"

// function definitions

//       chassis.setPose(x, y, theta); 
//       chassis.turnTo(x, y, timeout);    
//       chassis.follow(path file, timeout, lookahead distance, isAsynchronous, forwards (false makes the robot run the path backwards));
//       chassis.moveToPose(x, y, theta, timeout, {maxSpeed, minSpeed, earlyExitRange});
//       chassis.moveToPoint(x, y, timeout, {maxSpeed, minSpeed, earlyExitRange, forwards});
//
//       chassis.waitUntil(inches); use this to have your bot wait until the path is finished
//       a really big number means just wait until the path has finished

const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;
void safe_exit_conditions() {
  chassis.pid_turn_exit_condition_set(150_ms, 3_deg, 300_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(150_ms, 3_deg, 300_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(150_ms, 1_in, 300_ms, 3_in, 500_ms, 500_ms);
}

void soloAwpSafe(bool isRed) { 
  double sign = isRed ? 1 : -1;

  wallStakeTask.suspend();
  chassis.drive_angle_set(-50 * sign);

  intakeRaise.set_value(true);
  pros::delay(200);
  intake = 127;

  chassis.pid_drive_set(8, 40, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(200);
  intakeRaise.set_value(false);
  pros::delay(200);


  intake = 0;

  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  //turn to go to alliance stake
  chassis.pid_turn_set(-90 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(14, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(0 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-3, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  intake = 127;
  pros::delay(750);
  intake = 0;

  // move forward 
  chassis.pid_drive_set(8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to goal
  chassis.pid_turn_set(-140 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  backClamp.set_value(true);
  chassis.pid_drive_set(-36, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  backClamp.set_value(false);
  pros::delay(300);

  chassis.pid_turn_set(90 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake = 127;

  chassis.pid_drive_set(28, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(180 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(38, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-38, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(-45 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();


}

void soloAwpSafeBlue() { soloAwpSafe(false); }

void soloAwpSafeRed() { soloAwpSafe(true); }

void positiveSideBlue() { 
  is_red_team.store(false);
  isAutoClamp = true;

  wallStakeTask.suspend();

  pros::Task wallStakeWatch([]() {
    while(true)
    {
      if(wallStakeLimitSwitch.get_new_press())
      {
        wallStake.brake();
      }
      pros::delay(50);
    }
  });

  chassis.drive_angle_set(-20);

  wallStake.move_velocity(600);

  pros::delay(600);

  wallStake.move_velocity(-750);
  
  chassis.pid_drive_set(-5, DRIVE_SPEED, false);
  pros::delay(800);
  wallStake.move_velocity(0);
  chassis.pid_wait();
  
  intakeRaise.set_value(true);
  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait();

  roller_voltage.store(127); 
  chassis.pid_drive_set(20, 75, false);
  chassis.pid_wait();
  intakeRaise.set_value(false);
  pros::delay(200);

  chassis.pid_turn_set(-17, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(-34, 110, true);
  chassis.pid_wait();
  backClamp.set_value(true);

  chassis.pid_turn_set(120, TURN_SPEED);
  chassis.pid_wait();
  hook_voltage.store(127);
  chassis.pid_drive_set(27.5, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-17, DRIVE_SPEED, false);
  chassis.pid_wait();
  
  chassis.pid_turn_set(63, TURN_SPEED);
  chassis.pid_wait();
 
  wallStake.move_absolute(320, 127);
  chassis.pid_drive_set(35, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(45, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-15, 20, true);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-15, 20, true);
  chassis.pid_wait();

  chassis.pid_turn_set(220, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  chassis.pid_wait();
  backClamp.set_value(false);
  chassis.pid_turn_set(40, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
}

void positiveSideRed() { 
  is_red_team.store(true);
  isAutoClamp = true;

  wallStakeTask.suspend();

  pros::Task wallStakeWatch([]() {
    while(true)
    {
      if(wallStakeLimitSwitch.get_new_press())
      {
        wallStake.brake();
      }
      pros::delay(50);
    }
  });

  chassis.drive_angle_set(20);

  wallStake.move_velocity(600);

  pros::delay(600);

  wallStake.move_velocity(-600);
  
  chassis.pid_drive_set(-5, DRIVE_SPEED, false);
  pros::delay(800);
  wallStake.move_velocity(0);
  chassis.pid_wait();
  

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait();
  intakeRaise.set_value(true);

  roller_voltage.store(127); 
  chassis.pid_drive_set(20, 80, false);
  chassis.pid_wait();
  intakeRaise.set_value(false);
  pros::delay(200);

  chassis.pid_turn_set(15, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(-27, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-6, 80, false);
  chassis.pid_wait();
  set_clamp.store(true);

  chassis.pid_turn_set(235, TURN_SPEED);
  chassis.pid_wait();
  hook_voltage.store(127);
  chassis.pid_drive_set(27.5, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-13, DRIVE_SPEED, false);
  chassis.pid_wait();
  
  chassis.pid_turn_set(290, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(35, DRIVE_SPEED, false);
  chassis.pid_wait();
  pros::delay(300);
  chassis.pid_drive_set(45, 127, false);
  chassis.pid_wait();

  chassis.pid_drive_set(-15, 20, true);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-15, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(200, TURN_SPEED);
  chassis.pid_wait();
  set_clamp.store(false);
  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-200, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24, DRIVE_SPEED, true);


  /*
  leftDoinker.set_value(true);
  chassis.pid_drive_set(35, 127, false);
  pros::delay(200);
  chassis.pid_wait_until(31);

  pros::delay(100);
  leftDoinker.set_value(false);


  chassis.pid_drive_set(-13, 50, false);
  pros::delay(200);
  chassis.pid_wait_until(-7);
    intake = 0;

  leftDoinker.set_value(true);
  chassis.pid_wait();


  chassis.pid_turn_set(40, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  leftDoinker.set_value(false);

  chassis.pid_drive_set(31.5, DRIVE_SPEED, false);
  pros::delay(100);
  chassis.pid_wait();

  wallStake.move_absolute(470, 200);

  pros::delay(700);

  chassis.pid_drive_set(-19, DRIVE_SPEED, false);
  pros::delay(300);
  wallStake.move_absolute(-1000, 200);
  chassis.pid_wait();

  backClamp.set_value(true);

  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  liftSensor.reset_position();
  wallStake.move(0);

  chassis.pid_drive_set(-24, 70, true);
  pros::delay(100);
  chassis.pid_wait_until(-22);
  backClamp.set_value(false);
  chassis.pid_wait();

  intake = 127;

  chassis.pid_swing_set(ez::LEFT_SWING, 180, 70, 45);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  leftDoinker.set_value(true);
  backClamp.set_value(true);

  chassis.pid_drive_set(11, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(270, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(10, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  leftDoinker.set_value(false);

  chassis.pid_turn_set(175, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-34, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
*/
}


void positiveSideQuals(bool isRed) {
  double sign = isRed ? 1 : -1;

  chassis.drive_angle_set(-51 * sign);

  wallStake.move_absolute(480, 600);

  pros::delay(600);

  wallStake.move_absolute(10, 200);

  
  if (!isRed) {
    chassis.pid_swing_set(ez::RIGHT_SWING,-70 * sign, -SWING_SPEED, 0);
    pros::delay(100);
    chassis.pid_wait();
  } else {
    chassis.pid_swing_set(ez::LEFT_SWING,-70 * sign, -SWING_SPEED, 0);
    pros::delay(100);
    chassis.pid_wait();
  }

  backClamp.set_value(true);
  chassis.pid_drive_set(-38, 70, true);
  chassis.pid_wait_until(-36);
  backClamp.set_value(false);
  chassis.pid_wait();
  pros::delay(100);

  chassis.pid_turn_set(42 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  pros::delay(300);
  intake = 0;
  chassis.pid_wait_until(16);


  isRed ? rightDoinker.set_value(true) : leftDoinker.set_value(true);

  pros::delay(250);

  chassis.pid_turn_set(65 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  if (!isRed) {
    chassis.pid_drive_set(4, DRIVE_SPEED, false);
    chassis.pid_wait();
  } else {
    chassis.pid_drive_set(4, DRIVE_SPEED, false);
    chassis.pid_wait();
  }

  isRed ? leftDoinker.set_value(true) : rightDoinker.set_value(true);

  pros::delay(250);

  if (!isRed) {
    chassis.pid_swing_set(ez::RIGHT_SWING,45 * sign, -SWING_SPEED, 0);
    pros::delay(100);
    chassis.pid_wait();
  } else {
    chassis.pid_swing_set(ez::LEFT_SWING,45 * sign, -SWING_SPEED, 0);
    pros::delay(100);
    chassis.pid_wait();
  }


  chassis.pid_drive_set(-45, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait_until(-43);

  leftDoinker.set_value(false);
  rightDoinker.set_value(false);

  pros::delay(250);

  chassis.pid_turn_set(-10 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake.move_velocity(600);

  if (!isRed) {
    chassis.pid_swing_set(ez::RIGHT_SWING,200 * sign, SWING_SPEED, 20);
    pros::delay(100);
    chassis.pid_wait();
  } else {
    chassis.pid_swing_set(ez::LEFT_SWING,200 * sign, SWING_SPEED, 20);
    pros::delay(100);
    chassis.pid_wait();
  }

  chassis.pid_drive_set(30, DRIVE_SPEED, false);
  pros::delay(100);
  chassis.pid_wait_until(28);

  chassis.pid_turn_set(260.5 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(30.5, DRIVE_SPEED, false);
  pros::delay(100);
  chassis.pid_wait_until(7);
  isRed ? leftDoinker.set_value(true) : rightDoinker.set_value(true);
  chassis.pid_wait_until(28.5);

  intake = 0;

  chassis.pid_turn_set(340 * sign, TURN_SPEED);
  pros::delay(200);
  chassis.pid_wait();

  backClamp.set_value(true);


  chassis.pid_drive_set(15, DRIVE_SPEED, false);
  pros::delay(100);
  chassis.pid_wait_until(10);

  leftDoinker.set_value(false);
  rightDoinker.set_value(false);

  chassis.pid_turn_set(405 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait_until(403 * sign);

  chassis.pid_drive_set(54, DRIVE_SPEED, false);
  pros::delay(100);
  chassis.pid_wait_until(54);
}

void positiveSideSimple(bool isRed) {
  double sign = isRed ? -1 : 1;
  is_red_team.store(isRed);
  isAutoClamp = true;

  wallStakeTask.suspend();

  /*pros::Task wallStakeWatch([]() {
    while(true)
    {
      if(wallStakeLimitSwitch.get_new_press())
      {
        wallStake.brake();
      }
      pros::delay(50);
    }
  });*/

  chassis.drive_angle_set(0*sign);
  wallStake.move_velocity(600);
  pros::delay(300);
  wallStake.brake();
  chassis.pid_wait();
  hook_voltage.store(127);
  roller_voltage.store(127);
  chassis.pid_drive_set(42, DRIVE_SPEED, true);
  chassis.pid_wait_until(10);
  stopIntake = true;
  intakeRaise.set_value(true);


  chassis.pid_wait_until(25);
  intakeRaise.set_value(false);
  chassis.pid_wait();
  wallStake.move_velocity(600);
  pros::delay(400);
  stopIntake = false;
  chassis.pid_wait();
  wallStake.brake();

  chassis.pid_turn_set(-95*sign, TURN_SPEED);
  wallStake.move_velocity(-600);
  pros::delay(1200);
  wallStake.brake();
  chassis.pid_wait();
  roller_voltage.store(-127);
  
  chassis.pid_drive_set(-35, DRIVE_SPEED, true);
  chassis.pid_wait();
  

  
  chassis.pid_turn_set(54*sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(4, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  leftDoinker.set_value(true);
  chassis.pid_drive_set(-10, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_swing_set(isRed ? ez::RIGHT_SWING : ez::LEFT_SWING, -10 * sign, -70, -25);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-5, DRIVE_SPEED, true);
  chassis.pid_wait_until(-1);
  backClamp.set_value(true);
  backClamped = true;
  chassis.pid_wait();
/*  
  chassis.pid_drive_set(-40, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-20*sign, TURN_SPEED);
  hook_voltage.store(127);
  chassis.pid_wait();

  chassis.pid_drive_set(10, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(120*sign, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(60, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-15, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  chassis.pid_wait();*/
}

void positiveSideSimpleRed() { positiveSideSimple(true); } 

void positiveSideSimpleBlue() { positiveSideSimple(false); }


void positiveSideQualsBlue() { positiveSideQuals(false); }

void positiveSideQualsRed() { positiveSideQuals(true); }

void negativeSideQuals(bool isRed) {
  double sign = isRed ? 1 : -1;
  chassis.drive_angle_set(-22 * sign);
  pros::Task wallStakeWatch([]() {
    while(true)
    {
      if(wallStakeLimitSwitch.get_new_press()) wallStake.brake();
      pros::delay(50);
    }
  });

  wallStake.move_voltage(12000);
  pros::delay(750);
  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  chassis.pid_wait();
  wallStake.move_voltage(-12000);
  chassis.pid_turn_set(180 * sign, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  chassis.pid_wait();
  set_clamp.store(true);
  pros::delay(50);

  hook_voltage.store(127);
  roller_voltage.store(127);

  chassis.pid_turn_set(315 * sign, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(15, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  chassis.pid_swing_set(sign ? ez::RIGHT_SWING : ez::LEFT_SWING, 270 * sign, SWING_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12, 110, true);
  chassis.pid_wait();
  
  chassis.pid_swing_set(sign ? ez::RIGHT_SWING : ez::LEFT_SWING, 135 * sign, SWING_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(36, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(270 * sign, TURN_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(60, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-12, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-16, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90 * sign, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(60, DRIVE_SPEED, true);
  intakeRaise.set_value(true);
  chassis.pid_wait();

  chassis.pid_turn_set(0 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24, 50, false);
}

void negativeSideQualsBlue() { negativeSideQuals(false); }

void negativeSideQualsRed() { negativeSideQuals(true); }

void negativeSide(bool isRed) { 
  double sign = isRed ? -1 : 1;
  chassis.drive_angle_set(-25 * sign);
  backClamp.set_value(true);

  chassis.pid_drive_set(-34, 70, true);
  pros::delay(100);
  chassis.pid_wait_until(-32);
  backClamp.set_value(false);
  chassis.pid_wait();
  pros::delay(250);

  chassis.pid_turn_set(-90 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake = 127;
  chassis.pid_drive_set(31, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(-180 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(15, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(-220 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  

    pros::delay(1000);


  chassis.pid_turn_set(-180 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-28, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(-105 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  backClamp.set_value(true);

  chassis.pid_drive_set(-70, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

    backClamp.set_value(false);


  intake = 0;
}

void negativeSideBlue() { negativeSide(false); }

void negativeSideRed() { negativeSide(true); }

void skills() {
  safe_exit_conditions();

  // score on alliance stake
  intake = 127;
  pros::delay(500);
  intake = 0;

  // move forward 
  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to goal
  chassis.pid_turn_set(68, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move to goal
  backClamp.set_value(true);
  chassis.pid_drive_set(-24, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  backClamp.set_value(false);
  pros::delay(300);

  //turn to first ring
  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  intake = 127;
  chassis.pid_drive_set(21, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to second ring
  chassis.pid_turn_set(-52.5, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(40, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  //move back a bit
  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to third ring
  chassis.pid_turn_set(-180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(50, 60, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(500);

  // turn to fourth ring
  chassis.pid_turn_set(-55, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(11, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  //turn to put goal in corner
  chassis.pid_turn_set(15, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // put goal in corner
  backClamp.set_value(true);
  chassis.pid_drive_set(-12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(6, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(7, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to far ring
  chassis.pid_drive_set(72, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  intake = 0;

  // turn to blue goal
  chassis.pid_turn_set(-135, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to blue goal
  chassis.pid_drive_set(-32, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  //turn to blue goal
  chassis.pid_turn_set(-180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // grab blue goal
  chassis.pid_drive_set(-12, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  backClamp.set_value(false);
  pros::delay(300);

  // turn to corner
  chassis.pid_turn_set(-260, TURN_SPEED);
  pros::delay(300);
  chassis.pid_wait();

  // put goal in corner
  backClamp.set_value(true);

  chassis.pid_drive_set(10, DRIVE_SPEED, true);
  pros::delay(100);

  pros::delay(300);
  chassis.pid_drive_set(-42, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to middle goal
  chassis.pid_turn_set(-240, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to middle goal
  chassis.pid_drive_set(39, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to grab mid goal
  chassis.pid_turn_set(-120, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // grab mid goal
  chassis.pid_drive_set(-23, 70, true);
  pros::delay(100);
  chassis.pid_wait();
  
  backClamp.set_value(false);
  pros::delay(300);

  // turn second ring
  chassis.pid_turn_set(-225, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake = 127;

  // go to second ring
  chassis.pid_drive_set(35, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to third ring
  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to third ring
  chassis.pid_drive_set(36, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to fourth ring
  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to fourth ring
  chassis.pid_drive_set(28, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // backup a bit
  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to fifth ring
  chassis.pid_turn_set(-45, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to fifth ring
  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // BACK UP again
  chassis.pid_drive_set(-12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to corner
  chassis.pid_turn_set(193, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to corner
  backClamp.set_value(true);
  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
}

void skillsv2() {
  safe_exit_conditions();

  // score on alliance stake
  intake = 127;
  pros::delay(500);
  intake = 0;

  // move forward 
  chassis.pid_drive_set(14, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to goal
  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move to goal
  backClamp.set_value(true);
  chassis.pid_drive_set(-24, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  backClamp.set_value(false);
  pros::delay(200);

  //turn to first ring
  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  intake = 127;
  chassis.pid_drive_set(21, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to second ring
  chassis.pid_turn_set(-52.5, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(40, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  //move back a bit
  chassis.pid_drive_set(-10, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to third ring
  chassis.pid_turn_set(-180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(50, 60, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(500);

  // turn to fourth ring
  chassis.pid_turn_set(-47, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(14, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  // back up for alignment
  chassis.pid_drive_set(-9, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  //turn to put goal in corner
  chassis.pid_turn_set(45, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // put goal in corner
  backClamp.set_value(true);
  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  intake = 0;

  chassis.pid_drive_set(17, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to far ring
  chassis.pid_drive_set(-67, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-6, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  backClamp.set_value(false);
  pros::delay(200);

  //turn to first ring
  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  intake = 127;
  chassis.pid_drive_set(21, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to second ring
  chassis.pid_turn_set(52.5, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(40, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  //move back a bit
  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to third ring
  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(50, 60, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(500);

  // turn to fourth ring
  chassis.pid_turn_set(55, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // move forward and intake ring
  chassis.pid_drive_set(12, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  // back up for alignment
  chassis.pid_drive_set(-6, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  //turn to put goal in corner
  chassis.pid_turn_set(-45, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // put goal in corner
  backClamp.set_value(true);
  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  intake.move_velocity(200);

  chassis.pid_drive_set(135, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  intake = 0;

  chassis.pid_turn_set(-135, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-18, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-22, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  backClamp.set_value(false);
  pros::delay(200);

  intake = 127;

  chassis.pid_turn_set(135, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(36, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to third ring
  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to third ring
  chassis.pid_drive_set(34, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to fourth ring
  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to fourth ring
  chassis.pid_drive_set(28, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // backup a bit
  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to fifth ring
  chassis.pid_turn_set(-30, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to fifth ring
  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // BACK UP again
  chassis.pid_drive_set(-12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to corner
  chassis.pid_turn_set(213, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // go to corner
  backClamp.set_value(true);
  intake = 0;
  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING,315, SWING_SPEED, 10);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING,274, SWING_SPEED, 51);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(96, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(319, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  wallStake.move_absolute(450, 600);

  chassis.pid_drive_set(-58, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(315, TURN_SPEED);

  chassis.pid_drive_set(10, 40, true);
  pros::delay(100);
  chassis.pid_wait();

  wallStake.move_absolute(-20, 600);

}

void soloAwp(bool isRed) { 
  double sign = isRed ? 1 : -1;

  is_red_team.store(isRed);

  wallStakeTask.suspend();

  pros::Task wallStakeWatch([]() {
    while(true)
    {
      if(wallStakeLimitSwitch.get_new_press())
      {
        wallStake.brake();
      }
      pros::delay(50);
    }
  });

  chassis.drive_angle_set(125 * sign);

  wallStake.move_velocity(600);

  pros::delay(600);

  chassis.pid_drive_set(-18, DRIVE_SPEED, true);
  chassis.pid_wait();
  wallStake.move(-127);
  chassis.pid_turn_set(180 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-14, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-10, 40, false);
  chassis.pid_wait();
  set_clamp.store(true);
  chassis.pid_wait();
  chassis.pid_turn_set(265 * sign, TURN_SPEED);
  chassis.pid_wait();
  roller_voltage.store(127);
  hook_voltage.store(127);
  chassis.pid_drive_set(25, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(110 * sign, TURN_SPEED);
  chassis.pid_wait();
  intakeRaise.set_value(true);
  chassis.pid_wait();  
  chassis.pid_drive_set(60, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(45 * sign, TURN_SPEED);
  chassis.pid_wait();
  intakeRaise.set_value(false);
  isAutoClamp = false; 
  backClamp.set_value(false);
  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  hook_voltage.store(0);
  chassis.pid_wait();
  chassis.pid_turn_set(180 * sign, TURN_SPEED);
  chassis.pid_wait();  
  isAutoClamp = true;
  chassis.pid_drive_set(-15, DRIVE_SPEED, true);
  chassis.pid_wait_until(-8);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait();
  backClamp.set_value(true);
  backClamped = true;
  hook_voltage.store(127);
  chassis.pid_turn_set(85 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(270 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(48, DRIVE_SPEED, true);
}

void soloAwpBlue() {
  soloAwp(false);
}

void soloAwpRed() {
  soloAwp(true);
}

void easyNegativeQuals(bool isRed)
{
  double sign = isRed ? 1 : -1;

  is_red_team.store(isRed);

  wallStakeTask.suspend();

  pros::Task wallStakeWatch([]() {
    while(true)
    {
      if(wallStakeLimitSwitch.get_new_press())
      {
        wallStake.brake();
      }
      pros::delay(50);
    }
  });

  chassis.drive_angle_set(125 * sign);

  wallStake.move_velocity(600);

  pros::delay(600);

  chassis.pid_drive_set(-18, DRIVE_SPEED, true);
  chassis.pid_wait();
  wallStake.move(-127);
  chassis.pid_turn_set(180 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-14, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-10, 40, false);
  chassis.pid_wait();
  hook_voltage.store(-127);
  set_clamp.store(true);

  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  chassis.pid_wait();
  roller_voltage.store(127);
  hook_voltage.store(127);
  chassis.pid_turn_set(315 * sign, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(15, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_swing_set(isRed ? ez::RIGHT_SWING : ez::LEFT_SWING, 270 * sign, SWING_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(24, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_drive_set(-12, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_swing_set(isRed ? ez::RIGHT_SWING : ez::LEFT_SWING, 215 * sign, SWING_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(40, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  wallStake.move_velocity(200);
  chassis.pid_wait();
}
void easyNegativeQualsBlue()
{
  easyNegativeQuals(false);
}
void easyNegativeQualsRed()
{
  easyNegativeQuals(true);
}

void negativeNoAllianceStakeQuals(bool isRed)
{
  double sign = isRed ? 1 : -1;

  is_red_team.store(isRed);

  wallStakeTask.suspend();

  pros::Task wallStakeWatch([]() {
    while(true)
    {
      if(wallStakeLimitSwitch.get_new_press())
      {
        wallStake.brake();
      }
      pros::delay(50);
    }
  });

  chassis.pid_drive_set(-18, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(180 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-14, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-10, 40, false);
  chassis.pid_wait();
  hook_voltage.store(-127);
  roller_voltage.store(127);
  set_clamp.store(true);
  pros::delay(50);
  hook_voltage.store(127);

  chassis.pid_turn_set(315 * sign, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_swing_set(isRed ? ez::RIGHT_SWING : ez::LEFT_SWING, 270 * sign, SWING_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-12, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_swing_set(isRed ? ez::RIGHT_SWING : ez::LEFT_SWING, 135 * sign, SWING_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(36, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(215 * sign, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(60, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-16, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(60, DRIVE_SPEED, true);
  intakeRaise.set_value(true);
  intakeRaised = true;
  chassis.pid_wait();

  chassis.pid_turn_set(0 * sign, TURN_SPEED);
  intakeRaise.set_value(false);
  intakeRaised = false;
  chassis.pid_wait();

  chassis.pid_drive_set(24, 50, true);
}
void negativeNoAllianceStakeQualsBlue()
{
  negativeNoAllianceStakeQuals(false);
}
void negativeNoAllianceStakeQualsRed()
{
  negativeNoAllianceStakeQuals(true);
}

void negativeAllianceStakeLast(bool isRed)
{
  double sign = isRed ? 1 : -1;

  is_red_team.store(isRed);

  wallStakeTask.suspend();

  pros::Task wallStakeWatch([]() {
    while(true)
    {
      if(wallStakeLimitSwitch.get_new_press())
      {
        wallStake.brake();
      }
      pros::delay(50);
    }
  });

  chassis.drive_angle_set(180 * sign);

  chassis.pid_drive_set(-18, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-6, 50, false);

  chassis.pid_wait();
  set_clamp.store(true);

  chassis.pid_turn_set(330 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24.5, DRIVE_SPEED, true);
  roller_voltage.store(127);
  hook_voltage.store(127);
  chassis.pid_wait();
  chassis.pid_swing_set(isRed ? ez::RIGHT_SWING : ez::LEFT_SWING, 265 * sign, SWING_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(22, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-12, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_swing_set(isRed ? ez::RIGHT_SWING : ez::LEFT_SWING, 135 * sign, SWING_SPEED, -20);
  chassis.pid_wait();
  chassis.pid_drive_set(38, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(232 * sign, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_speed_max_set(40);
  chassis.pid_drive_set(60, DRIVE_SPEED, false);
  chassis.pid_wait_until(40);
  chassis.pid_wait();
  chassis.pid_drive_set(-20, 30, true);
  chassis.pid_wait();
  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-20, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(280 * sign, TURN_SPEED);
  chassis.pid_drive_set(-80, DRIVE_SPEED, false);

  /*
  chassis.pid_turn_set(90 * sign, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(48, DRIVE_SPEED, true);
  intakeRaise.set_value(true);
  wallStake.move_absolute(100, 200);
  chassis.pid_wait();
  chassis.pid_turn_set(180 * sign, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(6, 50, false);
  hook_voltage.store(0);
  wallStake.move_voltage(12000);
  chassis.pid_wait();
  chassis.pid_drive_set(-12, DRIVE_SPEED, true);
  chassis.pid_wait();
*/
  }
void negativeAllianceStakeLastRed()
{
  negativeAllianceStakeLast(true);
}
void negativeAllianceStakeLastBlue()
{
  negativeAllianceStakeLast(false);
}