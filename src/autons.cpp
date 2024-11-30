#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(12, 0.95, 0.0, 5.0, 2);//1.5, 0, 10, 0);
  chassis.set_drive_exit_conditions(0.1, 0, 3000);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_turn_constants(10, 0.8, 0, 0, 5);
  chassis.set_turn_exit_conditions(1.5, 30, 2000); // Last one was 1, 300, 3000

  chassis.set_heading_constants(6, .4, 0, 1, 0);

  chassis.set_swing_constants(6, .3, .001, 2, 15);
  chassis.set_swing_exit_conditions(1, 300, 3000);

}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}


//Driving Test
void drive_test(){
  chassis.drive_distance(12);
  wait(1,seconds);
  chassis.drive_distance(-6);
}

// Turning Test
void turn_test(){
  //turn_to_heading_large(120);
  turn_to_heading_medium(90);
  //turn_to_heading_small(45);
  //turn_to_heading_tiny(20);
  //turn_to_heading_tiny(30);
}


/* Funtion registered to run when Auto is started*/
void run_auto()
{
  auto_started = true;

  //skills_auto();
  //drive_test();
  red_wp_auto();
  auto_started = false;
}

/* ********************************* */
/* Bunch of pre-tuned turn functions */
/* ********************************* */
//Use for tiny turns (less than 30 degrees)
inline void turn_to_heading_tiny(float targetHeading) { chassis.turn_to_heading_1091A(targetHeading, 7.25, 1.0, 2000, 0.64, 0, 0.40, 5);}
//Use for small turns (30-60 degrees) - Tuned to 45
inline void turn_to_heading_small(float targetHeading) { chassis.turn_to_heading_1091A(targetHeading, 10, 1.0, 2000, 0.64, 0, 0.4, 5);}
//Use for medium turns (60-120 degrees) - tuned to 90
inline void turn_to_heading_medium(float targetHeading) {chassis.turn_to_heading_1091A(targetHeading, 12, 1.0, 1500, 0.78, 0.05, 0.80, 5); }
//Use for large turns (> 120 degrees) 
inline void turn_to_heading_large(float targetHeading) { chassis.turn_to_heading_1091A(targetHeading, 12, 1.0, 1500, 0.90, 0.05, 0.85, 5); }


/* ************************************ */
/* Bunch of pre-tuned Driving functions */
/* ************************************ */
inline void drive_distance_small(float distance) {
  chassis.drive_distance_1091A(distance, chassis.get_absolute_heading(), \
      /* driving volts, heading volts */ 11, 11, \
      /* tolerance, settle time, timeout */ 0.1, 20, 3000, \
      /* Driving kp, ki, kd, driving starti */ 0.95, 0.0, 2.5, 0.25*distance, \
      /* Heading kp, ki, kd, heading starti */ 0, 0, 0, 0);
}

inline void drive_distance_medium(float distance) {
  chassis.drive_distance_1091A(distance, chassis.get_absolute_heading(), \
      /* driving volts, heading volts */ 12, 12, \
      /* tolerance, settle time, timeout */ 0.1, 150, 3000, \
      /* Driving kp, ki, kd, driving starti */ 0.95, 0.2, 1.5, 0.25*distance, \
      /* Heading kp, ki, kd, heading starti */ 0, 0, 0, 0);
}

inline void drive_distance_large(float distance) {
  chassis.drive_distance_1091A(distance, chassis.get_absolute_heading(), \
      /* driving volts, heading volts */ 12, 12, \
      /* tolerance, settle time, timeout */ 0.25, 300, 3000, \
      /* Driving kp, ki, kd, driving starti */ 0.90, 0.0, 2.5, 0.1*distance, \
      /* Heading kp, ki, kd, heading starti */ 0, 0, 0, 0);
}

/// @brief Skills Auto
void skills_auto() {
  conveyor.setVelocity(100,percent);
  conveyor.setMaxTorque(100,percent);
  intake.setVelocity(100, percent);
  intake.setMaxTorque(100, percent);
  arm.setVelocity(100, percent);
  arm.setMaxTorque(100,percent);
  chassis.set_heading(0);
  
  chassis.drive_max_voltage=9;
  chassis.turn_max_voltage=9;
  conveyor.spinFor(0.7,seconds);
  conveyor.spin(reverse);
  wait(0.3,seconds);
  conveyor.stop();


  chassis.drive_timeout=400;
  chassis.drive_distance(3.8);
  turn_to_heading_large(255);
  chassis.drive_timeout=2000;
  chassis.turn_timeout=900;
  chassis.drive_distance(-8);
  chassis.drive_max_voltage=3;
  chassis.drive_distance(-9);

  chassis.drive_max_voltage=8;
  mogo.set(true);
  wait(1,seconds);
  turn_to_heading_medium(13);
  
  chassis.drive_max_voltage=9;
  chassis.turn_max_voltage=12;
  intake.spin(forward);
  conveyor.spin(forward);
  chassis.drive_settle_time=500;
 
  turn_to_heading_tiny(10);
  chassis.drive_timeout=750;
  chassis.drive_distance(20);
  turn_to_heading_tiny(17);
  arm.setStopping(brakeType::hold);
  arm.setVelocity(50.0, percent);
  wait(1.5, seconds);
  while (armRotation.position(degrees) < 24.0) {  //was 32
    arm.spin(reverse);
    task::sleep(5);
  }
  arm.stop(hold);
  turn_to_heading_tiny(34);
  chassis.drive_distance(37);

  //turn_to_heading_tiny(28);
 
}

/// @brief Raise the am to receive ring
void arm_get(){
  gotoReceiveRingPosition();
}

/// @brief Red Win Point Auto
void red_wp_auto() {
  chassis.drive_distance(-16);
  turn_to_heading_large(90);
}

/**
 * Should swing in a fun S shape.
 */
void swing_test(){
  chassis.left_swing_to_angle(90);
  //chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(1000);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}


/* ***************************************** */
/* ***************************************** */
/* ***************************************** */
/* Autos from 11/16/2024 Ignite Event        */
/* ***************************************** */
/* ***************************************** */
/* ***************************************** */

//Red Right Side Auton for Quals -
//1. Grab Mogo
//2. Score Preload
//3. Turn Left 90
//4. Get A disc
//5. Score that
//6. Touch Ladder
void QRedRight() {
  auto_started = true;

  //Set Ring Filtering to filter Blue rings
  rejectRed = false;
  //Drive to Mogo
  chassis.drive_with_voltage(-6,-6);
  wait(600, msec);
  chassis.drive_with_voltage(-3.5,-3.5);
  wait(900, msec);
  chassis.drive_stop(vex::brake);

  //Clamp Mogo and score preload
  clampMogo();
  wait(250, msec);  //wait fo mogo to settle, then scoer disc
  intakeAndConveyor.spin(forward);
  wait(250, msec);

  //Now turn Left about 100 degreees
  chassis.drive_with_voltage(6, -6);
  wait(450, msec);
  chassis.drive_stop(vex::hold);

  //Intake is already spinning, drive to get the 2nd disc
  chassis.drive_with_voltage(6,6);
  wait(950, msec);

  //Now Right Turn towards Ladder (turn #1)
  chassis.drive_with_voltage(-8, 8);
  wait(550, msec);
  //Drive forwad a bit
  chassis.drive_with_voltage(7.5, 7.5);
  wait(750, msec);

  //Now turn a bit more to the right to line up with ladder
  chassis.drive_with_voltage(-4, 4);
  wait(175, msec);

  //Stop intakeAndConveyor then raise arm
  intakeAndConveyor.stop();
  arm.setVelocity(60, percent);
  while(armRotation.position(degrees) < 120) { arm.spin(reverse); wait(5, msec);}
  arm.stop(hold); 
  
  //Drive to ladder
  chassis.drive_with_voltage(3, 3);
  wait(3000, msec);

  chassis.drive_stop(coast);
}

//Red Right Side Auton for Elims -
//1. Grab Mogo
//2. Score Preload
//3. Turn Left 90
//4. Get A disc
//5. Score that
//6. Orient towards Ladder
void ERedRight() {
  auto_started = true;

  //Set Ring Filtering to filter Blue rings
  rejectRed = false;
  //Drive to Mogo
  chassis.drive_with_voltage(-6,-6);
  wait(600, msec);
  chassis.drive_with_voltage(-3.5,-3.5);
  wait(900, msec);
  chassis.drive_stop(vex::brake);

  //Clamp Mogo and score preload
  clampMogo();
  wait(250, msec);  //wait fo mogo to settle, then scoer disc
  intakeAndConveyor.spin(forward);
  wait(250, msec);

  //Now turn Left about 100 degreees
  chassis.drive_with_voltage(6, -6);
  wait(450, msec);
  chassis.drive_stop(vex::hold);

  //Intake is already spinning, drive to get the 2nd disc
  chassis.drive_with_voltage(6,6);
  wait(950, msec);

  //Now Right Turn towards Ladder (turn #1)
  chassis.drive_with_voltage(-8, 8);
  wait(550, msec);
  //Drive forwad a bit
  chassis.drive_with_voltage(7.5, 7.5);
  wait(750, msec);

  //Stop intakeAndConveyor then raise arm
  intakeAndConveyor.stop();
  chassis.drive_stop(brake);
}

//Blue Left Side Auton for Quals
//1. Grab Mogo
//2. Score Preload
//3. Turn Right 90
//4. Get A disc
//5. Score that
//6. Touch Ladder
void QBlueLeft() {
  auto_started = true;

  //Set Ring Filtering to filter Blue rings
  rejectRed = true;
  //Drive to Mogo
  chassis.drive_with_voltage(-6,-6);
  wait(600, msec);
  chassis.drive_with_voltage(-3.5,-3.5);
  wait(900, msec);
  chassis.drive_stop(vex::brake);

  //Clamp Mogo and score preload
  clampMogo();
  wait(250, msec);  //wait fo mogo to settle, then scoer disc
  intakeAndConveyor.spin(forward);
  wait(250, msec);

  //Now turn Right about 100 degreees
  chassis.drive_with_voltage(-6, 6);
  wait(450, msec);
  chassis.drive_stop(vex::hold);

  //Intake is already spinning, drive to get the 2nd disc
  chassis.drive_with_voltage(6,6);
  wait(950, msec);

  //Now Left Turn towards Ladder (turn #1)
  chassis.drive_with_voltage(8, -8);
  wait(550, msec);
  //Drive forwad a bit
  chassis.drive_with_voltage(7.5, 7.5);
  wait(750, msec);

  //Now turn a bit more to the left to line up with ladder
  chassis.drive_with_voltage(4, -4);
  wait(175, msec);

  //Stop intakeAndConveyor then raise arm
  intakeAndConveyor.stop();
  arm.setVelocity(60, percent);
  while(armRotation.position(degrees) < 120) { arm.spin(reverse); wait(5, msec);}
  arm.stop(hold); 
  
  //Drive to ladder
  chassis.drive_with_voltage(3, 3);
  wait(3000, msec);

  chassis.drive_stop(coast);
}

//Blue Left Side Auton for Elims
//1. Grab Mogo
//2. Score Preload
//3. Turn Right 90
//4. Get A disc
//5. Score that
//6. orient towards ladder
void EBlueLeft() {
  auto_started = true;

  //Set Ring Filtering to filter Blue rings
  rejectRed = true;
  //Drive to Mogo
  chassis.drive_with_voltage(-6,-6);
  wait(600, msec);
  chassis.drive_with_voltage(-3.5,-3.5);
  wait(900, msec);
  chassis.drive_stop(vex::brake);

  //Clamp Mogo and score preload
  clampMogo();
  wait(250, msec);  //wait fo mogo to settle, then scoer disc
  intakeAndConveyor.spin(forward);
  wait(250, msec);

  //Now turn Right about 100 degreees
  chassis.drive_with_voltage(-6, 6);
  wait(450, msec);
  chassis.drive_stop(vex::hold);

  //Intake is already spinning, drive to get the 2nd disc
  chassis.drive_with_voltage(6,6);
  wait(950, msec);

  //Now Left Turn towards Ladder (turn #1)
  chassis.drive_with_voltage(8, -8);
  wait(550, msec);
  //Drive forwad a bit
  chassis.drive_with_voltage(7.5, 7.5);
  wait(750, msec);

  //Stop intakeAndConveyor
  intakeAndConveyor.stop();
  chassis.drive_stop(coast);
}

