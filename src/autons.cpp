#include "vex.h"

/* Test Change */

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
  //chassis.drive_distance(6);
  //chassis.drive_distance(12);
  //chassis.drive_distance(18);
  chassis.drive_distance(30);
  wait(1,seconds);
  chassis.drive_distance(-12);
}

// Turning Test
void turn_test(){
  //turn_to_heading_tiny(20); //20.2
  //turn_to_heading_tiny(30); //30.5
  //turn_to_heading_small(45);  //46.5
  //turn_to_heading_small(60);  //60.5
  //turn_to_heading_medium(75); //77
  //turn_to_heading_medium(90); //90.5
  //turn_to_heading_medium(105);  //106
  //turn_to_heading_large(120); //121.7
  //turn_to_heading_large(150); //151.1
  //turn_to_heading_large(175); //175.9
  //turn_to_heading_xlarge(150); //150.9 (but faster than _large)
  //turn_to_heading_xlarge(175); //175.4 (but faster than _large)
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



/* ********************************* */
/* Bunch of pre-tuned turn functions */
/* ********************************* */
//Use for tiny turns (less than 30 degrees)
/// @brief Make tiny turns.  Use for turns less than 30 degrees
/// @param targetHeading heading that we want to end up at
inline void turn_to_heading_tiny(float targetHeading) { chassis.turn_to_heading_1091A(targetHeading, 8.5, 1.0, 2000, 0.68, 0, 0.40, 5);}

/// @brief Make small turns.  Use for turns 30-60 degrees - Tuned to 45
/// @param targetHeading heading that we want to end up at
inline void turn_to_heading_small(float targetHeading) { chassis.turn_to_heading_1091A(targetHeading, 10.0, 1.0, 2000, 0.68, 0, 0.4, 5);}

/// @brief Make medium turns.  Use for turns 60-120 degrees - tuned to 90
/// @param targetHeading heading that we want to end up at
inline void turn_to_heading_medium(float targetHeading) {chassis.turn_to_heading_1091A(targetHeading, 12, 1.0, 2000, 0.75, 0.05, 0.80, 5); }

/// @brief Make large turns 120-150 degrees
/// @param targetHeading heading that we want to end up at
inline void turn_to_heading_large(float targetHeading) { chassis.turn_to_heading_1091A(targetHeading, 12, 1.0, 2000, 0.90, 0.05, 0.85, 5); }

/// @brief Make large turns >= 150 degrees
/// @param targetHeading heading that we want to end up at
inline void turn_to_heading_xlarge(float targetHeading) { chassis.turn_to_heading_1091A(targetHeading, 12, 1.0, 2000, 1.0, 0.05, 0.85, 5); }


/// @brief adjust the heading after a turn.  Use after calling one of the turn functions when we need precise turns
/// @param targetHeading heading that we want to end up at
/// @param tolerance How much error in the final heading we will accept
/// @param timeout timeout for the function in milliseconds
void adjustHeading(double targetHeading, double tolerance, double timeout) {
  float startingHeading = chassis.Gyro.heading();

  bool isTargetLTCurrent = targetHeading < startingHeading;
  bool isTurnLeft = false;

  //Figure out how many degrees to turn
  float degreesToTurn = fabs(static_cast<float>(startingHeading - targetHeading));
  bool isDiffLT180 = (degreesToTurn < 180.0);

  //If degrees to turn is >180, then normalize it to the opposite angle
  if (!isDiffLT180) degreesToTurn = fabs(static_cast<float>(360.0 - degreesToTurn));

  //Figure out whether to turn left or right
 /* if ((isDiffLT180 && isTargetLTCurrent) || (!isDiffLT180 && !isTargetLTCurrent)) isTurnLeft = false;  // Turn Right
  else isTurnLeft = true; //Turn Left
*/
  if(isDiffLT180) {
    if(isTargetLTCurrent) isTurnLeft = true;
    else isTurnLeft = false;
  }
  else {
    if(isTargetLTCurrent) isTurnLeft = false;
    else isTurnLeft = true;
  }
  
  degreesToTurn = degreesToTurn - tolerance;
  if (degreesToTurn > 0.2) {
    double ahTime = Brain.Timer.value();
    chassis.Gyro.setRotation(0.0, degrees);

    //Turn voltages are opposite of what they should be because our left and right motor configurations are reversed
    if (isTurnLeft) chassis.drive_with_voltage(2.0, -2.0);
    else chassis.drive_with_voltage(-2.0, 2.0);

    while(((Brain.Timer.value() - ahTime)*1000 < timeout) && (fabs(chassis.Gyro.rotation(degrees)) < degreesToTurn -tolerance)) task::sleep(5);
    chassis.drive_stop(hold);
  }
}

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

/// @brief Spin arm up for touching the ladder.  Used to create a task object so that we can do this in parallel with other things
/// @return always returns zero since Vex::task class expects that
int spinArmUpForLadder() {
  while(armRotation.position(degrees) < 145.0) {
     arm.spin(reverse);
     task::sleep(5);
  }
  arm.stop(hold);
  return 0;
}

/// @brief Spin arm back down.  Used to create a task object so that we can do this in parallel with other things
/// @return always returns zero since Vex::task class expects that
int spinArmBackDown() {
  arm.setVelocity(100, percent);
  arm.setStopping(coast);
  arm.spin(forward);  
 
  while(armRotation.position(degrees) > 10.0) {task::sleep(5);}  //wait for arm to get back down
  arm.stop(coast); //stop the arm once we reach about 10 degrees.  The arm will go down by itself rest of the way
  return 0;
}

/// @brief Color Soting loop for autos.  Used to create a task object so that we can do this in parallel with other things
/// @return always returns zero since Vex::task class expects that.  But in eality it ill neve return
int ringSortingAutonTask() {
  while(true) {
    checkAndFilterBadRing();
    task::sleep(5);
  }
  return 0;
}

double autonStartTime = 0.0;
/* Funtion registered to run when Auto is started*/
void run_selected_auton()
{
  autonStartTime = Brain.Timer.value();
  auto_started = true;

  //Start raising the arm on a separate thread
  vex::task colorSortingTask = vex::task(ringSortingAutonTask, vex::task::taskPriorityNormal);

  switch(current_auton_selection) {
    case 0:
      Brain.Screen.setFillColor(color::green);
      Brain.Screen.printAt(5, 200, "SKILLS");
      skills_auto();
      break;
    case 1:
      Brain.Screen.setFillColor(color::red);
      Brain.Screen.printAt(5, 200, "RED WIN POINT");
      red_wp_auto();
      break;
    case 2:
      Brain.Screen.setFillColor(color::red);
      Brain.Screen.printAt(5, 200,"RED RIGHT QUAL");
      red_right_qual_nopid_auto();
      break;
    case 3:
      //Blue Win Point
      Brain.Screen.setFillColor(color::blue);
      Brain.Screen.printAt(5, 200,"BLUE WIN POINT");
      break;
    case 4:
      Brain.Screen.setFillColor(color::blue);
      Brain.Screen.printAt(5, 200,"BLUE LEFT QUAL");
      blue_left_qual_nopid_auto();
      break;
    case 5:
      Brain.Screen.setFillColor(color::red);
      Brain.Screen.printAt(5, 200,"ELIMS RED RUSH");
      // Elims Red Rush
      break;
    case 6:
      // Elims Blue Rush
      Brain.Screen.setFillColor(color::blue);
      Brain.Screen.printAt(5, 200,"ELIMS BLUE RUSH");
      break;
    case 7:
      Brain.Screen.setFillColor(color::purple);
      Brain.Screen.printAt(5, 200,"DRIVE TEST");
      drive_test();
      break;
    case 8:
      Brain.Screen.setFillColor(color::purple);
      Brain.Screen.printAt(5, 200,"TURN TEST");
      turn_test();
      break;
    default:
      //Do Nothing
      Brain.Screen.setFillColor(color::black);
      Brain.Screen.printAt(5, 200,"*** NO AUTO ***");
      break;
  }
  colorSortingTask.stop();
  auto_started = false;
  Brain.Screen.setFont(fontType::mono20);
  Brain.Screen.printAt(5, 140, "TotalTme: %f", Brain.Timer.value() - autonStartTime);
}

void setup_auto() {
  conveyor.setVelocity(100,percent);
  conveyor.setMaxTorque(100,percent);
  conveyor.setStopping(coast);
  intake.setVelocity(100, percent);
  intake.setMaxTorque(100, percent);
  intake.setStopping(coast);
  arm.setVelocity(100, percent);
  arm.setMaxTorque(100,percent);
  chassis.set_heading(0.0);
}

/// @brief Shoot the ring on alliance stake
void shoot_alliance_ring() {
  conveyor.spinFor(directionType::fwd, 0.45,seconds); //was 0.6
  conveyor.spinFor(directionType::rev, 0.15,seconds); //was 0.25
  //conveyor.spin(reverse);
  //wait(0.3,seconds);
  conveyor.stop();
}

/// @brief Raise the am to receive ring
void arm_get(){
  gotoReceiveRingPosition();
}

/// @brief Skills Auto
void skills_auto() {
  //Set Ring Filtering to filter Blue rings
  rejectRed = false;

  setup_auto();
  chassis.drive_max_voltage=9;
  chassis.turn_max_voltage=9;
  arm.setMaxTorque(100,percent);
  arm.setVelocity(100,percent);
  
  
  /*
  shoot_alliance_ring();

  //chassis.drive_timeout=400;
  chassis.drive_distance(3.8);
  turn_to_heading_large(255);
  chassis.drive_timeout=2000;
  chassis.turn_timeout=900;
  chassis.drive_distance(-8);
  chassis.drive_max_voltage=3;
  chassis.drive_distance(-11);

  //Go get the fist ring
  chassis.drive_max_voltage=8;
  clampMogo(); //mogo.set(true);
  task::sleep(50);
  turn_to_heading_medium(0);
  
  chassis.drive_max_voltage=11;
  chassis.turn_max_voltage=10.5;
  intakeAndConveyor.spin(forward); 
  chassis.drive_distance(27.5);
  task::sleep(50);
  chassis.drive_distance(-7); //drive back a bit to make sure we miss the ladder edge when we go to get the next disk

  //go grab the disk into the arm
  turn_to_heading_tiny(26.0);  //was 25.25
  arm_get();
  chassis.drive_distance(50);
  chassis.drive_max_voltage=8;
  chassis.drive_distance(8);
  task::sleep(100);//wait(0.1,seconds);

 
  //aligning to wall stake
  chassis.drive_with_voltage(6, -6);  //First turn to the left bit so we can drive back and turn more easily (and miss the ladder)
  task::sleep(75);
  chassis.drive_stop(hold);
  chassis.drive_distance(-23.0);
  turn_to_heading_small(88.0);  //turn towards the wall
  chassis.drive_with_voltage(4, 4); //drive to the wall
  task::sleep(800); //wait to get to the wall  
  chassis.drive_stop(brake);

  //placing on disk on wall stake
  arm.setVelocity(80.0, pct);
  conveyor.stop(coast);  //stop conveyor first
  arm.spin(reverse);  //spin arm o score disk
  task::sleep(1000);
  //chassis.drive_with_voltage(6, 6); //drive up a bit more to try to get the ing onto the stake
  //task::sleep(300);
  //chassis.drive_stop(coast);

  //Wall stake done, now start spinning arm back down
  arm.setVelocity(100, percent);
  arm.setStopping(coast);
  arm.spin(forward);  

 
 //Now go get the next 4 rings
 //First, drive back, then turn to face the 4 rings in a row and then drive to pacman them up
 chassis.drive_max_voltage = 12.0;
 chassis.drive_distance(-9.5);
 intakeAndConveyor.spin(forward); //Spin conveyor and intake (this will also score the disk already in intake onto the mogo)
 turn_to_heading_medium(176.5);
 
 while(armRotation.position(degrees) > 15.0) {task::sleep(5);}  //wait for arm to get back down
 arm.stop(coast); //stop the arm (should be done coming back down by now)
 //Now drive forwad while intaking (drive a bit slow so rings can get on the mogo)
 chassis.drive_max_voltage = 7.0;
 chassis.drive_timeout = 4000;
 chassis.drive_distance(58);
 wait(0.3,seconds);
 chassis.drive_distance(-19);
 turn_to_heading_tiny(135);
 chassis.drive_distance(8);
 turn_to_heading_large(335);
 chassis.drive_distance(-10);
 mogo.set(false);
 wait(0.1,seconds);
 


 //drive to mogo with 1 blue ring on it

chassis.drive_max_voltage=9;
 chassis.drive_distance(5);
 turn_to_heading_medium(90);
 intake.spin(forward);
 chassis.drive_distance(-62,90);
 chassis.drive_max_voltage=4;
 turn_to_heading_tiny(77);
 chassis.drive_distance(-13);
 mogo.set(true); 
 //picked
 */
 chassis.set_heading(77); //setting heading one time
 mogo.set(true);
 task::sleep(100);
 turn_to_heading_medium(0);
 intake.spin(forward);
 conveyor.spin(forward);
 chassis.drive_max_voltage=8;
 chassis.drive_distance(15.5);
 task::sleep(700);
 arm_get();
 turn_to_heading_small(334); 
 chassis.drive_distance(42.5);
 turn_to_heading_small(285);
 chassis.drive_distance(.5);
 task::sleep(1300);
 conveyor.stop();
 arm.setVelocity(100,percent);
 arm.spin(reverse);  //spin arm o score disk
 task::sleep(1000);
 arm.stop();
 //on stake?


}



/// @brief Red Win Point Auto (Red - left side).  Scores 1 ring on alliance stake, 3 rings on Mogo, and touches ladder
void red_wp_auto() {
  //Set Ring Filtering to filter Blue rings
  rejectRed = false;

  setup_auto();

  //Drive back and point towards alliance stake
  chassis.drive_max_voltage = 6.0;
  chassis.drive_distance(-11.30);   //Drive back (was 11.30)
  turn_to_heading_medium(90.0); //Turn so back of robot is parallel with alliance stake wall (was 87.5) 
  task::sleep(250); //let gyro settle
  if(chassis.Gyro.heading() < 89 || chassis.Gyro.heading() > 91) adjustHeading(90.0, 0.5, 150);  //adjust heading to be as close to 90 as possible (this is a critical turn)
 
  //Now drive backwards to Alliance stake
  chassis.drive_with_voltage(-2.5, -2.5);
  task::sleep(600);
  //while(backDistanceSensor.objectDistance(distanceUnits::mm)>65) task::sleep(5);
  chassis.drive_stop(brake);
  //task::sleep(10);
  
  // Now Shoot the preload ring onto alliance stake
  shoot_alliance_ring();

  //Now drive forward and tun towards the mogo
  //chassis.set_heading(90);  // ********************** THIS IS TESTING CODE - REMOVE IT ***********************
  chassis.drive_max_voltage = 12.0; //first drive straight towads the ladder
  chassis.drive_distance(20.0); //was 20
  turn_to_heading_large(215); //Then turn towads the mogo (was 215)


  //Now drive to the mogo and clamp it (Total distance to mogo = 15-16"")
  chassis.drive_distance(-15);  //fast in initial art of drive, then slow (was -14)
  chassis.drive_max_voltage = 6;  //slow down in the last part of the mogo drive (was 3.5, then 4.0 [when distance was -12])
  chassis.drive_distance(-18); //was -12 @ 4.0v, then -15 @ 5v, then -18 @6v
  clampMogo();
  task::sleep(50);  //Wait a bit to let the mogo settle

  //Start intake and conveyor
  intakeAndConveyor.spin(fwd);

  //Now turn towards Neutral zone line and get a ring (the one closer to the ladder)
  //This is a 2 ste tun to force a left turn
  turn_to_heading_large(90.0);
  task::sleep(50);  //Give gyro time to settle
  turn_to_heading_tiny(45.0);  //Was 38; then 40; was _large; was 42.5 (_larrge)
  //task::sleep(250); //let gyro settle
  //if(chassis.Gyro.heading() > 42 || chassis.Gyro.heading() < 38) adjustHeading(40, 0.5, 100);  //Adjust heading so we are pointed to disk correctly

  //Get first ring next to the neutral zone
  chassis.drive_max_voltage = 12.0; //speed up again
  //chassis.drive_timeout = 1200;
  chassis.drive_distance(16.0); //Was 22.25
  task::sleep(500); //wait a bit to get ring and score it befoe doing the next thing (was 800; then 650) 
  
  //Get alliance side ring 
  chassis.drive_distance(-9); //Drive back a bit (was -9)
  turn_to_heading_small(337.5); //turn towards alliance side ring (was 345) [was _small]
  chassis.drive_distance(12.0); //Drive to alliance side ring (was 12)
  task::sleep(300); //wait a bit to get ring and score it befoe doing the next thing (was 250, then 250) 

  //Get 2nd ring next to the neutral zone
  turn_to_heading_small(50); //Turn toards ring (was 55)
  chassis.drive_distance(12.25); //Drive to ring (was 13; then 12.25)
  task::sleep(500); //wait a bit to get ring and score it befoe doing the next thing (was was  500; then 500)

  //Now turn towads ladder
  chassis.drive_with_voltage(-12, -12); //First drive back a bit so we do not cross the line when turning towards ladder
  task::sleep(225);
  chassis.drive_stop(brake);
  turn_to_heading_large(180); //Now turn towards ladder

  //Start raising the arm on a separate thread
  vex::task armTask(spinArmUpForLadder, vex::task::taskPriorityNormal);

  //Drive to the Ladder while raising the arm
  chassis.drive_stop(coast);  //This sets the chassis stop mode to coast as a safety net
  chassis.drive_with_voltage(4.75, 2.55); //Do a curve drive so we get more parallel to the ladder as we drive

  //Keep diving to the ladder till we run out of time, then stop in coast mode
  while((Brain.Timer.value() - autonStartTime) < 15.0) {
    task::sleep(5);
    //Do Nothing
  }
  //Stop intake and conveyor
  intakeAndConveyor.stop(brakeType::coast);
  chassis.drive_stop(coast);  
}

/* ***************************************** */
/* ***************************************** */
/* ***************************************** */
/* Autos adapted from Ignite Event           */
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
void red_right_qual_nopid_auto() {
  //Set Ring Filtering to filter Blue rings
  rejectRed = false;

  //Drive to Mogo
  chassis.drive_with_voltage(-6,-6);
  task::sleep(750);
  chassis.drive_with_voltage(-3.5,-3.5);
  task::sleep(950);
  chassis.drive_stop(vex::brake);

  //Clamp Mogo and score preload
  clampMogo();
  task::sleep(250);  //wait for mogo to settle, then score disc
 
  //spin intake and conveyor to score the preload
  intakeAndConveyor.spin(forward);

  //Now turn right about 90 degreees
  chassis.drive_with_voltage(6, -6);
  task::sleep(450);
  chassis.drive_stop(vex::hold);
  task::sleep(25);  //settle down before driving

  //Intake is already spinning, drive to get the 2nd disc
  chassis.drive_with_voltage(6,6);
  task::sleep(700);
  chassis.drive_stop(brake);
  task::sleep(500);

  //Now Right Turn towards Ladder (turn #1)
  chassis.drive_with_voltage(-8, 8);
  task::sleep(550);

  //Drive forwad a bit
  chassis.drive_with_voltage(6.5, 6.5);
  task::sleep(750);

  //Stop intakeAndConveyor then raise arm
  intakeAndConveyor.stop();

  //Start raising the arm on a separate thread
  vex::task armTask(spinArmUpForLadder, vex::task::taskPriorityNormal);
 
 //Now turn a bit more to the right to line up perpeidular to the ladder
  chassis.drive_with_voltage(-5, 5);
  task::sleep(325); //was 550
  
  //Drive to ladder
  chassis.drive_with_voltage(2.5, 2.5);
  task::sleep(3000);

  chassis.drive_stop(coast);
}



//Blue Left Side Auton for Quals
//1. Grab Mogo
//2. Score Preload
//3. Turn Right 90
//4. Get A disc
//5. Score that
//6. Touch Ladder
void blue_left_qual_nopid_auto() {
  //Set Ring Filtering to filter Blue rings
  rejectRed = true;

  //Drive to Mogo
  chassis.drive_with_voltage(-6,-6);
  task::sleep(750);
  chassis.drive_with_voltage(-3.5,-3.5);
  task::sleep(950);
  chassis.drive_stop(vex::brake);

  //Clamp Mogo and score preload
  clampMogo();
  task::sleep(250);  //wait for mogo to settle, then score disc
 
  //spin intake and conveyor to score the preload
  intakeAndConveyor.spin(forward);

  //Now turn left about 90 degreees
  chassis.drive_with_voltage(-6, 6);
  task::sleep(450);
  chassis.drive_stop(vex::hold);
  task::sleep(25);  //settle down before driving

  //Intake is already spinning, drive to get the 2nd disc
  chassis.drive_with_voltage(6,6);
  task::sleep(700);
  chassis.drive_stop(brake);
  task::sleep(500);

  //Now Left Turn towards Ladder (turn #1)
  chassis.drive_with_voltage(8, -8);
  task::sleep(550);

  //Drive forwad a bit
  chassis.drive_with_voltage(6.5, 6.5);
  task::sleep(750);

  //Stop intakeAndConveyor then raise arm
  intakeAndConveyor.stop();

  //Start raising the arm on a separate thread
  vex::task armTask(spinArmUpForLadder, vex::task::taskPriorityNormal);

  //Now turn a bit more to the left to line up perpendicular with ladder
  chassis.drive_with_voltage(5, -5);
  task::sleep(250);

  //Drive to ladder
  chassis.drive_with_voltage(2.5, 2.5);
  task::sleep(3000);

  chassis.drive_stop(coast);
}