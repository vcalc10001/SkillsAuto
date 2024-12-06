#include "vex.h"

using namespace vex;

competition Competition;


/*
 0 = Skills
 1 = Red WP (4 Rings + Ladder)
 2 = Red Right Side (2 ring + Ladder)
 3 = Red ELIMS (WP w/o Ladder)
 4 = Blue WP (4 rings + Ladder)
 5 = Blue Left Side (2 ring + Ladder)
 6 = Blue ELIMS (WP w/o Ladder)
 7 = Drive Test
 8 = Turn Test
*/
int current_auton_selection = 5;
bool rejectRedRings = true;

bool userControl_started = false;
bool auto_started = false;
double autonStartTime = 0.0;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
//ZERO_TRACKER_NO_ODOM,
TANK_TWO_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LF, LT, LB),

//Right Motors:
motor_group(RF, RT, RB),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT6,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
2.75,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT1,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.00,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT10,

//Sideways tracker diameter (reverse to make the direction switch):
-2.00,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

void printAutonMode() {
    Brain.Screen.setFont(fontType::mono40);
    Brain.Screen.setFillColor(color::black);

    switch(current_auton_selection) {
      case 0:
        Brain.Screen.setFillColor(color::purple);
        Brain.Screen.printAt(5, 200, "SKILLS        ");
        break;
      case 1:
        Brain.Screen.setFillColor(color::red);
        Brain.Screen.printAt(5, 200, "RED WIN POINT ");
        break;
      case 2:
        Brain.Screen.setFillColor(color::red);
        Brain.Screen.printAt(5, 200,"RED RIGHT QUAL ");
        break;
      case 3:
        Brain.Screen.setFillColor(color::red);
        Brain.Screen.printAt(5, 200,"ELIMS RED      ");
        break;
      case 4:
        Brain.Screen.setFillColor(color::blue);
        Brain.Screen.printAt(5, 200,"BLUE WIN POINT ");
        break;
      case 5:
        Brain.Screen.setFillColor(color::blue);
        Brain.Screen.printAt(5, 200,"BLUE LEFT QUAL ");
        break;
      case 6:
        Brain.Screen.setFillColor(color::blue);
        Brain.Screen.printAt(5, 200,"ELIMS BLUE     ");
        break;
      case 7:
        Brain.Screen.setFillColor(color::green);
        Brain.Screen.printAt(5, 200,"DRIVE TEST     ");
        break;
      case 8:
        Brain.Screen.setFillColor(color::green);
        Brain.Screen.printAt(5, 200,"TURN TEST      ");
        break;
      default:
        Brain.Screen.setFillColor(color::black);
        Brain.Screen.printAt(5, 200,"--- NO AUTO ---");
        break;
    }
}

void onAutonSelectorPressed()
{
  if(!auto_started) {
    if(autonSelectorBumper.pressing() > 0) {
      current_auton_selection ++;
      Brain.Screen.clearScreen();
      task::sleep(500);
    }
    if (current_auton_selection == 9) current_auton_selection = 0;
    printAutonMode();
  }
}

int printSensorValues()
{
  Brain.Screen.clearScreen();
  while(true) {
    Brain.Screen.setFont(fontType::mono20);
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.printAt(5,20,"Battery Percentage: %03d", Brain.Battery.capacity()); 

    Brain.Screen.printAt(5, 40,"Chassis Heading Reading: %0.4f", chassis.Gyro.heading());

    Brain.Screen.printAt(5,60,"Color reading: %04d", (int) myOptical.hue());

    Brain.Screen.printAt(5,80,"Front Distance reading: %04d", (int) frontDistanceSensor.objectDistance(distanceUnits::mm));

    Brain.Screen.printAt(5,100,"Back Distance reading: %04d", (int) backDistanceSensor.objectDistance(distanceUnits::mm));

    printAutonMode();
    
    task::sleep(250);
  }
  return 0;
}
/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  Controller1.ButtonR1.pressed(clampMogo);
  Controller1.ButtonR2.pressed(releaseMogo);

  Controller1.ButtonL1.pressed(spinConveyorForward);
  Controller1.ButtonL2.pressed(spinConveyorReverse);

  Controller1.ButtonLeft.pressed(gotoReceiveRingPosition);
  Controller1.ButtonUp.pressed(rotateArmForward);
  Controller1.ButtonDown.pressed(rotateArmBack);
  Controller1.ButtonRight.pressed(lockRing);

  Controller1.ButtonX.pressed(lowerDoinker);
  Controller1.ButtonB.pressed(raiseDoinker);

  autonSelectorBumper.pressed(onAutonSelectorPressed);

  //start a task to continously print sensor values on brain screen on a separate thread
  task printSensorValuesTask = vex::task(printSensorValues, vex::task::taskPrioritylow);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  auto_started = false;
  userControl_started = true;
  Brain.Screen.clearScreen();

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
  userControl_started = false;
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(run_selected_auto);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
