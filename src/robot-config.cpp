#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//A global controller object
controller Controller1 = controller(primary);

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);
motor LF = motor(PORT8, ratio6_1, false);
motor LT = motor(PORT7, ratio6_1, true);
motor LB = motor(PORT9, ratio6_1, false);

motor RF = motor(PORT3, ratio6_1, true);
motor RT = motor(PORT4, ratio6_1, false);
motor RB = motor(PORT2, ratio6_1, true);

//Add your devices below, and don't forget to do the same in robot-config.h:
motor conveyor = motor(PORT5, ratio6_1, false);
motor intake = motor(PORT14, ratio18_1, false);
motor_group intakeAndConveyor = motor_group(conveyor, intake);
motor arm = motor(PORT13, ratio18_1, true);

rotation odomX = rotation(PORT10, true);
rotation odomY = rotation(PORT1, false);
rotation armRotation = rotation(PORT20, false);

digital_out mogo = digital_out(Brain.ThreeWirePort.H);
digital_out doinker = digital_out(Brain.ThreeWirePort.G);

optical myOptical = optical(PORT12);

inertial myInertial = inertial(PORT6);

distance backDistanceSensor = distance(PORT19);

void vexcodeInit( void ) {
  // nothing to initialize

  //Set Drivetrain properties - Left Motors
  //Left Side  
  LF.setVelocity(100, percentUnits::pct);
  LF.setMaxTorque(100, percentUnits::pct);
  LF.setStopping(brakeType::coast);
  LT.setVelocity(100, percentUnits::pct);
  LT.setMaxTorque(100, percentUnits::pct);
  LT.setStopping(brakeType::coast);
  LB.setVelocity(100, percentUnits::pct);
  LB.setMaxTorque(100, percentUnits::pct);
  LB.setStopping(brakeType::coast);

  //Set Drivetrain properties - Right Motors
  //Right Side  
  RF.setVelocity(100, percentUnits::pct);
  RF.setMaxTorque(100, percentUnits::pct);
  RF.setStopping(brakeType::coast);
  RT.setVelocity(100, percentUnits::pct);
  RT.setMaxTorque(100, percentUnits::pct);
  RT.setStopping(brakeType::coast);
  RB.setVelocity(100, percentUnits::pct);
  RB.setMaxTorque(100, percentUnits::pct);
  RB.setStopping(brakeType::coast);

  //Set Conveyor Speed and Torque
  conveyor.setVelocity(100, percentUnits::pct);
  conveyor.setMaxTorque(100, percentUnits::pct);
  conveyor.setStopping(brakeType::coast);

  //Set Intake Speed and Torque
  intake.setVelocity(100, percentUnits::pct);
  intake.setMaxTorque(100, percentUnits::pct);
  intake.setStopping(brakeType::coast);

  //Set Arm speed and Torque
  arm.setVelocity(100, percentUnits::pct);
  arm.setMaxTorque(100, percentUnits::pct);
  arm.setStopping(brakeType::hold);

  //Set Arm Rotation Sensor position
  armRotation.setPosition(0, rotationUnits::deg);

  //Turn on Optical Sensor
  myOptical.gestureDisable();
  myOptical.setLight(ledState::on);
  myOptical.setLightPower(100,percentUnits::pct);

  //Set Mogo to Opened
  mogo.set(false);
  
  //Set Doinker to closed
  doinker.set(true);

  //Calibrate the inertial
  myInertial.startCalibration();
  while (myInertial.isCalibrating()) { task::sleep(50); }
}