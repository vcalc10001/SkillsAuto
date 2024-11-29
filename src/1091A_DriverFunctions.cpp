#include "vex.h"

/*---------------------------------------------------------------------------*/
/*  These are our Driver control Functions                                   */
/*---------------------------------------------------------------------------*/
//Mogo Functions
void clampMogo(void) {
  mogo.set(true);
}

void releaseMogo(void) {
  mogo.set(false);
}

//Doinker Functions
void lowerDoinker(void) {
  doinker.set(false);
}

void raiseDoinker(void) {
   doinker.set(true);;
}

//Conveyor Functions

//Spins Intake and conveyor forward (take in rings and score); while looking for ring to reject
void spinConveyorForward(void) {
  while (Controller1.ButtonL1.pressing()) {
    intakeAndConveyor.spin(forward);
    
    //When color sensor sees the the rings we want to reject, stop conveyor momentarily so that the ring flies off
    if(rejectRed) { //Reject Red Rings
      if (myOptical.hue() < 25) { //Red Hue is generally around 20 or lower
        task::sleep(8); //Wait a bit before stopping conveyor
        intakeAndConveyor.stop();
        waitUntil((myOptical.hue() > 25));      
        intakeAndConveyor.spin(forward);
      }
    }
    else {  //Reject Blue Rings
      if (myOptical.hue() > 150) {  //Blue Hue is generally 210 or higher
        task::sleep(8); //Wait a bit before stopping conveyor
        intakeAndConveyor.stop();
        waitUntil((myOptical.hue() < 150));      
        intakeAndConveyor.spin(forward);
      }
    }
    task::sleep(5);
  }
  intakeAndConveyor.stop();
}

//Spins Intake and conveyor in reverse
void spinConveyorReverse(void) {
  while (Controller1.ButtonL2.pressing()) {
    intakeAndConveyor.spin(reverse);
  task::sleep(5);
  }
  intakeAndConveyor.stop();
}

//Arm Functions
// Bring arm to the position to receive the ring
void gotoReceiveRingPosition(void) {
  arm.setStopping(brakeType::hold);
  arm.setVelocity(50.0, percent);
  while (armRotation.position(degrees) < 24.0) {  //was 32
    arm.spin(reverse);
    task::sleep(5);
  }
  arm.stop(hold);
}

// Raise the Arm (when Up Button is pressed)
void rotateArmForward(void) {
  arm.setStopping(brakeType::hold);
  arm.setVelocity(80.0, percent); //Above 60% RPM, torque drops (but we need speed too)
  while ((Controller1.ButtonUp.pressing() && armRotation.position(degrees) < 150.0)) {
    arm.spin(reverse);
  task::sleep(5);
  }
  arm.stop(hold);
}

// Rotate Arm back when down button is pressed
void rotateArmBack(void) {
  arm.setStopping(brakeType::hold);
  arm.setVelocity(100.0, percent);
  while (Controller1.ButtonDown.pressing()) {
    arm.spin(forward);
    if (armRotation.position(degrees) < 20.0) {
      arm.setStopping(brakeType::coast);
    }
  task::sleep(5);
  }
  arm.stop(coast);
}

// Hit the ring a few times to lock it into the arm (when right button is pressed)
void lockRing(void) {
  if (armRotation.position(degrees) > 15.0 && armRotation.position(degrees) < 40.0) { //Only do this if the arm is in ring receiving position
  /*
    for(int ii = 0; ii < 1; ii++) {
      conveyor.spinFor(reverse, 100.0 - (ii*15.0), degrees, true); //Spin back less each time we doink the ring (since the ring would have moved forward a bit in the arm with each doink)
      conveyor.spin(forward);
      task::sleep(300);
    }
  */
    conveyor.spinFor(reverse, 67.5, degrees, false); //At the end spin conveyor back a bit so that conveyor hook gets out of the ring's way
    //conveyor.stop();
  }
}