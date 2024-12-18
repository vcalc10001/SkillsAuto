#include "vex.h"

/* ****************************************************************************** */
/* 1091A's implementation of PID functions on top of the JAR Template Drive class */
/* ****************************************************************************** */


/* ************** */
/* Turn Functions */
/* ************** */
void Drive::turn_to_heading_1091A(float targetHeading) {
  turn_to_heading_1091A(targetHeading, turn_max_voltage, turn_settle_error, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_heading_1091A(float targetHeading, float turn_max_voltage, float turn_settle_error, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti) {
  float startingHeading = Gyro.heading();

  bool isTargetLTCurrent = targetHeading < startingHeading;
  bool isTurnLeft = false;

  //Figure out how many degrees to turn
  float degreesToTurn = fabs(static_cast<float>(startingHeading - targetHeading));
  bool isDiffLT180 = degreesToTurn < 180.0;

  //If degrees to turn is >180, then normalize it to the opposite angle
  if (!isDiffLT180) degreesToTurn = fabs(static_cast<float>(360.0 - degreesToTurn));

  //Figure out whether to turn left or right
  if ((isDiffLT180 && isTargetLTCurrent) || (!isDiffLT180 && !isTargetLTCurrent)) isTurnLeft = false;  // Turn Right
  else isTurnLeft = true; //Turn Left

  //degreesToTurn = degreesToTurn - turn_settle_error;

  if (degreesToTurn > 0.0) {
    Gyro.setRotation(0.0, degrees);
    float degreesTurned = 0.0;
    float error = 0.0;
    float previousError = 0.0;
    float derivative = 0.0;
    float integral = 0.0;
    float currentVolts = static_cast<float>(fabs(turn_max_voltage));
    long loopCount = 0;
    double setTime = Brain.Timer.value();
    double gyroReadingDelayInMSec = 10.0;

    //While we have not timed out, and have not yet turned enough (or have turned too much)
    while ((Brain.Timer.value() - setTime)*1000.0 < turn_timeout \
            && ((degreesTurned < (degreesToTurn - turn_settle_error)) \
             || (degreesTurned > (degreesToTurn + turn_settle_error)))) {
      loopCount++;
      if (isTurnLeft) drive_with_voltage(-currentVolts, currentVolts);  //For left turn, left side is -ve volts, Right side is +ve volts
      else drive_with_voltage(currentVolts, -currentVolts); //For right turn, Left side is +ve volts, Right side is -ve volts

      //Wait a bit before reading the Gyro (but never less than 5 msec)
      if((gyroReadingDelayInMSec / loopCount) < 2.0) task::sleep(2);
      else task::sleep(static_cast<uint32_t>(gyroReadingDelayInMSec / loopCount));

      //Read how much we have turned and normalize the amount turned
      degreesTurned = fabs(static_cast<float>(Gyro.rotation())*(360.0/gyro_scale));
      if (degreesTurned > 180.0) degreesTurned = fabs(static_cast<float>(degreesTurned - 360.0));

      //PID calculations
      error = (degreesToTurn - degreesTurned);
      if (fabs(error) < turn_starti) integral+=error;
      // Check if error has crossed the zero line, and if it has, eliminates the integral term
      // and reverse the turn direction (we only cross the zero line when we are trying to settle into a steady state)
      if ((error>0 && previousError<0)||(error<0 && previousError>0)){ 
        integral = 0; 
        isTurnLeft = !isTurnLeft;
      }
      derivative = error - previousError;
      previousError = error;
      
      //Now calculate the new voltage to give the motors
      currentVolts = static_cast<float>(fabs(turn_max_voltage)) * ((error * turn_kp) + (derivative * turn_kd) + (integral * turn_ki))/degreesToTurn;

      //If the new voltage is below the minimum volts we need, then set the volts to the minimum volts
      if(currentVolts <2.0) currentVolts = 2.0;
    }
    if(isTurnLeft){ //On Left turns, stop the right motors first
      DriveR.stop(hold);
      DriveL.stop(hold);
    }
    else{ //On Right turns, stop the left motors first
      DriveL.stop(hold);
      DriveR.stop(hold);
    }
    task::sleep(10);
  }
}


/* ************** */
/* Drive Functions */
/* ************** */
void Drive::drive_distance_1091A(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);

  float start_average_position = ForwardTracker_diameter*M_PI*R_ForwardTracker.position(degrees)/360.0;
  float average_position = start_average_position;
  float drive_error = distance;
  
  while((fabs(drive_error) >= fabs(drive_settle_error)) || !drivePID.is_settled()){
    average_position = ForwardTracker_diameter*M_PI*R_ForwardTracker.position(degrees)/360.0;
    float drive_remining = (distance+start_average_position-average_position);
    float drive_volts = (drivePID.compute(drive_remining) * drive_max_voltage)/distance;

    drive_volts = clamp(drive_volts, -drive_max_voltage, drive_max_voltage);
    //If the newly calculated drivevolts is less than 2.5, then make it 2.5 (while maintaining the +ve/-ve sign)
    if((drive_volts >= 0.0) && (drive_volts < 2.5)) drive_volts = 2.5;
    else if((drive_volts < 0.0) && (drive_volts > -2.5)) drive_volts = -2.5;

    drive_with_voltage(drive_volts, drive_volts);
    task::sleep(5);
  }
  drive_stop(hold);
}

/*
void Drive::turn_to_heading_1091A_IQBase(float targetHeading, float turn_max_voltage, float turn_settle_error, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti) {
  float startingHeading = Gyro.heading();

  bool isTargetLTCurrent = targetHeading < startingHeading;
  float degreesToTurn = fabs(static_cast<float>(startingHeading - targetHeading));
  bool isDiffLT180 = degreesToTurn < 180.0;
  bool isTurnLeft = false;

  if (!isDiffLT180) {
    degreesToTurn = fabs(static_cast<float>(360.0 - degreesToTurn));
  }
  if ((isDiffLT180 && isTargetLTCurrent) || (!isDiffLT180 && !isTargetLTCurrent)) {
    // Turn Right
    isTurnLeft = false;
  }
  else {
    // Turn Left
    isTurnLeft = true;
  }

  degreesToTurn = degreesToTurn - turn_settle_error;
  if (degreesToTurn > 0.0) {
    Gyro.setRotation(0.0, degrees);
    float degreesTurned = 0.0;
    float error = 0.0;
    float previousError = 0.0;
    float derivative = 0.0;
    float integral = 0.0;
    float currentVolts = static_cast<float>(fabs(turn_max_voltage));
    long loopCount = 0;
    double setTime = Brain.Timer.value()*1000.0;
    double gyroReadingDelayInMSec = 10.0;

    while ((degreesTurned < degreesToTurn && ((Brain.Timer.value()*1000.0) - setTime) < turn_timeout)) {
      loopCount++;
      if (isTurnLeft) drive_with_voltage(-currentVolts, currentVolts);  //For left turn, left side is -ve volts, Right side is +ve volts
      else drive_with_voltage(currentVolts, -currentVolts); //For right turn, Left side is +ve volts, Right side is -ve volts

      //Wait a bit before reading the Gyro (but never less than 5 msec)
      if((gyroReadingDelayInMSec / loopCount) < 2.0) task::sleep(2);
      else task::sleep(uint(gyroReadingDelayInMSec / loopCount));

      //Read hoiw much we have turned and normalize the amount turned
      degreesTurned = fabs(static_cast<float>(Gyro.rotation())*(360.0/gyro_scale));
      if (degreesTurned > 180.0) degreesTurned = fabs(static_cast<float>(degreesTurned - 360.0));

      //PID calculations
      error = (degreesToTurn - degreesTurned);
      if (fabs(error) < turn_starti) integral+=error;
      // Checks if the error has crossed 0, and if it has, it eliminates the integral term.
      if ((error>0 && previousError<0)||(error<0 && previousError>0)){ 
        integral = 0; 
      }
      derivative = error - previousError;
      previousError = error;
      
      //Now calculate the new voltage to give the motors
      currentVolts = static_cast<float>(fabs(turn_max_voltage)) * ((error * turn_kp) + (derivative * turn_kd) + (integral * turn_ki))/degreesToTurn;

      //If the new voltage is below the minimum volts we need, then set the volts to the minimum volts
      if(currentVolts <2.0) currentVolts = 2.0;
      
      //give it a rest before going back to the loop
      //task::sleep(5);
    }
    if(isTurnLeft){ 
      DriveR.stop(hold);
      DriveL.stop(hold);
    }
    else{
      DriveL.stop(hold);
      DriveR.stop(hold);
    }
  }
}
*/