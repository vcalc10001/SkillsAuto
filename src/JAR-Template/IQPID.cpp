/*

// User defined function
void myblockfunction_AdjustHeading_target_tolerane_timeout(double myblockfunction_AdjustHeading_target_tolerane_timeout__target, double myblockfunction_AdjustHeading_target_tolerane_timeout__tolerane, double myblockfunction_AdjustHeading_target_tolerane_timeout__timeout) {
  ahTime = Brain.Timer.value();
  myblockfunction_CalculateTurnVariables_target(myblockfunction_AdjustHeading_target_tolerane_timeout__target);
  degreesToTurn = degreesToTurn - myblockfunction_AdjustHeading_target_tolerane_timeout__tolerane;
  if (degreesToTurn > 0.2) {
    if (debugModeOn) {
      Brain.Screen.newLine();
      Brain.Screen.setFont(mono12);
      Brain.Screen.print("AH-D=");
      Brain.Screen.print(printToBrain_numberFormat(), static_cast<float>(degreesToTurn));
      if (IsTurnLeft) {
        Brain.Screen.print("[L]");
      }
      else {
        Brain.Screen.print("[R]");
      }
    }
    MyGyro.setRotation(0.0, degrees);
    // This was 60 RPM.  But after drie train change, it was too fast
    Drivetrain.setTurnVelocity(40.0, rpm);
    while ((fabs(static_cast<float>(MyGyro.rotation())) < degreesToTurn && Brain.Timer.value() - ahTime < myblockfunction_AdjustHeading_target_tolerane_timeout__timeout)) {
      if (IsTurnLeft) {
        Drivetrain.turn(left);
      }
      else {
        Drivetrain.turn(right);
      }
    wait(20, msec);
    }
    Drivetrain.stop();
    if (debugModeOn) {
      Brain_precision = 2;
      Brain.Screen.print(",T=");
      Brain.Screen.print(printToBrain_numberFormat(), static_cast<float>(Brain.Timer.value() - ahTime));
      Brain_precision = 1;
      Brain.Screen.print(",H=");
      Brain.Screen.print(printToBrain_numberFormat(), static_cast<float>(MyGyro.heading()));
    }
  }
  else {
    if (debugModeOn) {
      Brain.Screen.newLine();
      Brain.Screen.setFont(mono12);
      Brain.Screen.print("AH-NothingToDo [H=");
      Brain_precision = 1;
      Brain.Screen.print(printToBrain_numberFormat(), static_cast<float>(MyGyro.heading()));
      Brain.Screen.print("]");
    }
  }
}
*/