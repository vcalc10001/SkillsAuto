#pragma once
//#include "vex.h"
#include "v5.h"
#include "v5_vcs.h"
#include "vex_imu.h"

using namespace vex;

//Mogo Functions
void clampMogo(void);
void releaseMogo(void);

//Doinker Functions
void lowerDoinker(void);
void raiseDoinker(void);

//Conveyor Functions
void spinConveyorForward(void);
void spinConveyorReverse(void);

//Arm Functions
void gotoReceiveRingPosition(void);
void rotateArmForward(void);
void rotateArmBack(void);
void lockRing(void);

//Color sorting Function
void checkAndFilterBadRing(void);
