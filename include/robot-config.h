using namespace vex;

extern brain Brain;

extern controller Controller1;


//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;
extern motor LF;
extern motor LT;
extern motor LB;

extern motor RF;
extern motor RT;
extern motor RB;

//Add your devices below, and don't forget to do the same in robot-config.cpp:
extern motor conveyor;
extern motor intake;
extern motor_group intakeAndConveyor;
extern motor arm;

extern rotation odomX;
extern rotation odomY;
extern rotation armRotation;

extern digital_out mogo;
extern digital_out doinker;

extern optical myOptical;

extern inertial myInertial;

extern distance backDistanceSensor;

extern bumper autonSelectorBumper;


void  vexcodeInit( void );

void printAutonMode(void);