using namespace vex;

extern brain Brain;
extern digital_out Frame;


//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:

extern motor LA;
extern motor LB;
extern motor_group LM;

extern motor RA;
extern motor RB;
extern motor_group RM;

extern motor LowerRoller;
extern motor UpperRoller;
extern motor MiddleRoller;
extern motor IntakeRoller;

extern controller Controller1;




void  vexcodeInit( void );