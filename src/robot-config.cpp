#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

digital_out Frame(Brain.ThreeWirePort.A);

// motor LA = motor(PORT15, ratio6_1, false);
// motor LB = motor(PORT14, ratio6_1, false);
// motor_group LM = motor_group (LA, LB);

// motor RA = motor(PORT20, ratio6_1, true);
// motor RB = motor(PORT19, ratio6_1, true);
// motor_group RM = motor_group (RA, RB);

// motor Intake = motor(PORT21, ratio6_1, true);
// motor Chamber = motor(PORT3, ratio6_1, true);
// motor Goal = motor (PORT2, ratio6_1, true);


//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:


void vexcodeInit( void ) {
  // nothing to initialize
}   