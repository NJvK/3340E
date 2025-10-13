#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(12, 1.8, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
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

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){

default_constants();
chassis.drive_max_voltage = 12;
chassis.drive_distance(24);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  default_constants();
  chassis.Gyro.calibrate();
  while (chassis.Gyro.isCalibrating())
  wait(200, msec);
  chassis.turn_max_voltage = 12;
  chassis.turn_to_angle(180);
  chassis.turn_to_angle(270);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
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
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    //task::sleep(20);
  }
}

/** frr
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








void LongGoalRoller(){
  IntakeRoller.spin(reverse, 100, percent);
  LowerRoller.spin(reverse, 100, percent);
  MiddleRoller.spin(forward, 100, percent);
  UpperRoller.spin(forward, 100, percent);
}
void StopRollers(){
  IntakeRoller.stop();
  LowerRoller.stop();
  MiddleRoller.stop();
  UpperRoller.stop();
}
void HaltRollers(){
  IntakeRoller.spin(reverse, 100, percent);
  LowerRoller.spin(forward, 100, percent);
  MiddleRoller.spin(forward, 100, percent);
  UpperRoller.spin(forward, 100, percent);
}

void DroppAll(){
  IntakeRoller.spin(forward, 100, percent);
  LowerRoller.spin(reverse, 100, percent);
  MiddleRoller.spin(reverse, 100, percent);
  UpperRoller.spin(reverse, 100, percent);
}

void MiddleGoalRoller(){
  IntakeRoller.spin(reverse, 100, percent);
  LowerRoller.spin(reverse, 100, percent);
  MiddleRoller.spin(forward, 100, percent);
  UpperRoller.spin(reverse, 100, percent);
}

// --------------------------------------------------
// I need to shorten these autons because it doesnt run all within 15 seconds
// --------------------------------------------------


void LEFT_RED_AUTON(){
  // // ///////// SET UP ///////////
  // default_constants();
  // chassis.Gyro.calibrate();
  // while (chassis.Gyro.isCalibrating())
  // wait(200, msec);

  // chassis.drive_max_voltage = 9;
  // chassis.turn_max_voltage = 4;


  // //////// Go to get 3 blocks  ///////////  
  // chassis.drive_distance(28);
  // wait(400, msec);
  
  // HaltRollers();
  // // wait(200, msec);

  // chassis.turn_to_angle(326);
  // wait(700, msec);

  // chassis.drive_max_voltage = 2;
  // wait(400, msec);

  // chassis.drive_distance(8);
  // wait(400, msec);
  
  // chassis.turn_to_angle(330);
  // wait(700, msec);

  // chassis.drive_distance(8);
  // wait(400, msec);

  // chassis.turn_to_angle(51);
  // wait(700, msec);

  // StopRollers();
  // // wait(200, msec);

  // chassis.drive_distance(12);
  // wait(400, msec);

  // chassis.turn_to_angle(40);

  // ///// Score 3 blocks /////
  // MiddleGoalRoller();
  // wait(1500, msec);
  // --------------------------------------------------
  ///////// SET UP ///////////
  default_constants();
  chassis.Gyro.calibrate();
  while (chassis.Gyro.isCalibrating())
  wait(200, msec);

  chassis.drive_max_voltage = 9;
  chassis.turn_max_voltage = 7;


  //////// score  1 block  ///////////  
  chassis.drive_distance(45);
  wait(400, msec);

  chassis.turn_to_angle(90);
  wait(700, msec);

  chassis.drive_distance(18);
  wait(200, msec);

  LongGoalRoller();
  wait(1500, msec);

  StopRollers();
  // wait(200, msec);


  ///////// go get 2 blocks ///////////
  chassis.drive_distance(-14);
  wait(400, msec);

  HaltRollers();
  // wait(200, msec);  

  chassis.turn_to_angle(-225);
  wait(200, msec);

  chassis.drive_distance(21);
  wait(200, msec);

  chassis.drive_max_voltage = 2;
  chassis.turn_max_voltage = 2;
  wait(200, msec);

  chassis.drive_distance(6);
  wait(200, msec);

  chassis.drive_distance(-6);
  wait(200, msec);

  chassis.turn_to_angle(-220);
  wait(200, msec);

  
  chassis.drive_distance(12);
  wait(200, msec);

  ///// Score 2 blocks /////
  chassis.turn_to_angle(-224);
  wait(200, msec);

  
  chassis.drive_distance(16);
  wait(200, msec);

  DroppAll();
  // wait(200,msec);
}

void RIGHT_RED_AUTON(){
  ///////// SET UP ///////////
  default_constants();
  chassis.Gyro.calibrate();
  while (chassis.Gyro.isCalibrating())
  wait(200, msec);

  chassis.drive_max_voltage = 9;
  chassis.turn_max_voltage = 5;


  //////// score  1 block  ///////////  
  chassis.drive_distance(45);
  wait(400, msec);

  chassis.turn_to_angle(-90);
  wait(700, msec);

  chassis.drive_distance(18);
  wait(200, msec);

  LongGoalRoller();
  wait(1500, msec);

  StopRollers();
  // wait(200, msec);


  ///////// go get 2 blocks ///////////
  chassis.drive_distance(-14);
  wait(400, msec);

  HaltRollers();
  // wait(200, msec);  

  chassis.turn_to_angle(225);
  wait(200, msec);

  chassis.drive_distance(21);
  wait(200, msec);

  chassis.drive_max_voltage = 2;
  chassis.turn_max_voltage = 2;
  wait(200, msec);

  chassis.drive_distance(6);
  wait(200, msec);

  chassis.drive_distance(-6);
  wait(200, msec);

  chassis.turn_to_angle(220);
  wait(200, msec);

  
  chassis.drive_distance(12);
  wait(200, msec);

  ///// Score 2 blocks /////
  chassis.turn_to_angle(224);
  wait(200, msec);

  
  chassis.drive_distance(16);
  wait(200, msec);

  DroppAll();
  // wait(200,msec);
}
void LEFT_BLUE_AUTON(){

}
void RIGHT_BLUE_AUTON(){

}


// --------------------------------------------------
// I could use some of the above code to make skills becasue I have 60 seconds
// --------------------------------------------------


void SKILLS_AUTON(){

}

void RANDOM_AUTON(){

}