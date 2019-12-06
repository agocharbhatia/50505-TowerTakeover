/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       sahilchhina                                               */
/*    Created:      Mon Nov 25 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// intakeLeft           motor         15              
// intakeRight          motor         5               
// Controller1          controller                    
// pushOut              motor         7               
// lf                   motor         11              
// lb                   motor         19              
// rf                   motor         1               
// rb                   motor         10              
// lift                 motor         16              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
#include "algorithm"

using namespace vex;

 void intake () {
  intakeLeft.spin(directionType::fwd, 80, velocityUnits::rpm);
  intakeRight.spin(directionType::fwd, 80, velocityUnits::rpm); }
 
 void outake () {
  intakeLeft.spin(directionType::fwd, -80, velocityUnits::rpm);
  intakeRight.spin(directionType::fwd, -80, velocityUnits::rpm);
 }

 void intakeMovement() {
   if(Controller1.ButtonR1.pressing()) {
     outake();
   } else if(Controller1.ButtonR2.pressing()) {
     intake();
   } else {
     intakeLeft.stop(hold);
     intakeRight.stop(hold);
   }
 }
 
 void pushOutForward() {
  // pushOut.spin(fwd, 60, rpm);
  pushOut.rotateTo(1.525, rev, 45, rpm, false);
 }
 void pushOutReturn() {
  // pushOut.spin(reverse, 175, rpm);
  pushOut.rotateTo(0, rev, 120, rpm, false);
 }

 void pushOutLift() {
   pushOut.rotateTo(0.9, rev, 45, rpm, false);
 }
 
  void pushOutMechanism() {
    Controller1.ButtonUp.pressed(pushOutForward);
    Controller1.ButtonDown.pressed(pushOutReturn);
    Controller1.ButtonRight.pressed(pushOutLift);

  //   if(Controller1.ButtonRight.pressing()) {
  //   pushOut.spin(fwd, 45, rpm);
  // } else if(Controller1.ButtonLeft.pressing()) {
  //   pushOut.spin(reverse, 45, rpm);
  // } else {
  //   pushOut.stop(hold);
  // }
  }

void liftControl() {
  if(Controller1.ButtonL1.pressing()) {
    lift.spin(fwd, 170, rpm);
  } else if(Controller1.ButtonL2.pressing()) {
    lift.spin(reverse, 170, rpm);
  } else {
    lift.stop(hold);
  }
}

void driveControl() {
  lf.spin(fwd, (Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent)), percent);
  lb.spin(fwd, (Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent)), percent);
  rf.spin(fwd, (Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent)), percent);
  rb.spin(fwd, (Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent)), percent);
}

int main() {

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
 
 while(true) {
 
  Controller1.ButtonR2.pressed(intake);

  Controller1.ButtonR1.pressed(outake);
 
  pushOutMechanism();

  Controller1.ButtonDown.pressed(pushOutReturn);

  liftControl();

  driveControl();

  intakeMovement();
 }
 
  
}
