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
// intakeLeft           motor         4               
// intakeRight          motor         18              
// Controller1          controller                    
// pushOut              motor         3               
// lf                   motor         5               
// lb                   motor         6               
// rf                   motor         7               
// rb                   motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
#include "algorithm"

using namespace vex;

 void intake () {
  intakeLeft.spin(directionType::fwd, 200, velocityUnits::rpm);
  intakeRight.spin(directionType::fwd, 200, velocityUnits::rpm); }
 
 void outake () {
  intakeLeft.spin(directionType::fwd, -200, velocityUnits::rpm);
  intakeRight.spin(directionType::fwd, -200, velocityUnits::rpm);
 }
 
 void pushOutForward() {
   pushOut.rotateTo(2, rev, 120, rpm, false);
 }
 void pushOutBackward() {
  pushOut.rotateTo(-2, rev, 120, rpm, false);
 }

int pushOutVal = 1;

void pushOutMechanism() {

  if(pushOutVal == 1) {
    pushOutForward();
    pushOutVal += 1;
  }
  if(pushOutVal == 2) {
    pushOutBackward();
    pushOutVal = 1;
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
 
intake();
 
 Controller1.ButtonDown.pressed(outake);
 
 Controller1.ButtonUp.pressed(intake);
 
 Controller1.ButtonRight.pressed(pushOutMechanism);

 driveControl();
  
}
