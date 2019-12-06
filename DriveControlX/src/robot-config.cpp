#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor intakeLeft = motor(PORT15, ratio18_1, false);
motor intakeRight = motor(PORT5, ratio18_1, true);
controller Controller1 = controller(primary);
motor pushOut = motor(PORT7, ratio18_1, true);
motor lf = motor(PORT11, ratio18_1, false);
motor lb = motor(PORT19, ratio18_1, false);
motor rf = motor(PORT1, ratio18_1, true);
motor rb = motor(PORT10, ratio18_1, true);
motor lift = motor(PORT16, ratio18_1, false);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}