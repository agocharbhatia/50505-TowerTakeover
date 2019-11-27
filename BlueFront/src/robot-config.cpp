#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor lf = motor(PORT1, ratio18_1, false);
motor lb = motor(PORT2, ratio18_1, false);
motor rf = motor(PORT3, ratio18_1, true);
motor rb = motor(PORT4, ratio18_1, true);
motor pushOut = motor(PORT5, ratio18_1, false);
motor intakeLeft = motor(PORT6, ratio18_1, false);
motor intakeRight = motor(PORT7, ratio18_1, true);
motor lift = motor(PORT8, ratio18_1, false);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}