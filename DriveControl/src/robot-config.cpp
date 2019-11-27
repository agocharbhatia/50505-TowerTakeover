#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor intakeLeft = motor(PORT4, ratio18_1, false);
motor intakeRight = motor(PORT18, ratio18_1, true);
controller Controller1 = controller(primary);
motor pushOut = motor(PORT3, ratio18_1, false);
motor lf = motor(PORT5, ratio18_1, false);
motor lb = motor(PORT6, ratio18_1, false);
motor rf = motor(PORT7, ratio18_1, true);
motor rb = motor(PORT8, ratio18_1, true);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}