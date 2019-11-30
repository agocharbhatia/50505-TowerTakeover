using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor intakeLeft;
extern motor intakeRight;
extern controller Controller1;
extern motor pushOut;
extern motor lf;
extern motor lb;
extern motor rf;
extern motor rb;
extern motor lift;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );