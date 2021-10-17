#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LF = motor(PORT9, ratio18_1, false);
motor LB = motor(PORT2, ratio18_1, false);
motor RF = motor(PORT19, ratio18_1, true);
motor RB = motor(PORT18, ratio18_1, true);
motor intake = motor(PORT7, ratio18_1, false);
motor arm = motor(PORT8, ratio18_1, false);
gyro Gyro = gyro(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}