#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
digital_out Piston = digital_out(Brain.ThreeWirePort.B);
motor LF = motor(PORT19, ratio18_1, false);
motor LM = motor(PORT11, ratio18_1, true);
motor LB = motor(PORT17, ratio18_1, true);
motor RF = motor(PORT6, ratio18_1, true);
motor RM = motor(PORT18, ratio18_1, false);
motor RB = motor(PORT14, ratio18_1, false);
motor Arm = motor(PORT2, ratio18_1, false);
motor intake = motor(PORT18, ratio18_1, false);
motor tilter = motor(PORT12, ratio18_1, false);
motor angler = motor(PORT3, ratio18_1, false);
digital_out PistonBack = digital_out(Brain.ThreeWirePort.A);
inertial Inertial = inertial(PORT7);

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
