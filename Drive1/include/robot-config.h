using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern digital_out Piston;
extern motor LF;
extern motor LM;
extern motor LB;
extern motor RF;
extern motor RM;
extern motor RB;
extern motor Arm;
extern motor intake;
extern motor tilter;
extern motor angler;
extern digital_out PistonBack;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );