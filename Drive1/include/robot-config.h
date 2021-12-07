using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LF;
extern motor LB;
extern motor RF;
extern motor RB;
extern motor intake;
extern motor arm;
extern inertial Inertial;
extern motor tilter;
extern motor angler;
extern digital_out Piston;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );