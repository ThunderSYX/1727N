#include "main.h"

vex::brain Brain;
// vex::motor Lift(vex::PORT5, vex::gearSetting::ratio36_1, true);
// vex::motor Tilt(vex::PORT6, vex::gearSetting::ratio36_1, true);
vex::motor Intake (vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor RF (vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor RM (vex::PORT19, vex::gearSetting::ratio18_1, true);
vex::motor RB (vex::PORT15, vex::gearSetting::ratio18_1, false);
vex::motor LF (vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor LM (vex::PORT19, vex::gearSetting::ratio18_1, true);
vex::motor LB (vex::PORT15, vex::gearSetting::ratio18_1, false);

// vex::motor_group intake(LeftIntake, RightIntake);
vex::motor_group leftDrive(LF, LM, LB); // l stands for Left Side motor group
vex::motor_group rightDrive(RF, RM, RB); // r stands for Right Side motor group
vex::motor_group ld(LF, RB); // ld stands for Left Diagonal motor group
vex::motor_group rd(RF, LB); // rd stands for Right Diagonal motor group
vex::motor_group d(LF, LM, LB, RF, RM, RB);
vex::inertial Inertial(vex::PORT18);
// vex::gyro Gyro (Brain.ThreeWirePort.G);
vex::pot tilt(Brain.ThreeWirePort.G);
vex::pot lift(Brain.ThreeWirePort.F);
// vex::encoder le(Brain.ThreeWirePort.A); // Left Encoder
// vex::encoder re(Brain.ThreeWirePort.C); // Right Encoder
vex::encoder xe(Brain.ThreeWirePort.A); // X-axis Encoder
vex::encoder ye(Brain.ThreeWirePort.C); // Y-axis Encoder
vex::rotation xr(vex::PORT1); // X-axis Rotation Sensor
vex::rotation yr(vex::PORT2); // X-axis Rotation Sensor
vex::limit Test (Brain.ThreeWirePort.B); // Test Button
vex::limit Debug (Brain.ThreeWirePort.D); // Debug Button
vex::smartdrive sdt(leftDrive, rightDrive, Inertial, 12.56, 9, 10, distanceUnits::in);
vex::drivetrain dt(leftDrive, rightDrive);
vex::controller Controller1;
vex::competition Competition;
ACCESS_OS os;
ROBOT robot;

int liftMax = 41;
int liftTowerMid = 37;
int liftTowerLow = 30;
int liftMin = 11;
int tiltMax = 80;
int tiltStack = 70;
int tiltMin = 32; 
bool isStacking = false;
double driveSpeedFactor = 1;
double turnSpeedFactor = 1;
double tileInch = 23.6;
double trackWidth = 9; // inches
double wheelBase = 10; // inches
double baseDiagonal = 13.45; // inches