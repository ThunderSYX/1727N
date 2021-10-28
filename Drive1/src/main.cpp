/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Shiqi Yang                                                */
/*    Created:      Thu Sep 14 2021                                           */
/*    Description:  1727N Drive Program                                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LF                   motor         9               
// LB                   motor         2               
// RF                   motor         19              
// RB                   motor         18              
// intake               motor         7               
// arm                  motor         14              
// Inertial             inertial      10              
// tilter               motor         15              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;
competition Competition;
 
motor_group leftDrive(LF, LB);
motor_group rightDrive(RF, RB);
motor_group tankDrive(LF, LB, RF, RB);

bool intakeTrue = false;
const int scale = 120;
void pre_auton(void){
vexcodeInit();
}

void resetEncoders(){
  tankDrive.setPosition(0, degrees);
  arm.setPosition(0, degrees);
  tilter.setPosition(0, degrees);
}

/////=================================== Driver Control  ===================================/////

void userDrive(){
rightDrive.spin(fwd, Controller1.Axis2.position(), pct);
leftDrive.spin(fwd, Controller1.Axis3.position(), pct);
//RB.spin(fwd, Controller1.Axis2.position(), pct);
//LB.spin(fwd, Controller1.Axis3.position(), pct);
}

void armControl(int percent){
  if (Controller1.ButtonR1.pressing()){
    arm.spin(fwd, percent, pct);
  }
  else if (Controller1.ButtonR2.pressing()){
    arm.spin(directionType::rev, percent, pct);
  }
  else {
    arm.stop(brake);
  }
}

void armMacro(int mid, int full){
  if(Controller1.ButtonB.pressing()){
    arm.spinToPosition(0, degrees);
  }
  else if(Controller1.ButtonA.pressing()){
    arm.spinToPosition(-1*mid, degrees);
  }
  else if(Controller1.ButtonX.pressing()){
    arm.spinToPosition(-1*full, degrees);
  }
  else{
    arm.setStopping(hold);
  }
}

void tilterMacro(int down, int up)
{
  if(Controller1.ButtonDown.pressing()){
    tilter.spinToPosition(up, degrees);
  }
  else if(Controller1.ButtonUp.pressing()){
    tilter.spinToPosition(down, degrees);
  }
  else{
    tilter.setStopping(hold);
  }
}

void intakeControl(int percent){
  if (Controller1.ButtonL1.pressing()){
   intakeTrue = true;
  }
  else if (Controller1.ButtonL2.pressing()){
   intakeTrue = false;
   intake.stop();
  }
  if (intakeTrue){
   intake.spin(fwd, percent, pct);
  }
  else if(Controller1.ButtonY.pressing()) {
    intake.spin(directionType::rev, 100, pct);
  }
  else {
    intake.setStopping(coast);
  }
}


/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
/////=================================== Auton Control  ===================================/////


void setTank(double l, double r){
  leftDrive.spin(fwd, l*scale, voltageUnits::mV);
  //LB.spin(fwd, l*scale, voltageUnits::mV);
  rightDrive.spin(fwd, r*scale, voltageUnits::mV);
  //RB.spin(fwd, r*scale, voltageUnits::mV);
}

void tank(double deg){
  tankDrive.spinToPosition(deg, degrees);
}
 
void tankTest(double l, double r){
  LF.spin(fwd, l, voltageUnits::volt);
  LB.spin(fwd, l, voltageUnits::volt);
  RF.spin(fwd, r, voltageUnits::volt);
  RB.spin(fwd, r, voltageUnits::volt);
}

void turn90(int dir){
  setTank(80*dir, -80*dir);
  wait(400, msec);
  setTank(0, 0);
}
 
double gkP = 0.05;
 
void gturn(double angle){
  while(true){
    double gError = angle - Inertial.orientation(yaw, degrees);
    if (gError < 3 && gError > -3){
      return;
    }
    double speed = gError * gkP;
    tankTest(-0.5*speed, 0.5*speed);
  }
}

void holdBrake(){
  tankDrive.setStopping(hold);
}

void coastBrake(){
  tankDrive.setStopping(coast);
}

double avgPosition(){
  return (LF.position(degrees) + LB.position(degrees) + RF.position(degrees) + RB.position(degrees))/4;
}


/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
///// =================================== PID Control  =================================== /////


bool enablePID = false;
bool resetDrive = false;

double desiredValue;
double desiredTurn;

double kP = 0.05;
double kI = 0.0;
double kD = 0.0;

double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

double error;
double totalError = 0;
double prevError = 0;
double drv;

double turnError;
double turnTotalError = 0;
double turnPrevError = 0;
double turnDrv;

int PID(){
  while(enablePID){
    if (resetDrive) {
      resetDrive = false;
      LF.setPosition(0,degrees);
      LB.setPosition(0,degrees);
      RF.setPosition(0,degrees);
      RB.setPosition(0,degrees);
    }

  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Lateral    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////

  error = avgPosition() - desiredValue;
  totalError += error;
  drv = error - prevError;
  double lateralPower = (error * kP + totalError * kI + drv * kD);

  ////================================================================////
  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Turning    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////

  /*turnError = avgPosition() - desiredValue;
  turnTotalError += turnError;
  turnDrv = turnError - turnPrevError;
   double turnPower = (turnError * kP + turnTotalError * kI + turnDrv * kD);*/

  ////================================================================////

  //setTank(lateralPower + turnPower, lateralPower - turnPower);
  tankTest(lateralPower, lateralPower);
  //leftTank(lateralPower + turnPower);
  //rightTank(lateralPower - turnPower);
  prevError = error;
  turnPrevError = turnError;
  vex::task::sleep(20);
  }
  return 1; 
}

/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////

void autonomous(void){
  tank(1800);
  gturn(90.0);

//vex::task drivePID(PID);
//resetDrive = true;
//desiredValue = 100.0;
//desiredTurn = 0.0;
 
//vex::task::sleep(50);

/*
holdBrake();
setTank(-127, -127);
wait(300, msec);
setTank(0, 0);
wait(300, msec);
setIntake(127);
turn90(-1);
wait(300, msec);
setTank(-127,-127);
wait(300, msec);
turn90(1);
wait(300, msec);
setTank(80, 80);
wait(300, msec);
setTank(0,0);
*/

}
void usercontrol(void){
while (1){
  userDrive();
  armMacro(150, 600);
  tilterMacro(600, 250);
  armControl(50);
  intakeControl(90);
  wait(20, msec);
 }
}
 
//double x = Inertial.orientation(roll, degrees);
//double y = Inertial.orientation(pitch, degrees);
int main(){
  Brain.Screen.setFont(mono40);
  resetEncoders();
  Inertial.startCalibration();
  vex::this_thread::sleep_for(2000);
  while(1){
    //Brain.Screen.printAt( 10, 50, "Angle %6.1f", Inertial.orientation(yaw, degrees));

    //Brain.Screen.printAt( 10, 50, "Arm %6.1f", arm.position(degrees));

    Brain.Screen.printAt( 10, 50, "LF %6.1f", LF.position(degrees));
    Brain.Screen.printAt( 10, 200, "LB %6.1f", LB.position(degrees));
    Brain.Screen.printAt( 300, 50, "RF %6.1f", RF.position(degrees));
    Brain.Screen.printAt( 300, 200, "RB %6.1f", RB.position(degrees));
    vex::this_thread::sleep_for(50);
  }
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true){
    wait(100, msec);
  }
}

///// =================================== Unused Bin  =================================== /////
/*
void setDrive(int units, int voltage){
int direction = abs(units)/units;
resetEncoders();
Gyro.resetRotation();
while(avgPosition()< abs(units)){
  setTank(voltage * direction + Gyro.rotation(degrees), voltage * direction - Gyro.rotation(degrees));
  this_thread::sleep_for(50);
}
setTank(0, 0);
}

void initialize(){
coastBrake();
intake.setStopping(hold);
armHold();
}

void gyroTurn(int angle){
int target = Gyro.value(rotationUnits::deg) + angle;
int error = 0;
do{
  error = Gyro.value(rotationUnits::deg) - target;
  if(error < 0){
  //turns right
  setTank(0, 70);
  }
  else{
  //turns left
  setTank(70, 0);
  }
}
while(abs(error) < 10);
holdBrake();
}

*/
 
 
 

