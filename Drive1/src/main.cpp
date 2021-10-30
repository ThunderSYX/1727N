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
// Inertial             inertial      11              
// tilter               motor         15              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;
competition Competition;
 
motor_group leftDrive(LF, LB);
motor_group rightDrive(RF, RB);
motor_group tankDrive(LF, LB, RF, RB);

bool intakeTrue = false;
bool toggle = false;
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
  if(toggle == false){
    rightDrive.spin(fwd, Controller1.Axis2.position(), pct);
    leftDrive.spin(fwd, Controller1.Axis3.position(), pct);
  }
  else if(toggle == true){
    rightDrive.spin(reverse, Controller1.Axis3.position(), pct);
    leftDrive.spin(reverse, Controller1.Axis2.position(), pct);
  }
  if(Controller1.ButtonLeft.pressing()){
    toggle = !toggle;
  }
}

void armControl(int percent){
  if (Controller1.ButtonR1.pressing()){
    arm.spin(fwd, percent, pct);
  }
  else if (Controller1.ButtonR2.pressing()){
    arm.spin(reverse, percent, pct);
  }
  else {
    arm.stop(hold);
  }
}

void armMacro(int mid, int full){
  arm.setStopping(hold);
  if(Controller1.ButtonB.pressing()){
    arm.spinToPosition(0, degrees, false);
  }
  else if(Controller1.ButtonA.pressing()){
    arm.spinToPosition(mid, degrees, false);
  }
  else if(Controller1.ButtonX.pressing()){
    arm.spinToPosition(full, degrees, false);
  }
}

void tilterMacro(int up, int down){
  tilter.setStopping(hold);
  if(Controller1.ButtonDown.pressing()){
    tilter.spinToPosition(down, degrees, false);
  }
  else if(Controller1.ButtonUp.pressing()){
    tilter.spinToPosition(up, degrees, false);
  }
  else if(Controller1.ButtonRight.pressing()){
    tilter.spinToPosition(0, degrees, false);
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
    intake.spin(reverse, 100, pct);
  }
  else {
    intake.setStopping(coast);
  }
}


/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
/////=================================== Auton Control  ===================================/////

void setIntake(int input){
  intake.spin(fwd, input*scale, voltageUnits::mV);
}

void tank(double deg){
  tankDrive.spinToPosition(deg, degrees);
}
 
void setTank(double l, double r){
  leftDrive.spin(fwd, l, volt);
  rightDrive.spin(fwd, r, volt);
}

double gkP = 0.175;
double gError = 0;
 
void gturn(double angle){
  gError = angle - Inertial.orientation(yaw, degrees);
  while(!(gError < 1.0 && gError > -1.0)){
    gError = angle - Inertial.orientation(yaw, degrees);
    double speed = gError * gkP;
    setTank(speed, -speed);
    task::sleep(20);
  }
}

double avgPosition(){
  return (LF.position(degrees) + LB.position(degrees) + RF.position(degrees) + RB.position(degrees))/4;
}


/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
///// =================================== PID Control  =================================== /////


bool enablePID = false;
bool resetDrive = false;

double desVal;

double kP = 0.061;
double kI = 0.0;
double kD = 0.045;

double turnkP = 0.1;
double turnkI = 0.0;
double turnkD = 0.0;

double error, totalError = 0, prevError = 0, drv;

double turnError, turnTotalError = 0, turnPrevError = 0, turnDrv;


int PID(){
  while(enablePID){
    if (resetDrive) {
      resetDrive = false;
      LF.setPosition(0,degrees);
      LB.setPosition(0,degrees);
      RF.setPosition(0,degrees);
      RB.setPosition(0,degrees);
      Inertial.setRotation(0, degrees);
    }

  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Lateral    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////

  error = -1*desVal - avgPosition();
  totalError += error;
  drv = error - prevError;
  double lateralPower = (error * kP + totalError * kI + drv * kD);

  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Turning    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////

  turnError = Inertial.orientation(yaw, degrees) - 0.0;
  turnTotalError += turnError;
  turnDrv = turnError - turnPrevError;
   double turnPower = (turnError * turnkP + turnTotalError * turnkI + turnDrv * turnkD);

  ////================================================================////

  setTank(lateralPower - turnPower, lateralPower + turnPower);
  prevError = error;
  turnPrevError = turnError;

  vex::task::sleep(20);
  }
  return 1;
}

void move(double val){
  resetDrive = true;
  desVal = val;
}

/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////

void autonomous(void){
  tilter.setPosition(0, degrees);
  enablePID = true;
  vex::task drivePID(PID);

  move(500);
  wait(500, msec);
  setIntake(127);
  wait(800, msec);
  setIntake(0);

  move(300);

  vex::task::sleep(800);

  gturn(90.0);
  wait(100, msec);

  move(-600);
  tilter.spinFor(forward, 670, degrees, false);

  vex::task::sleep(1000);

  gturn(0.0);
  wait(100, msec);

  move(1800);

  vex::task::sleep(3000);

  move(100);
  vex::task::sleep(800);

  tilter.spinFor(reverse, 350, degrees, false);
  gturn(40.0);
  wait(400, msec);
  setIntake(127);
  wait(500, msec);
  setIntake(0); 
}

void usercontrol(void){
  while (1){
    userDrive();
    armControl(60);
    tilterMacro(600, 250);
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
    Brain.Screen.printAt( 10, 50, "Angle %6.1f", Inertial.orientation(yaw, degrees));
    Brain.Screen.printAt( 10, 200, "AVG %6.1f", avgPosition());

    //Brain.Screen.printAt( 10, 50, "Arm %6.1f", arm.position(degrees));

    //Brain.Screen.printAt( 10, 50, "LF %6.1f", LF.position(degrees));
    //Brain.Screen.printAt( 10, 200, "LB %6.1f", LB.position(degrees));
    //Brain.Screen.printAt( 300, 50, "RF %6.1f", RF.position(degrees));
    //Brain.Screen.printAt( 300, 200, "RB %6.1f", RB.position(degrees));
    vex::this_thread::sleep_for(50);
  }
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true){
    wait(100, msec);
  }
}
