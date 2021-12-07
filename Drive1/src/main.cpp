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
// LF                   motor         2               
// LB                   motor         18              
// RF                   motor         19              
// RB                   motor         9               
// intake               motor         20              
// arm                  motor         14              
// Inertial             inertial      13              
// tilter               motor         15              
// angler               motor         7               
// Piston               digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;
competition Competition;
 
motor_group leftDrive(LF, LB);
motor_group rightDrive(RF, RB);
motor_group tankDrive(LF, LB, RF, RB);

bool intakeTrue = false;

const int maxVel = 10;

const int armPct = 60;
const int intakePct = 100;
const int tilterPct = 45;
const int midTilt = 200;
const int lowTilt = 600;
const int highArm = 1200;

double drivePct = 1;

void pre_auton(void){
  vexcodeInit();
}

/////=================================== Driver Control  ===================================/////

bool slow = false;

void userDrive(){
  float maxSpeed = 100;
  float leftPct = (Controller1.Axis3.position()*drivePct)/maxSpeed;
  float rightPct = (Controller1.Axis2.position()*drivePct)/maxSpeed;

  float leftNewPct = leftPct * leftPct *leftPct*100;
  float rightNewPct = rightPct *rightPct *rightPct*100;
  if(slow){
    drivePct = 0.45;
  }
  else{
    drivePct = 1;
  }
    rightDrive.spin(fwd, rightNewPct, pct);
    leftDrive.spin(fwd, leftNewPct, pct);
}

void pistonControl(){
  if(Controller1.ButtonUp.pressing()){
    Piston.set(true);
  }
  else {
    Piston.set(false);
  }
  wait(20,msec);
}

void toggleSlow(){
  if(slow){
    slow = false;
  }
  else{
    slow = true;
  }
}

void armControl(){
  arm.setStopping(hold);
  if (Controller1.ButtonR1.pressing()){
    arm.spin(fwd, armPct, pct);
  }
  else if (Controller1.ButtonR2.pressing()){
    arm.spin(reverse, armPct, pct);
  }
  else if (!arm.isSpinning()){
    arm.stop();
  }
}

void tilterMacro(){
  tilter.setStopping(hold);
  if(Controller1.ButtonDown.pressing()){
    tilter.spinToPosition(-lowTilt, degrees, false);
  }
  else if(Controller1.ButtonUp.pressing()){
    tilter.spinToPosition(-midTilt, degrees, false);
  }
  else if(Controller1.ButtonRight.pressing()){
    tilter.spinToPosition(0, degrees, false);
  }
}

void tilterControl(){
  tilter.setStopping(hold);
  if (Controller1.ButtonX.pressing()){
    tilter.spin(fwd, tilterPct, pct);
  }
  else if (Controller1.ButtonB.pressing()){
    tilter.spin(reverse, tilterPct, pct);
  }
  else if (!tilter.isSpinning()){
    tilter.stop();
  }
}

void anglerControl(){
  angler.setStopping(hold);
  if (Controller1.ButtonUp.pressing()){
    angler.spin(fwd, 80, pct);
  }
  else if (Controller1.ButtonDown.pressing()){
    angler.spin(reverse, 80, pct);
  }
  else if (!angler.isSpinning()){
    angler.stop();
  }
}

void intakeControl(){
  if (Controller1.ButtonL1.pressing()){
   intakeTrue = true;
  }
  else if (Controller1.ButtonL2.pressing()){
   intakeTrue = false;
   intake.stop();
  }
  if (intakeTrue){
   intake.spin(fwd, intakePct, pct);
   //drivePct = 0.6;
  }
  else if(Controller1.ButtonY.pressing()) {
    intake.spin(reverse, 100, pct);
  }
  else {
    intake.setStopping(coast);
    drivePct = 1;
  }
}

/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
/////=================================== Auton Control  ===================================/////

void setIntake(int input){
  intake.spin(fwd, input, pct);
}
 
void setTank(double l, double r){
  leftDrive.spin(fwd, l, volt);
  rightDrive.spin(fwd, r, volt);
}

/*
timer startTime;
double gkP = 0.3;

void gturn(double angle) {
  bool right = signbit(-angle);
  while (true) {
    double gError = angle - Inertial.orientation(yaw, degrees);
    if (gError < 3.0 && gError > -3.0) {
      startTime.reset();
      if (startTime.time(msec) == 300 && gError < 3.0 && gError > -3.0) {
        break;
      }
    }
    else {
      double speed = gError * gkP;
      double leftCorrection = 0.0;
      double rightCorrection = 0.0;
      double motorError = leftDrive.position(degrees) + rightDrive.position(degrees); 
      if ((right && signbit(-motorError)) || (!right && signbit(motorError))) {
        rightCorrection =  motorError * 0.5;
      } 
      else if ((right && signbit(motorError)) || (right && signbit(-motorError))) {
        leftCorrection = motorError * 0.5;
      }
      if (speed > maxVel) {
        speed = maxVel;
      }
      setTank(speed + leftCorrection, -speed + rightCorrection);
    }
  }
}
*/

double gkP = 0.4;
double gError;
int restTime;
int totalTime;

void gturn(double angle){
  while(true){
    restTime = 0;
    gError = angle - Inertial.orientation(yaw, degrees);
    double speed = gError * gkP;
    if((gError < 3.0 && gError > -3.0)){
      //restTime+=20;
      //if (restTime >= 300){
        break;
      //}
    }
    if(totalTime /1000 > 0.6){
      break;
    }
    totalTime+=20;
    setTank(speed, -speed);
    task::sleep(20);
  }
}


double leftPosition(){
  return (LF.position(degrees) + LB.position(degrees))/2;
}

double rightPosition(){
  return (RF.position(degrees) + RB.position(degrees))/2;
}


/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
///// =================================== PID Control  =================================== /////


bool enablePID = false;

double kP = 0.7;
double kI = 0.0;
double kD = 0.07;

double turnkP = 0.3;
double turnkI = 0.0;
double turnkD = 0.0;

double lError, lTotalError = 0, lPrevError = 0, lDrv;
double rError, rTotalError = 0, rPrevError = 0, rDrv;
double lP, rP, lI, rI, lD, rD;

double turnError, turnTotalError = 0, turnPrevError = 0, turnDrv;

void resetDrive(){
  LF.setPosition(0, degrees);
  RF.setPosition(0, degrees);
  LB.setPosition(0, degrees);
  RB.setPosition(0, degrees);
}

double time1 = 0;
double lFinalPower;
double rFinalPower;

void PID(int desVal, double desTurn){
  while(true){

  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Lateral    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////
  
  lError = desVal - leftPosition();
  lTotalError += lError;
  lDrv = lError - lPrevError;

  rError = desVal - rightPosition();
  rTotalError += rError;
  rDrv = rError - rPrevError; 

  lP = lError * kP;
  lI = lTotalError * kI;
  lD = lDrv * kD;

  rP = rError * kP;
  rI = rTotalError * kI;
  rD = rDrv * kD;

  

  double leftPower = (lP + lI + lD);
  
  double rightPower = (rP + rI + rD);

  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Turning    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////

  turnError = Inertial.orientation(yaw, degrees) - desTurn;
  turnTotalError += turnError;
  turnDrv = turnError - turnPrevError;
  double turnPower = (turnError * turnkP + turnTotalError * turnkI + turnDrv * turnkD);

  ////================================================================////

  if(leftPower > maxVel){
    leftPower = maxVel;
  }
  if(rightPower > maxVel){
    rightPower = maxVel;
  }

  lFinalPower = leftPower - turnPower;
  rFinalPower = rightPower + turnPower;

  setTank(lFinalPower, rFinalPower);
  lPrevError = lError;
  rPrevError = rError;
  turnPrevError = turnError;


  if((lError < 4 && lError > -4) && (rError < 4 && rError > -4)){
    LF.stop(hold);
    LB.stop(hold);
    RF.stop(hold);
    RB.stop(hold);
    break;
  }
  else if (time1/1000 > 3){
    break;
  }

  time1+=20;
  vex::task::sleep(20);
  }
}

void move2(double dist, int vel){
  LF.setVelocity(vel, pct);
  LB.setVelocity(vel, pct);
  RF.setVelocity(vel, pct);
  RB.setVelocity(vel, pct);
  LF.spinFor(fwd, dist, degrees, false);
  LB.spinFor(fwd, dist, degrees, false);
  RF.spinFor(fwd, dist, degrees, false);
  RB.spinFor(fwd, dist, degrees);
}

/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////

void autonomous(void){
 
}

void usercontrol(void){
  Controller1.ButtonA.pressed(toggleSlow);
  while (1){
    enablePID = false;
    userDrive();
    armControl();
    //tilterMacro();
    tilterControl();
    intakeControl();  
    anglerControl();
    wait(20, msec);
  }
}

int main(){
 
  Brain.Screen.setFont(mono40);
  Inertial.startCalibration();
  vex::this_thread::sleep_for(2000);
  //autonButton(280, 80, 75, 75, "Auton1");
  //autonButton(200, 80, 75, 75, "Auton2");
  //autonButton(360, 80, 75, 75, "Auton3");
  
  while(1){
    Brain.Screen.printAt( 10, 50, "Angle %6.1f", Inertial.orientation(yaw, degrees));
    Brain.Screen.printAt( 10, 125, "Left %6.1f", leftPosition());
    Brain.Screen.printAt( 10, 200, "Right %6.1f", rightPosition());
    //Brain.Screen.printAt( 250, 125, "lER %6.1f", lError);
    //Brain.Screen.printAt( 250, 200, "rER %6.1f", rError);
    Brain.Screen.setFont(monoS);
    //checkAutonPress(280, 80, 75, 75, 0);
    //checkAutonPress(200, 80, 75, 75, 1);
    //checkAutonPress(360, 80, 75, 75, 2);
    //Brain.Screen.printAt(140,70, "  __  _______  _____   _______  ____  _____  ");
    //Brain.Screen.printAt(140,90, " /  ||  ___  |/ ___ `.|  ___  ||_   \\|_   _| ");
    //Brain.Screen.printAt(140,110," `| ||_/  / /|_/___) ||_/  / /   |   \\ | |   ");
    //Brain.Screen.printAt(140,130,"  | |    / /  .'____.'    / /    | |\\ \\| |   ");
    //Brain.Screen.printAt(140,150," _| |_  / /  / /_____    / /    _| |_\\   |_  ");
    //Brain.Screen.printAt(140,170,"|_____|/_/   |_______|  /_/    |_____|\\____| ");
    Brain.Screen.setFont(mono40);

    vex::this_thread::sleep_for(50);
  }
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true){
    wait(100, msec);
  }
}
