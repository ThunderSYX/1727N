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
// Piston               digital_out   B               
// LF                   motor         17              
// LM                   motor         11              
// LB                   motor         20              
// RF                   motor         1               
// RM                   motor         6               
// RB                   motor         9               
// Arm                  motor         2               
// intake               motor         18              
// tilter               motor         12              
// angler               motor         3               
// PistonBack           digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;
competition Competition;
 
motor_group leftDrive(LF, LM, LB);
motor_group rightDrive(RF, RM, RB);
motor_group tankDrive(LF, LM, LB, RF, RM, RB);

bool intakeTrue = false;

const int maxVel = 10;

const int armPct = 90;
const int intakePct = 100;
const int tilterPct = 45;

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

  float leftNewPct = leftPct * leftPct *leftPct*100*drivePct;
  float rightNewPct = rightPct *rightPct *rightPct*100*drivePct;
  
  if (Controller1.ButtonA.pressing()){
   slow = true;
  }
  if (Controller1.ButtonA.pressing() && slow){
   slow = false;
  }
  
  if(slow){
    drivePct = 0.45;
  }
  else{
    drivePct = 1;
  }
    rightDrive.spin(fwd, rightNewPct, pct);
    leftDrive.spin(fwd, leftNewPct, pct);
}

bool pistonDown = false;
bool pistonBackDown = false;

void pistonControl(){
  if(Controller1.ButtonL2.pressing()){
    pistonDown = true;
  }
  else if (Controller1.ButtonL1.pressing()){
    pistonDown = false; 
  }
  if (pistonDown){
    Piston.set(true);
  }
  else {
    Piston.set(false);
  }

  if(Controller1.ButtonDown.pressing()){
    pistonBackDown = true;
  }
  else if (Controller1.ButtonUp.pressing()){
    pistonBackDown = false; 
  }
  if (pistonBackDown){
    PistonBack.set(true);
  }
  else {
    PistonBack.set(false);
  }

  wait(20,msec);
}

void armControl(){
  Arm.setStopping(hold);
  if (Controller1.ButtonR1.pressing()){
    Arm.spin(fwd, armPct, pct);
  }
  else if (Controller1.ButtonR2.pressing()){
    Arm.spin(reverse, armPct, pct);
  }
  else if (!Arm.isSpinning()){
    Arm.stop();
  }
}

void intakeControl(){
  if (Controller1.ButtonX.pressing() && !intakeTrue){
   intakeTrue = true;
   this_thread::sleep_for(20);
  }
  else if (Controller1.ButtonB.pressing() && intakeTrue){
   intakeTrue = false;
   this_thread::sleep_for(20);
  }
  if (intakeTrue){
   intake.spin(reverse, intakePct, pct);
   //drivePct = 0.6;
  }
  else {
    intake.stop();
  }
  if(Controller1.ButtonY.pressing()) {
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

double gkP = 0.5;
double gError;
int restTime;
int totalTime;

void gturn(double angle){
  while(true){
    gError = angle - Inertial.orientation(yaw, degrees);
    double speed = gError * gkP;
    if((gError < 2.0 && gError > -2.0)){
      if (restTime/100 >= 3){
        break;
      }
      restTime += 20;
    }
    else {
      restTime = 0;
    }
    totalTime+=20;
    setTank(speed, -speed);
    task::sleep(20);
  }
}

double leftPosition(){
  return (LF.position(degrees) + LM.position(degrees) + LB.position(degrees))/3;
}

double rightPosition(){
  return (RM.position(degrees) + RB.position(degrees))/2;
}


/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
///// =================================== PID Control  =================================== /////


bool enablePID = false;

double kP = 0.7;
double kI = 0.0;
double kD = 0.0;

double turnkP = 0.0;
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
  LM.setPosition(0, degrees);
  RM.setPosition(0, degrees);
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

  lFinalPower = leftPower;// - turnPower;
  rFinalPower = rightPower;// + turnPower;

  setTank(lFinalPower, rFinalPower);
  lPrevError = lError;
  rPrevError = rError;
  turnPrevError = turnError;


  if((lError < 4 && lError > -4) && (rError < 4 && rError > -4)){
    LF.stop(hold);
    LM.stop(hold);
    LB.stop(hold);
    RF.stop(hold);
    RM.stop(hold);
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
  LM.setVelocity(vel, pct);
  RF.setVelocity(vel, pct);
  RM.setVelocity(vel, pct);
  RB.setVelocity(vel, pct);
  LF.spinFor(fwd, dist, degrees, false);
  LM.spinFor(fwd, dist, degrees, false);
  LB.spinFor(fwd, dist, degrees, false);
  RF.spinFor(fwd, dist, degrees, false);
  RM.spinFor(fwd, dist, degrees, false);
  RB.spinFor(fwd, dist, degrees);
}

/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////

void autonomous(void){
  //PID(2000, 0.0);
  PistonBack.set(true);
  gturn(90);
  resetDrive();
  //move2(200, 20);
  PID(200, 90.0);

  /*PistonBack.set(false);
  wait(1500, msec);
  move2(-100, 50);
  PistonBack.set(true);
  wait(500, msec);
  move2(300, 60);
*/
  
  /*
  Piston.set(false);
  move2(1000, 80);
  move2(100, 20);
  wait(400, msec);
  Piston.set(true);
  wait(400, msec);
  PistonBack.set(false);
  move2(-1000, 40);
  */

  /*
  Piston.set(false);
  move2(1000, 80);
  move2(100, 20);
  wait(400, msec);
  Piston.set(true);
  wait(400, msec);
  move2(800, 60);
  wait(500, msec);
  Arm.spinToPosition(700, degrees);
  LF.spinFor(fwd, 800, degrees, false);
  LM.spinFor(fwd, 800, degrees, false);
  LB.spinFor(fwd, 800, degrees, false);
  RF.spinFor(fwd, 1100, degrees, false);
  RM.spinFor(fwd, 1100, degrees, false);
  RB.spinFor(fwd, 1100, degrees);
  wait(300, msec);
  Piston.set(false);
  wait(500, msec);
  move2(-400, 70);
  */
}

void usercontrol(void){
  while (1){
    enablePID = false;
    userDrive();
    armControl();
    intakeControl();  
    pistonControl();
    wait(20, msec);
  }
}

int main(){
  Piston.set(true);
  PistonBack.set(true);
  Arm.setPosition(0, degrees);
  Brain.Screen.setFont(mono40);
  Inertial.startCalibration();
  vex::this_thread::sleep_for(2000);
  //autonButton(280, 80, 75, 75, "Auton1");
  //autonButton(200, 80, 75, 75, "Auton2");
  //autonButton(360, 80, 75, 75, "Auton3");
  
  while(1){
    Brain.Screen.printAt( 10, 50, "Angle %6.1f", Inertial.orientation(yaw, degrees));
    Brain.Screen.printAt( 10, 125, "Left %6.1f", LShaft.position(degrees));
    Brain.Screen.printAt( 10, 200, "Right %6.1f", -RShaft.position(degrees));
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
