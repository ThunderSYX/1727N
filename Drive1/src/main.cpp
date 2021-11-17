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
// intake1              motor         7               
// arm                  motor         14              
// Inertial             inertial      12              
// tilter               motor         15              
// intake2              motor         11              
// bump                 limit         A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;
competition Competition;
 
motor_group leftDrive(LF, LB);
motor_group rightDrive(RF, RB);
motor_group intake(intake1, intake2);
motor_group tankDrive(LF, LB, RF, RB);

bool intakeTrue = false;
bool toggle = false;

const int scale = 120;
const int maxVel = 8;

const int armPct = 60;
const int intakePct = 60;
const int tilterPct = 45;
const int tilterPosMid = 300;
const int tilterPosLow = 680;
const int highArm = 1200;

double drivePct = 1;

void pre_auton(void){
  vexcodeInit();
}

int autonSelect;

void autonButton(int x, int y, int width, int height, std::string text){
  Brain.Screen.drawRectangle(x, y, width, height);
  Brain.Screen.printAt(x + width/2, y + height/2, text.c_str());
}

void checkAutonPress(int x, int y, int width, int height, int select) {
  if (Brain.Screen.pressing()) {
    if ((Brain.Screen.xPosition() >= x &&
      Brain.Screen.xPosition() <= x + width) &&
      (Brain.Screen.yPosition() >= y &&
      Brain.Screen.yPosition() <= y + height)) {
      autonSelect = select;
    }
  }
}

void resetEncoders(){
  tankDrive.setPosition(0, degrees);
  arm.setPosition(0, degrees);
  tilter.setPosition(0, degrees);
}

/////=================================== Driver Control  ===================================/////

bool slow = false;

float maxSpeed = 100;
float leftPct;
float rightPct;

float leftNewPct;
float rightNewPct;

void userDrive(){
  rightPct = (Controller1.Axis2.position()*drivePct)/maxSpeed;
  leftPct = (Controller1.Axis3.position()*drivePct)/maxSpeed;

  leftNewPct = leftPct * leftPct * leftPct * 100;
  rightNewPct = rightPct * rightPct * rightPct * 100;
  if(slow){
    drivePct = 0.45;
  }
  else{
    drivePct = 1;
  }
  if(toggle == false){
    rightDrive.spin(reverse, rightNewPct, pct);
    leftDrive.spin(reverse, leftNewPct, pct);
  }
  else if(toggle == true){
    rightDrive.spin(fwd, Controller1.Axis3.position()*drivePct, pct);
    leftDrive.spin(fwd, Controller1.Axis2.position()*drivePct, pct);
  }
  if(Controller1.ButtonLeft.pressing()){
    toggle = !toggle;
    this_thread::sleep_for(300);
  }
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
    tilter.spinToPosition(-tilterPosLow, degrees, false);
  }
  else if(Controller1.ButtonUp.pressing()){
    tilter.spinToPosition(-tilterPosMid, degrees, false);
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

void intakeControl(){
  if (Controller1.ButtonL1.pressing()){
   intakeTrue = true;
  }
  else if (Controller1.ButtonL2.pressing()){
   intakeTrue = false;
   intake.stop();
  }
  if (intakeTrue){
   intake1.spin(fwd, intakePct, pct);
   intake2.spin(fwd, intakePct*0.9, pct);
   drivePct = 0.6;
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

void tank(double deg){
  tankDrive.spinToPosition(deg, degrees);
}
 
void setTank(double l, double r){
  leftDrive.spin(fwd, l, volt);
  rightDrive.spin(fwd, r, volt);
}

//double gkP = 0.3;

timer startTime;

/*
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

/*
void gturn(double angle){
  while(true){
    double gError = angle - Inertial.orientation(yaw, degrees);
    if (gError < 3.0 && gError > -3.0){
      startTime.reset();
      if (startTime.time(msec) == 300 && gError < 3.0 && gError > -3.0){
        break;
      }
    }
    double speed = gError * gkP;
    if (speed > maxVel){
      speed = maxVel;
    }
    setTank(speed, -speed);
  }
}
*/

double gkP = 0.21;

void gturn(double angle){
  double gError = angle - Inertial.orientation(yaw, degrees);
  while(true){
    gError = angle - Inertial.orientation(yaw, degrees);
    double speed = gError * gkP;
    if((gError < 3.0 && gError > -3.0)){
      break;
    }
    setTank(-speed, speed);
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

//double desVal;
//double desTurn;

double kP = 0.071;
double kI = 0.0;
double kD = 0.15;

double turnkP = 1.5;
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

void PID(int desVal, double desTurn){
  while(true){

  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Lateral    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////
  
  lError = -desVal - leftPosition();
  lTotalError += lError;
  lDrv = lError - lPrevError;

  rError = -desVal - rightPosition();
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

  setTank(leftPower + turnPower, rightPower - turnPower);
  lPrevError = lError;
  rPrevError = rError;
  turnPrevError = turnError;


  if((lError < 7 && lError > -7) && (rError < 7 && rError > -7)){
    break;
  }
  else if (time1/1000 > 1.5){
    break;
  }

  time1+=20;
  vex::task::sleep(20);
  }
}

//void move(double val, double turns){
  //resetDrive = true;
  //desVal = val;
  //desTurn = turns;
//}

void stack(double fwd, double back, double ang){
  arm.spinFor(-highArm, degrees);
  resetDrive();
  PID(fwd, ang);
  tilter.spinFor(midTilt, degrees);
  resetDrive();
  PID(back, ang);
  arm.spinFor(highArm, degrees);
  tilter.spinToPosition(lowTilt, degrees);
}

/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////

void autonomous(void){
  arm.setStopping(hold);
  tilter.setStopping(hold);
  tilter.setPosition(0, degrees);
  arm.setPosition(0, degrees);
  //====================================LEFT====================================//
  tilter.setVelocity(70, pct);
  
  tilter.spinFor(reverse, 735, degrees, false);

  PID(500, 0.0);
 
  tilter.setVelocity(35, pct);
  tilter.spinFor(fwd, 300, degrees);
 
  setIntake(60);
  wait(1500, msec);
  setIntake(0);

  arm.setVelocity(70, pct);
  arm.spinFor(highArm, degrees);

  resetDrive();
  PID(300, 0.0);
  arm.spinFor(-250, degrees);
  wait(1000, msec);
  tilter.spinFor(reverse, 300, degrees);
  wait(500, msec);
  PID(-400, 0.0);

  arm.spinFor(-920, degrees);
  gturn(-70);
  
  //====================================----====================================//
  
  // Solo I think
  //PID(-320, 0.0);
  //vex::task::suspend(drivePID);

  //gturn(90.0);
  //vex::task::resume(drivePID);

  //PID(-670, 90.0);
  //vex::task::suspend(drivePID);
 
  //tilter.spinFor(reverse, 670, degrees, false);

  //gturn(0.0);
  //wait(100, msec);

  //move(2620, 0);


  //tilter.spinFor(fwd, 350, degrees, false);
  //wait(400, msec);
  
  //resetDrive();
  //PID(-400, 0.0);
  //move(300, 90.0);
  //vex::task::sleep(800);
  
  //setIntake(55);
  //wait(500, msec);
  //setIntake(0);

  


  //======================================= Right Side =======================================//  
  /*
  tilter.setVelocity(70, pct);
  tilter.spinFor(reverse, 720, degrees);
 
  resetDrive();
  PID(900, 0.0);
  vex::task::sleep(800);


  tilter.spinFor(fwd, 300, degrees);

  setIntake(55);
  wait(1000, msec);
  
  PID(-600, 0.0);
  */

  /*
  gturn(120);
  wait(300, msec);

  leftDrive.setVelocity(60, pct);
  rightDrive.setVelocity(60, pct);
  leftDrive.spinTo(-1800, degrees, false);
  rightDrive.spinTo(-1800, degrees, false);

  leftDrive.setVelocity(80, pct);
  rightDrive.setVelocity(80, pct);

  setIntake(0);

  leftDrive.spinTo(2000, degrees, false);
  rightDrive.spinTo(2000, degrees, false);

  tilter.stop(coast);
  */
 
  //=======================================SKILLS======================================//
  /* 
  tilter.spinFor(reverse, lowTilt, degrees);

  move(500, 0.0);

  tilter.spinFor(reverse, midTilt, degrees);

  setIntake(55);
  wait(800, msec);
  setIntake(0);
  
  stack(230, -200, 0.0);
 
  gturn(90.0);

  move(-2600, 90);

  move(300, 90);

  gturn(0);
  
  move(500, 0.0);

  gturn(-90);

  stack(200, -300, -90);
  
  gturn(90);

  move(800, -90);
  tilter.spinFor(midTilt, degrees);

  move(800, -90);

  stack(200, -300, -90);
  */
}

void usercontrol(void){
  Controller1.ButtonA.pressed(toggleSlow);
  while (1){
    enablePID = false;
    userDrive();
    armControl();
    tilterMacro();
    tilterControl();
    intakeControl();  
    wait(20, msec);
  }
}

int main(){
 
  Brain.Screen.setFont(mono40);
  resetEncoders();
  Inertial.startCalibration();
  vex::this_thread::sleep_for(2000);
  //autonButton(280, 80, 75, 75, "Auton1");
  //autonButton(200, 80, 75, 75, "Auton2");
  //autonButton(360, 80, 75, 75, "Auton3");
  
  while(1){
    Brain.Screen.printAt( 10, 50, "Angle %6.1f", Inertial.orientation(yaw, degrees));
    Brain.Screen.printAt( 10, 125, "Left %6.1f", leftNewPct);
    Brain.Screen.printAt( 10, 200, "Right %6.1f", rightNewPct);
    Brain.Screen.printAt( 250, 125, "left %6.1f", leftPct);
    Brain.Screen.printAt( 250, 200, "right %6.1f", rightPct);
    //Brain.Screen.setFont(monoS);
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
