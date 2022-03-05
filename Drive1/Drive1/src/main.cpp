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

int maxVel = 10;

const int armPct = 90;
const int intakePct = 100;
const int tilterPct = 45;

double drivePct = 1;

void pre_auton(void){
  vexcodeInit();
}

/////=================================== Driver Control  ===================================/////

bool slow = false;
bool latch = false;

void userDrive(){
  float maxSpeed = 100;
  float leftPct = (Controller1.Axis3.position()*drivePct)/maxSpeed;
  float rightPct = (Controller1.Axis2.position()*drivePct)/maxSpeed;
  
  if (Controller1.ButtonA.pressing() && !slow){
   slow = true;
  }
  if (Controller1.ButtonRight.pressing() && slow){
   slow = false;
  }
  
  if(slow){
    drivePct = 0.45;
  }
  else{
    drivePct = 1;
  }
    float leftNewPct = leftPct * leftPct *leftPct*100*drivePct;
    float rightNewPct = rightPct *rightPct *rightPct*100*drivePct;
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
  if (Controller1.ButtonX.pressing()){
   intakeTrue = true;
   this_thread::sleep_for(20);
  }
  else if (Controller1.ButtonB.pressing()){
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
    intake.spin(fwd, 100, pct);
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

double leftPosition(){
  return LShaft.position(degrees);
}

double rightPosition(){
  return -RShaft.position(degrees);
}


/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
///// =================================== PID Control  =================================== /////


bool enablePID = false;

double kP = 0.7;
double kI = 0.0;
double kD = 2;

double turnkP = 0.2;
double turnkI = 0.0;
double turnkD = 0.0;

double lError, lTotalError = 0, lPrevError = 0, lDrv;
double rError, rTotalError = 0, rPrevError = 0, rDrv;
double lP, rP, lI, rI, lD, rD;

double turnError, turnTotalError = 0, turnPrevError = 0, turnDrv;

void resetDrive(){
  LShaft.setPosition(0, degrees);
  RShaft.setPosition(0, degrees);
}

double time1 = 0;
double lFinalPower;
double rFinalPower;
double maxTime = 1;

bool isHeading = false;

void PID(int desVal, double desTurn){
  resetDrive();
  int restTime = 0;
  int totalTime = 0;
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
  if(!isHeading){
     turnError = Inertial.orientation(yaw, degrees) - desTurn;
  }
  else {
     turnError = Inertial.heading() - desTurn;
  }
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

  if((lError <= 2 && lError >= -2) && (rError <= 2 && rError >= -2)){
    if (restTime/100 >= 1.5){
      //LF.stop(hold);
      //LM.stop(hold);
      //LB.stop(hold);
      //RF.stop(hold);
      //RM.stop(hold);
      //RB.stop(hold);
        break;
      }
      restTime += 20;
    }
    else {
      restTime = 0;
    }
  
    if (totalTime/1000 > maxTime){
      break;
    }

  totalTime+=20;
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

double gkP = 1.2;

void gturn(double angle){     
  double gError;
  double restTime = 0;
  double totalTime = 0;
  bool right;
  if (angle > 0){
    right = true;
  }
  else if (angle < 0){
    right = false;
  }
  while(true){
    if (right){
      gError = angle - Inertial.heading();
    }
    else {
      gError = Inertial.yaw() - angle;
    }
    double speed = gError * gkP;
    if((gError < 1 && gError > -1)){
      if (restTime/100 >= 4){
        break;
      }
      restTime += 20;
    }
    else {
      restTime = 0;
    }
    if (totalTime / 1000 > 1){
      break;
    }
    totalTime+=20;
    if (right){
      setTank(speed, -speed);
    }
    else {
      setTank(-speed, speed);
    }
      task::sleep(20);
  }
}

void leftturn(double angle){
  int restTime = 0;
  int totalTime = 0;
  while(true){
    double gError = angle - Inertial.orientation(yaw, degrees);
    double speed = gError * gkP;
    if((gError < 1 && gError > -1)){
      if (restTime/100 >= 4){
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

void awpPoint(){
  maxVel = 6;
  Arm.spinFor(fwd, 200, degrees);
  
  intake.spin(reverse);
  wait(800, msec);
  intake.stop();

  PID(200, 0);
  wait(300, msec);
  leftturn(90);
  wait(300, msec);
  PID(450, 90);
  wait(300, msec);
  gturn(0);
  maxVel = 4;
  PID(-2100, 0);
  wait(500, msec);
  move2(-100, 45);
  wait(500, msec);
  PistonBack.set(true);
  wait(500, msec);
  intake.spin(reverse);
  PID(400, 0);
  PistonBack.set(false);
  leftturn(45);
}

void leftSide(){
  resetDrive();
  Piston.set(false);
  PistonBack.set(false);

  //PID(900, 0.0);
  move2(880, 90);
  wait(300, msec);
  move2(80, 35);
  Piston.set(true);

  PID(-900, 0.0);
  gturn(-90);

  //PID(-80, -90.0);
  move2(-80, 45);
  Arm.spinFor(fwd, 200, degrees);
  intake.spin(reverse);
  wait(2, sec);
  move2(400, 45);
  gturn(0);
}

void rightSide(){
  resetDrive();
  Piston.set(false);
  PistonBack.set(false);

  //PID(850, 0.0);
  move2(830, 100);
  move2(100, 45);
  Piston.set(true);
  
  wait(300, msec);
  PID(-550, 0.0);
  leftturn(-90);
  move2(-300, 30);
  //PID(-210, -90.0);
  PistonBack.set(true);
  wait(750, msec);
  PID(400, -90.0);
  //Arm.spinFor(fwd, 400, degrees);
  intake.setVelocity(100, pct);
  intake.spin(reverse);
  wait(700, msec);
  
  //Arm.spinFor(reverse, 380, degrees);
  PistonBack.set(false);
  intake.stop();
  PID(400, -90.0);
  Piston.set(false);
  
  
  PID(-300, -90.0);
  gturn(-45);
  PID(850, -45.0);
}

void skills(){
  resetDrive();
  Piston.set(false);
  maxVel = 6;
  move2(-60, 30);
  PistonBack.set(true);
  wait(400, msec);
  PID(100, 0.0);
  gturn(93);
  maxTime = 10;
  PID(1900, 93.0);
  Piston.set(true);
  
  //PID(1050, 90.0);
  gturn(270);
  move2(-150, 45);
  PistonBack.set(false);
  move2(120, 45);
  wait(500, msec);
  gturn(180);
  maxTime = 1;
  //isHeading = true;
  //PID(-500, 180);
  move2(-450, 60);
  PistonBack.set(true);
  gturn(180);
  //PID(900, 180);
  move2(1200, 50);

  //isHeading = false;
  wait(400, msec);
  Arm.setVelocity(80, pct);
  Arm.spinFor(fwd, 1400, degrees);

  gturn(90);
  move2(300, 45);
  Arm.spinFor(reverse, 400, degrees);
  Piston.set(false);
  Arm.spinFor(fwd, 400, degrees);
  wait(400, msec);
  move2(-300, 45);
  leftturn(-90);
  wait(500, msec);
  Arm.spinToPosition(0, degrees);
  move2(2000, 60);

  Piston.set(true);
  gturn(0);
  Piston.set(false);
  gturn(180);
  PistonBack.set(false);

  move2(800, 45);
  Piston.set(true);
  gturn(90);
  PID(1200, 90);

  Piston.set(false);
  PistonBack.set(false);
  /*
  isHeading = true;
  PID(600, 180);
  Piston.set(true);
  PID(-600, 180);
  isHeading = false;
  leftturn(-90);
  Piston.set(true);
  */
/*
  wait(300, msec);
  PistonBack.set(false);
  gturn(45);//penis envy
  //isHeading = true;
  
  maxTime = 10;
  maxVel = 5;
  intake.spin(reverse);
  PID(-2300, 45);
  PID(300, 45);
  gturn(0);
  isHeading = false;
  PID(-380, 0.0);
  wait(400, msec);
  PistonBack.set(true);
  wait(400, msec);
  PID(1000, 0.0);
  gturn(-90);
  isHeading = true;
  move2(300, 45);
  wait(400, msec);
  Piston.set(false);
  move2(-200, 45);
  */
}

void autonomous(void){
  intake.setVelocity(60, pct);
  isHeading = false;
  maxTime = 1;
  
  //leftSide();
  //rightSide();
  //awpPoint();
  skills();
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
  PistonBack.set(false);
  Arm.setPosition(0, degrees);
  Brain.Screen.setFont(mono40);
  Inertial.startCalibration();
  vex::this_thread::sleep_for(2000);
  //autonButton(280, 80, 75, 75, "Auton1");
  //autonButton(200, 80, 75, 75, "Auton2");
  //autonButton(360, 80, 75, 75, "Auton3");
  
  while(1){
    Brain.Screen.printAt( 10, 50, "Angle %6.1f", Inertial.heading());
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
    //Brain.Screen.printAt(140, 190, "Penis Envy");
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
