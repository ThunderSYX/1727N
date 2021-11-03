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
void pre_auton(void){
vexcodeInit();
}

typedef struct button  {
  int xpos;
  int ypos;
  int width;
  int height;
  bool state;
  vex::color color;
} button;

button buttons[] = {
  {30, 30, 20, 20, false, 0x808080},
  {30, 60, 20, 20, false, 0x808080},
  {60, 30, 20, 20, false, 0x808080},
  {60, 60, 20, 20, false, 0x808080},
  {90, 30, 20, 20, false, 0x808080}
};

void buttonPressed(int xpos, int ypos) {
  
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
  arm.setStopping(hold);
  if (Controller1.ButtonR1.pressing()){
    arm.spin(fwd, percent, pct);
  }
  else if (Controller1.ButtonR2.pressing()){
    arm.spin(reverse, percent, pct);
  }
  else if (!arm.isSpinning()){
    arm.stop();
  }
}

void tilterMacro(int up, int down){
  tilter.setStopping(hold);
  if(Controller1.ButtonDown.pressing()){
    tilter.spinToPosition(-down, degrees, false);
  }
  else if(Controller1.ButtonUp.pressing()){
    tilter.spinToPosition(-up, degrees, false);
  }
  else if(Controller1.ButtonRight.pressing()){
    tilter.spinToPosition(0, degrees, false);
  }
}

void tilterControl(int percent){
  tilter.setStopping(hold);
  if (Controller1.ButtonX.pressing()){
    tilter.spin(fwd, percent, pct);
  }
  else if (Controller1.ButtonB.pressing()){
    tilter.spin(reverse, percent, pct);
  }
  else if (!tilter.isSpinning()){
    tilter.stop();
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

double gkP = 0.3;
int maxVel = 10;

timer startTime;

void gturn(double angle){
  while(true){
    double gError = angle - Inertial.orientation(yaw, degrees);
    if (gError < 5.0 && gError > -5.0){
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

double lError, lTotalError = 0, lPrevError = 0, lDrv;
double rError, rTotalError = 0, rPrevError = 0, rDrv;

double turnError, turnTotalError = 0, turnPrevError = 0, turnDrv;


int PID(){
  while(enablePID){
    if (resetDrive) {
      leftDrive.setPosition(0, degrees);
      rightDrive.setPosition(0, degrees);
      resetDrive = false;
    }

  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Lateral    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////

  lError = -1*desVal - avgPosition();
  lTotalError += lError;
  lDrv = lError - lPrevError;
  double leftPower = (lError * kP + lTotalError * kI + lDrv * kD);

  rError = -1*desVal - avgPosition();
  rTotalError += rError;
  rDrv = rError - rPrevError; 
  double rightPower = (rError * kP + rTotalError * kI + rDrv * kD);

  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
  ///////////////////////////    Turning    ///////////////////
  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<////

  turnError = Inertial.orientation(yaw, degrees) - 0.0;
  turnTotalError += turnError;
  turnDrv = turnError - turnPrevError;
  double turnPower = (turnError * turnkP + turnTotalError * turnkI + turnDrv * turnkD);

  ////================================================================////

  setTank(leftPower - turnPower, rightPower + turnPower);
  lPrevError = lError;
  rPrevError = rError;
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

  move(-300);

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
    tilterMacro(450, 680);
    tilterControl(35);
    intakeControl(65);  
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
    Brain.Screen.setFont(monoS);
    Brain.Screen.printAt(140,70, "  __  _______  _____   _______  ____  _____  ");
    Brain.Screen.printAt(140,90, " /  ||  ___  |/ ___ `.|  ___  ||_   \\|_   _| ");
    Brain.Screen.printAt(140,110," `| ||_/  / /|_/___) ||_/  / /   |   \\ | |   ");
    Brain.Screen.printAt(140,130,"  | |    / /  .'____.'    / /    | |\\ \\| |   ");
    Brain.Screen.printAt(140,150," _| |_  / /  / /_____    / /    _| |_\\   |_  ");
    Brain.Screen.printAt(140,170,"|_____|/_/   |_______|  /_/    |_____|\\____| ");
    Brain.Screen.setFont(mono40);
    
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
