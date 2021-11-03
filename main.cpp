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

void autonomous(void){

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
