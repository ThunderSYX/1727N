#include "vex.h"
using namespace vex;
competition Competition;
motor_group intake(intake1, intake2);


void pre_auton(void){
  vexcodeInit();
}

/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////
/////=================================== Auton Control  ===================================/////

void setIntake(int input){
  intake.spin(fwd, input, pct);
}

void setTank(double l, double r){
  LF.spin(fwd, l, volt);
  LB.spin(fwd, l, volt);
  RF.spin(fwd, r, volt);
  RB.spin(fwd, r, volt);
}

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
bool resetDrive = false;

double desVal, desTurn;

double kP = 0.049;
double kI = 0.0;
double kD = 0.0;

double turnkP = 0.1;
double turnkI = 0.0;
double turnkD = 0.0;

const int maxVel = 8;

double lError, lTotalError = 0, lPrevError = 0, lDrv, rError, rTotalError = 0, rPrevError = 0, rDrv;
double lP, rP, lI, rI, lD, rD;

double turnError, turnTotalError = 0, turnPrevError = 0, turnDrv;

int PID(){
  while(true){
    if (resetDrive){
      LF.setPosition(0, degrees);
      RF.setPosition(0, degrees);
      LB.setPosition(0, degrees);
      RB.setPosition(0, degrees);
    }

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

  double turnDiff = leftPosition() - rightPosition();
  turnError = turnDiff - desTurn;
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

  if(((lError+rError)/2) < 5 && ((lError+rError)/2) > -5){
    enablePID = false;
  }

  setTank(leftPower + turnPower, rightPower - turnPower);
  lPrevError = lError;
  rPrevError = rError;
  turnPrevError = turnError;

  vex::task::sleep(20);
  }
  return 1;
}

void move(double val, double turns){
  resetDrive = true;
  desVal = val;
  desTurn = turns;
}

/////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>/////

void autonomous(void){
  arm.setStopping(hold);
  tilter.setStopping(hold);
  tilter.setPosition(0, degrees);
  arm.setPosition(0, degrees);

  tilter.spinFor(reverse, 720, degrees);
  
  vex::task drivePID(PID);

  move(500, 0.0);

  tilter.spinFor(fwd, 200, degrees);

  setIntake(55);
  wait(800, msec);
  setIntake(0);

  move(230, 0.0);

  arm.spinToPosition(-700, degrees);
  tilter.spinFor(reverse, 100, degrees);

  vex::task::sleep(4000);

  tilter.spinFor(fwd, 350, degrees, false);
  wait(400, msec);
  
  move(-400, 0.0);
  vex::task::sleep(800);
}

void resetEncoders(){
  arm.setPosition(0, degrees);
  tilter.setPosition(0, degrees);
}

void usercontrol(void){
  while (1){
    enablePID = false;
    wait(20, msec);
  }
}

int main(){
  Brain.Screen.setFont(mono40);
  resetEncoders(); 
  Inertial.startCalibration();
  vex::this_thread::sleep_for(2000);
  
  while(1){
    Brain.Screen.printAt( 10, 50, "Angle %6.1f", Inertial.orientation(yaw, degrees));
    Brain.Screen.printAt( 10, 125, "Left %6.1f", leftPosition());
    Brain.Screen.printAt( 10, 200, "Right %6.1f", rightPosition());
    //Brain.Screen.printAt( 220, 125, "lP %6.1f", lP);
    //Brain.Screen.printAt( 200, 200, "rP %6.1f", rP);
    vex::this_thread::sleep_for(50);
  }
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();

  while (true){
    wait(100, msec);
  }
}