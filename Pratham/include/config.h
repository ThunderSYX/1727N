#pragma once

extern vex::brain Brain;
extern vex::motor Arm;
extern vex::motor Tilt;
extern vex::motor Intake;
extern vex::motor RF;
extern vex::motor RM;
extern vex::motor RB;
extern vex::motor LF;
extern vex::motor LM;
extern vex::motor LB;
// extern vex::motor_group intake;
extern vex::motor_group leftDrive;
extern vex::motor_group rightDrive;
namespace mg
{
  extern vex::motor_group x;
  extern vex::motor_group y;
}
extern vex::motor_group ld;
extern vex::motor_group rd;
extern vex::motor_group d;
extern vex::inertial Inertial;
extern vex::gyro Gyro;
extern vex::pot tilt;
extern vex::pot lift;
extern vex::encoder le;
extern vex::encoder re;
extern vex::encoder xe;
extern vex::encoder ye;
extern vex::rotation xr;
extern vex::rotation yr;
extern vex::limit Test;
extern vex::limit Debug;
extern vex::smartdrive sdt;
extern vex::drivetrain dt;
extern vex::controller Controller1;
extern vex::competition Competition;

extern int liftMax;
extern int liftTowerMid;
extern int liftTowerLow;
extern int liftTilter;
extern int liftMin;
extern int tiltMax;
extern int tiltStack;
extern int tiltMin;
extern bool isStacking;
extern double driveSpeedFactor;
extern double turnSpeedFactor;
extern double tileInch;
extern double trackWidth;
extern double wheelBase;
extern double baseDiagonal;