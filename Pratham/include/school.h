#pragma once
// It's called school because it holds a bunch of classes. Get it?
#include "calc\PID.h"
#include "calc\FPS.h"
#include "calc\MAPS.h"

class IMU
{
  private:
  public:
  IMU();
  static PID pid;
  static void To();
  static void For();
  void aTo();
  void aFor();
  void To(double target);
  void For(double target);
  void aTo(double target);
  void aFor(double target);
};

class BASE
{
  private:
  public:
  BASE();
  static PID pidTheta;
  static PID pid;
  static void To();
  static void For();
  void aTo();
  void aFor();
  void To(double target, double targetTheta);
  void For(double target, double targetTheta);
  void aTo(double target, double targetTheta);
  void aFor(double target, double targetTheta);
  static void polarFor();
  void driveFor(double revolutions, int speed);
  void driveTo(double revolutions, int speed);
};

class HDRIVE 
{
  private:
  public:
  HDRIVE();
  static PID pidX;
  static PID pidY;
  static void To();
  static void For();
  void aTo();
  void aFor();
  void To(double magnitude);
  void For(double magnitude);
  void aTo(double magnitude);
  void aFor(double magnitude);
  void To(double x, double y);
  void For(double x, double y);
  void aTo(double x, double y);
  void aFor(double x, double y);
  void To(double position[]);
  void For(double distance[]);
  void aTo(double position[]);
  void aFor(double distance[]);
};

class LIFTER
{
  private:
  public:
  LIFTER();
  static PID pid;
  static void To();
  static void For();
  void aTo();
  void aFor();
  void To(double target);
  void For(double target);
  void aTo(double target);
  void aFor(double target);
};

class ROBOT
{
  private:
  public:
  ROBOT();
  IMU imu = IMU();
  BASE base;
  MECH mech;
  HOLO holo;
  LIFTER lifter;
  FPS fps;
  MAPS map;
};