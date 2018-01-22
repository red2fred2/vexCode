#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

/****************************************************************
////////////////////////// Constants ////////////////////////////
****************************************************************/

//drivetrain
const int leftDriveDeadzone = 10;
const double leftDriveMult = 1.0;
const int rightDriveDeadzone = 10;
const double rightDriveMult = 1.0;

const double drivePMult = 1.0;
const double driveIMult = 1.0;
const double driveDMult = 1.0;

////////////////////////////////////////////////////////////////
//lift
const int leftLiftDeadzone = 10;
const double leftLiftMult = 1.0;
const int rightLiftDeadzone = 10;
const double rightLiftMult = 1.0;

/////////////////////////////////////////////////////////////////

//arm
const int armDeadzone = 10;
const double armMult = 1.0;

/////////////////////////////////////////////////////////////////

//baseLift
const int leftBaseLiftDeadzone = 10;
const double leftBaseLiftMult = 1.0;
const int rightBaseLiftDeadzone = 10;
const double rightBaseLiftMult = 1.0;


/****************************************************************
////////////////////////// Variables ////////////////////////////
****************************************************************/

double driveIntegral = 0;
double driveError = 0;


/****************************************************************
/////////////////////// Utility Functions ///////////////////////
****************************************************************/

int deadzone(int value, int deadzone) {
    if(value > deadzone) return value;
    else return 0;
}

/////////////////////////////////////////////////////////////////

int applyMult(int value, double multiplier) {
  int val = (double) value;
  return (int)(val * multiplier);
}

/////////////////////////////////////////////////////////////////

void drive(int left, int right) {
  int deadLeft = deadzone(left, leftDriveDeadzone);
  int multLeft = applyMult(deadLeft, leftDriveMult);
  motor[leftDrive] = multLeft;

  int deadRight = deadzone(right);
  int multRight = applyMult(deadRight, rightDriveMult);
  motor[rightDrive] = multRight;
}

/////////////////////////////////////////////////////////////////

void lift(int left, int right) {
  int deadleft = deadzone(left, leftLiftDeadzone);
  int multLeft = applyMult(deadLeft, leftLiftMult);
  motor[leftLift] = leftMult;

  int deadRight = deadzone(right, rightLiftDeadzone);
  int multRight = applyMult(deadRight, rightLiftMult);
  motor[rightLift] = multRight;
}

/////////////////////////////////////////////////////////////////

void arm(int power) {
  int deadPower = deadzone(power, armDeadzone);
  int multPower = applyMult(deadPower, armMult);
  motor[armSlide] = multPower;
}

/////////////////////////////////////////////////////////////////

void baseLift(int left, int right) {
  int deadleft = deadzone(left, leftBaseLiftDeadzone);
  int multLeft = applyMult(deadleft, leftBaseLiftMult);
  motor[baseLiftLeft] = multLeft;
  
  int deadRight = deadzone(right, rightBaseLiftDeadzone);
  int multRight = applyMult(deadRight, rightBaseLiftMult);
  motor[baseLiftRight] = multRight;
}

/////////////////////////////////////////////////////////////////

int leftDriveEncoderNullifier = 0;
int getLeftDriveEncoder() {
  return encoderValue - leftDriveEncoderNullifier;
}
void nullifyLeftDriveEncoder() {
  leftDriveEncoderNullifier = getLeftDriveEncoder();
}

/////////////////////////////////////////////////////////////////

int rightDriveEncoderNullifier = 0;
int getRightDriveEncoder() {
  return encoderValue - rightDriveEncoderNullifier;
}
void nullifyRightDriveEncoder() {
  rightDriveEncoderNullifier = getRightDriveEncoder();
}


/****************************************************************
///////////////////////// PID Functions /////////////////////////
****************************************************************/

double p(double target, double actual, double mult) {
  return mult * (target - actual);
}

/////////////////////////////////////////////////////////////////

double i(double target, double actual, double mult) {
  driveI += p(target, actual, mult);
  return driveIntegral;
}

/////////////////////////////////////////////////////////////////

double d(double target, double actual, double mult) {
  double newError = p(target, actual, mult);
  double diff = newError - driveError;
  driveError = newError;
  return mult * (newError);
}

/////////////////////////////////////////////////////////////////

double pi(double target, double actual, double Pmult, double Imult) {
  return p(target, actual, Pmult) + i(target, actual, Imult);
}

/////////////////////////////////////////////////////////////////

double pd(double target, double actual, double Pmult, double Dmult) {
  return p(target, actual, Pmult) + d(target, actual, Dmult);
}

/////////////////////////////////////////////////////////////////

double id(double target, double actual, double Imult, double Dmult) {
  return i(target, actual, Imult) + d(target, actual, Dmult);
}

/////////////////////////////////////////////////////////////////

double pid(double target, double actual, double Pmult, double Imult, double Dmult) {
  return p(target, actual, Pmult) + i(target, actual, Imult) + d(target, actual, Dmult);
}


/****************************************************************
/////////////////////// Teleop Functions ////////////////////////
****************************************************************/

void tankDrive() {
  int left = VexRT(Ch3);
  int right = VexRT(Ch2);
  drive(left, right);
}

/////////////////////////////////////////////////////////////////

void joystickDrive() {
  int up = vexRT(Ch3);
  int right = vexRT(Ch4);
  drive(up + right, up - right);
}

/////////////////////////////////////////////////////////////////

void liftControl() {
  int left = vexRT(Ch3Xmtr2);
  int right = vexRT(Ch2Xmtr2);
  lift(left, right);
}

/////////////////////////////////////////////////////////////////

void armControl() {
  int forward = vexRT(Ch4Xmtr2);
}

/////////////////////////////////////////////////////////////////

void baseLiftControl() {
  int power = VexRT(Ch1Xmtr2);
  baseLift(power);
}


/****************************************************************
//////////////////////// Auto Functions /////////////////////////
****************************************************************/


/****************************************************************
///////////////////////////// Main //////////////////////////////
****************************************************************/
void pre_auton() {
  //setup
  nullifyLeftDriveEncoder();
  nullifyRightDriveEncoder();
}

/////////////////////////////////////////////////////////////////

task autonomous() {
  //routine


  //cleanup
  nullifyLeftDriveEncoder();
  nullifyRightDriveEncoder();
}

/////////////////////////////////////////////////////////////////

task usercontrol() {
  while(true) {
    //automatic

    //driver
    //tankDrive();
    joystickDrive();

    //copilot
    liftControl();
    armControl();
    baseLiftControl();
  }
}
