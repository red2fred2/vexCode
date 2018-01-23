#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

/****************************************************************
////////////////////////// Constants ////////////////////////////
****************************************************************/

//drivetrain
const int leftDriveDeadzone               = 10;
const double leftDriveMult                = 1.0;
const int rightDriveDeadzone              = 10;
const double rightDriveMult               = 1.0;

const double leftPMult                    = 1.0;
const double leftIMult                    = 1.0;
const double leftDMult                    = 1.0;

const double rightPMult                   = 1.0;
const double rightIMult                   = 1.0;
const double rightDMult                   = 1.0;

const double inchesPerWheelRotation       = 1.0;
const double inchesPerSwingTurnDegree     = 1.0;
const double inchesPerPivotDegree         = 1.0;

const double leftEncoderTicksPerRotation  = 100.0;
const double rightEncoderTicksPerRotation = 100.0;
const int leftTicksPerInch = (int)(leftEncoderTicksPerRotation / inchesPerWheelRotation);
const int rightTicksPerInch = (int)(rightEncoderTicksPerRotation / inchesPerWheelRotation);


////////////////////////////////////////////////////////////////
//lift
const int leftLiftDeadzone  = 10;
const double leftLiftMult   = 1.0;
const int rightLiftDeadzone = 10;
const double rightLiftMult  = 1.0;

/////////////////////////////////////////////////////////////////

//arm
const int armDeadzone = 10;
const double armMult  = 1.0;

/////////////////////////////////////////////////////////////////

//baseLift
const int leftBaseLiftDeadzone  = 10;
const double leftBaseLiftMult   = 1.0;
const int rightBaseLiftDeadzone = 10;
const double rightBaseLiftMult  = 1.0;


/****************************************************************
////////////////////////// Variables ////////////////////////////
****************************************************************/

double leftIntegral = 0;
double leftError = 0;
double rightIntegral = 0;
double rightError = 0;

bool tankMode = false;
bool buttonLock = false;


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

double P(double target, double actual, double Pmult) {
  return Pmult * (target - actual);
}

/////////////////////////////////////////////////////////////////

double leftI(double target, double actual, double Pmult, double Imult) {
  leftIntegral += P(target, actual, Pmult);
  return Imult * leftIntegral;
}
double rightI(double target, double actual, double Pmult, double Imult) {
  rightIntegral += P(target, actual, Pmult);
  return Imult * rightIntegral;
}

/////////////////////////////////////////////////////////////////

double leftD(double target, double actual, double Pmult, double Dmult) {
  double newError = P(target, actual, Pmult);
  double diff = newError - leftError;
  leftError = newError;
  return Dmult * newError;
}
double rightD(double target, double actual, double Pmult, double Dmult) {
  double newError = P(target, actual, Pmult);
  double diff = newError - rightError;
  rightError = newError;
  return Dmult * newError;
}

/////////////////////////////////////////////////////////////////

int leftPI(double target, double actual, double Pmult, double Imult) {
  return (int)(P(target, actual, Pmult) + leftI(target, actual, Imult));
}
int rightPI(double target, double actual, double Pmult, double Imult) {
  return (int)(P(target, actual, Pmult) + rightI(target, actual, Imult));
}

/////////////////////////////////////////////////////////////////

int leftPD(double target, double actual, double Pmult, double Dmult) {
  return (int)(P(target, actual, Pmult) + leftD(target, actual, Dmult));
}
int rightPD(double target, double actual, double Pmult, double Dmult) {
  return (int)(P(target, actual, Pmult) + leftD(target, actual, Dmult));
}

/////////////////////////////////////////////////////////////////

int leftID(double target, double actual, double Imult, double Dmult) {
  return (int)(leftI(target, actual, Imult) + leftD(target, actual, Dmult));
}
int rightID(double target, double actual, double Imult, double Dmult) {
  return (int)(rightI(target, actual, Imult) + rightD(target, actual, Dmult));
}

/////////////////////////////////////////////////////////////////

int leftPID(double target, double actual, double Pmult, double Imult, double Dmult) {
  return (int)(P(target, actual, Pmult) + leftI(target, actual, Imult) + leftD(target, actual, Dmult));
}
int rightPID(double target, double actual, double Pmult, double Imult, double Dmult) {
  return (int)(P(target, actual, Pmult) + rightI(target, actual, Imult) + rightD(target, actual, Dmult));
}

/////////////////////////////////////////////////////////////////

void resetLeftPID() {
  leftIntegral = 0;
  leftError = 0;
}
void resetRightPID() {
  rightIntegral = 0;
  rightError = 0;
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

void driveStraight(double inches) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = leftTicksPerInch * inches + getLeftDriveEncoder;
  int rightTicks = rightTicksPerInch * inches + getRightDriveEncoder;

  drive(
    leftPID(leftTicks, getLeftDriveEncoder, leftPMult, leftIMult, leftDMult),
    rightPID(rightTicks, getRightDriveEncoder, rightPMult, rightIMult, rightDMult)
  );
}

/////////////////////////////////////////////////////////////////

void leftSwingTurn(double degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = inchesPerSwingTurnDegree * leftTicksPerInch * degrees + getLeftDriveEncoder;
  int rightTicks = 0;

  drive(
    leftPID(leftTicks, getLeftDriveEncoder, leftPMult, leftIMult, leftDMult),
    rightPID(rightTicks, getRightDriveEncoder, rightPMult, rightIMult, rightDMult)
  );
}

void rightSwingTurn(double degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = 0;
  int rightTicks = inchesPerSwingTurnDegree * leftTicksPerInch * degrees + getRightDriveEncoder;

  drive(
    leftPID(leftTicks, getLeftDriveEncoder, leftPMult, leftIMult, leftDMult),
    rightPID(rightTicks, getRightDriveEncoder, rightPMult, rightIMult, rightDMult)
  );
}

/////////////////////////////////////////////////////////////////

void leftPivot(double degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = inchesPerPivotDegree * leftTicksPerInch * degrees + getLeftDriveEncoder;
  int rightTicks = -(inchesPerPivotDegree * rightTicksPerInch * degrees + getRightDriveEncoder);

  drive(
    leftPID(leftTicks, getLeftDriveEncoder, leftPMult, leftIMult, leftDMult),
    rightPID(rightTicks, getRightDriveEncoder, rightPMult, rightIMult, rightDMult)
  );
}

void rightPivot(double degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = -(inchesPerPivotDegree * leftTicksPerInch * degrees + getLeftDriveEncoder);
  int rightTicks = inchesPerPivotDegree * rightTicksPerInch * degrees + getRightDriveEncoder;

  drive(
    leftPID(leftTicks, getLeftDriveEncoder, leftPMult, leftIMult, leftDMult),
    rightPID(rightTicks, getRightDriveEncoder, rightPMult, rightIMult, rightDMult)
  );
}

/////////////////////////////////////////////////////////////////

void autoLift(int left, int right, double seconds) {
  lift(left, right);
  wait1Msec((int)(1000 * seconds));
}

/////////////////////////////////////////////////////////////////

void autoArm(int power, double seconds) {
  arm(power);
  wait1Msec((int)(1000 * seconds));
}

/////////////////////////////////////////////////////////////////

void autoBaseLift(int left, int right, double seconds) {
  baseLift(left, right);
  wait1Msec((int)(1000 * seconds));
}


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
  driveStraight(12);

  //cleanup
  nullifyLeftDriveEncoder();
  nullifyRightDriveEncoder();
}

/////////////////////////////////////////////////////////////////

task usercontrol() {
  while(true) {
    //automatic

    //driver
    if(VexRT(Btn6U)) {
      if(!buttonLock) {
        tankMode = !tankMode;
      }
      buttonLock = true;
    } else {
      buttonLock = false;
    }

    if(tankMode) {
      tankDrive();
    } else {
      joystickDrive();
    }

    //copilot
    liftControl();
    armControl();
    baseLiftControl();
  }
}
