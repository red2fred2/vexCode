#pragma platform(VEX2)
#pragma competitionControl(Competition)
#pragma config(UART_Usage, UART2, uartVEXLCD, baudRate19200, IOPins, None, None)
#include "Vex_Competition_Includes.c"

/****************************************************************
////////////////////////// Constants ////////////////////////////
****************************************************************/

//general

const double expoMult                     = 0.039579;
const int expoPow                         = 3;

const double inchesPerWheelRotation       = 1.0;
const double inchesPerSwingTurnDegree     = 1.0;
const double inchesPerPivotDegree         = 1.0;

const double leftEncoderTicksPerRotation  = 100.0;
const double rightEncoderTicksPerRotation = 100.0;

const int leftTicksPerInch = (int)(leftEncoderTicksPerRotation / inchesPerWheelRotation);
const int rightTicksPerInch = (int)(rightEncoderTicksPerRotation / inchesPerWheelRotation);

////////////////////////////////////////////////////////////////

//lcd

const int lcdLeft   = 1;
const int lcdCenter = 2;
const int lcdRight  = 4;
const int lcdDelay  = 5;

const String lcdEnterString = "<         Enter        >";
const String lcdIsRunningString = "is running!";

////////////////////////////////////////////////////////////////

//drivetrain
const int leftDriveDeadzone  = 10;
const double leftDriveMult   = 1.0;
const int rightDriveDeadzone = 10;
const double rightDriveMult  = 1.0;

const double PMult           = 1.0;
const double leftIMult       = 1.0;
const double leftDMult       = 1.0;
const double rightIMult      = 1.0;
const double rightDMult      = 1.0;

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

/////////////////////////////////////////////////////////////////

//claw
const int clawDeadzone = 10;
const double clawMult  = 1.0;


/****************************************************************
////////////////////////// Variables ////////////////////////////
****************************************************************/

double leftIntegral = 0;
double leftError = 0;
double rightIntegral = 0;
double rightError = 0;

bool tankMode = false;

int lcdSelection = 0;


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

int expoCurve(int in) {
  double base = expoMult * in;
  return (int)(pow(base, expoPow));
}

/////////////////////////////////////////////////////////////////

void waitUntilTrue(bool in) {
  while(not in) {}
  wait1Msec(lcdDelay);
}

/////////////////////////////////////////////////////////////////

void waitUntilFalse(bool in) {
  while(in) {}
  wait1Msec(lcdDelay);
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

void expoDrive(int left, int right) {
  int deadLeft = deadzone(left, leftDriveDeadzone);
  int expoLeft = expoCurve(deadLeft);
  int multLeft = applyMult(expoLeft, leftDriveMult);
  motor[leftDrive] = multLeft;

  int deadRight = deadzone(right);
  int expoRight = expoCurve(deadRight);
  int multRight = applyMult(expoRight, rightDriveMult);
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

void claw(int power) {
  int deadPower = deadzone(power, clawDeadzone);
  int multPower = applyMult(deadPower, clawMult);
  motor[claw] = multPower;
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
///////////////////////// Auto Selector /////////////////////////
****************************************************************/

void lcdControl(int in) {
  waitUntilTrue(in);

  if(in == lcdleft) lcdSelection--;
  if(in == lcdRight) lcdSelection++;

  //only keep sane values
  if(lcdSelection < 0) lcdSelection = 3;
  if(lcdSelection > 3) lcdSelection = 0;

  waitUntilFalse(in);
}

/////////////////////////////////////////////////////////////////

void lcdDisplay(String top, String bottom) {
  clearLCDLine(0);
  clearLCDLine(1);
  displayLCDCenteredString(0, top);
  displayLCDCenteredString(1, bottom);
}

/////////////////////////////////////////////////////////////////

void lcdAutonomousSelector() {
  bool centerPressed = nLCDButtons == lcdCenter;
  bool autonomous = bIfiAutonomousMode;

  while(not(centerPressed or autonomous)) {
    switch(lcdSelection) {
      case 0: lcdDisplay("Left Auto", lcdEnterString); break;
      case 1: lcdDisplay("Right Auto", lcdEnterString); break;
      case 2: lcdDisplay("Nothing", lcdEnterString); break;
      case 3: lcdDisplay("Nothing", lcdEnterString); break;
    }
    lcdControl(nLCDButtons);
  }
}

/////////////////////////////////////////////////////////////////

void runAuto() {
  switch(lcdSelection) {
    case 0:
      lcdDisplay("Left Auto", lcdIsRunningString);
      leftAuto();
    break;
    case 1:
      lcdDisplay("Right Auto", lcdIsRunningString);
      rightAuto();
    break;
    case 2:
      lcdDisplay("Nothing", lcdIsRunningString);
    break;
    case 3:
      lcdDisplay("Nothing", lcdIsRunningString);
    break; 
    default: lcdDisplay("Invalid", "lcdSelection"); break;
  }
}


/****************************************************************
///////////////////////// PID Functions /////////////////////////
****************************************************************/

double P(double target, double actual) {
  return Pmult * (target - actual);
}

/////////////////////////////////////////////////////////////////

double leftI(double target, double actual) {
  leftIntegral += P(target, actual);
  return leftIMult * leftIntegral;
}
double rightI(double target, double actual) {
  rightIntegral += P(target, actual);
  return leftIMult * rightIntegral;
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
  expoDrive(left, right);
}

/////////////////////////////////////////////////////////////////

void joystickDrive() {
  int up = vexRT(Ch3);
  int right = vexRT(Ch4);
  expoDrive(up + right, up - right);
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

void autoDrive(double inches) {
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

void autoLeftSwingTurn(double degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = inchesPerSwingTurnDegree * leftTicksPerInch * degrees + getLeftDriveEncoder;
  int rightTicks = 0;

  drive(
    leftPID(leftTicks, getLeftDriveEncoder, leftPMult, leftIMult, leftDMult),
    rightPID(rightTicks, getRightDriveEncoder, rightPMult, rightIMult, rightDMult)
  );
}

void autoRightSwingTurn(double degrees) {
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

void autoLeftPivot(double degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = inchesPerPivotDegree * leftTicksPerInch * degrees + getLeftDriveEncoder;
  int rightTicks = -(inchesPerPivotDegree * rightTicksPerInch * degrees + getRightDriveEncoder);

  drive(
    leftPID(leftTicks, getLeftDriveEncoder, leftPMult, leftIMult, leftDMult),
    rightPID(rightTicks, getRightDriveEncoder, rightPMult, rightIMult, rightDMult)
  );
}

void autoRightPivot(double degrees) {
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
////////////////////////// Auto Groups //////////////////////////
****************************************************************/

void leftAuto() {
  //drive off bar
  autoDrive(4);
  //turn to match tile
  autoLeftSwingTurn(45);
  //raise lift to clear base
  autoLift(127, 127, 1.0);
  //drive to right mobile base
  autoDrive(30);
  //move arm to front limit
  autoArm(127, 1.0);
  //lift base
  autoBaseLift(127, 127, 0.5);

}

/////////////////////////////////////////////////////////////////

void rightAuto() {
  //drive off bar
  autoDrive(4);
  //turn to match tile
  autoRightSwingTurn(45);
  //raise lift to clear base
  autoLift(127, 127, 1.0);
  //drive to right mobile base
  autoDrive(30);
  //move arm to front limit
  autoArm(127, 1.0);
  //lift base
  autoBaseLift(127, 127, 0.5);

}


/****************************************************************
///////////////////////////// Main //////////////////////////////
****************************************************************/
void pre_auton() {
  //setup
  nullifyLeftDriveEncoder();
  nullifyRightDriveEncoder();
  lcdAutonomousSelector();
}

/////////////////////////////////////////////////////////////////

task autonomous() {
  runAuto();
}

/////////////////////////////////////////////////////////////////

task usercontrol() {
  //setup
  nullifyLeftDriveEncoder();
  nullifyRightDriveEncoder();

/////////////////////////////////////////////////////////////////

  //loop
  while(true) {

    //driver
    if(VexRT(Btn6U)) {
      waitUntilFalse(VexRT(Btn6U));
      tankMode = not tankMode;
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
