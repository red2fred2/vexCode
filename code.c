#pragma platform(VEX2)
#pragma competitionControl(Competition)

#pragma config(UART_Usage, UART2, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(Sensor, dgtl1, leftDriveEncoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl3, rightDriveEncoder, sensorQuadEncoder)

#pragma config(Motor, port1, leftDrive, tmotorNone, openLoop)
#pragma config(Motor, port2, rightDrive, tmotorNone, openLoop)
#pragma config(Motor, port3, leftLift, tmotorNone, openLoop)
#pragma config(Motor, port4, rightLift, tmotorNone, openLoop)
#pragma config(Motor, port5, armSlide, tmotorNone, openLoop)
#pragma config(Motor, port6, baseLiftLeft, tmotorNone, openLoop)
#pragma config(Motor, port7, baseLiftRight, tmotorNone, openLoop)
#pragma config(Motor, port8, clawMotor, tmotorNone, openLoop)

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

const string lcdEnterString = "<         Enter        >";
const string lcdIsRunningString = "is running!";

////////////////////////////////////////////////////////////////

//drivetrain
const int leftDriveDeadzone  = 10;
const double leftDriveMult   = 1.0;
const int rightDriveDeadzone = 10;
const double rightDriveMult  = 1.0;

const double Pmult           = 1.0;
const double Imult           = 1.0;
const double Dmult           = 1.0;

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

/////////////////////////////////////////////////////////////////

//synchronization locks
bool autoDriveFinished = false;
bool autoLeftSwingTurnFinished = false;
bool autoRightSwingTurnFinished = false;
bool autoLeftPivotTurnFinished = false;
bool autoLiftFinished = false;
bool autoBaseLiftFinished = false;
bool autoArmFinished = false;
bool autoClawFinished = false;


/****************************************************************
/////////////////////// Utility Functions ///////////////////////
****************************************************************/

int deadzone(int value, int deadzone) {
    if(value > deadzone) return value;
    else return 0;
}

/////////////////////////////////////////////////////////////////

int applyMult(int value, double multiplier) {
  return (int)(value * multiplier);
}

/////////////////////////////////////////////////////////////////

int expoCurve(int in) {
  double base = expoMult * in;
  return (int)(pow(base, expoPow));
}

/////////////////////////////////////////////////////////////////

void waitUntilTrue(bool in) {
  while(!in) {}
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

  int deadRight = deadzone(right, rightDriveDeadzone);
  int multRight = applyMult(deadRight, rightDriveMult);
  motor[rightDrive] = multRight;
}

/////////////////////////////////////////////////////////////////

void expoDrive(int left, int right) {
  int deadLeft = deadzone(left, leftDriveDeadzone);
  int expoLeft = expoCurve(deadLeft);
  int multLeft = applyMult(expoLeft, leftDriveMult);
  motor[leftDrive] = multLeft;

  int deadRight = deadzone(right, rightDriveDeadzone);
  int expoRight = expoCurve(deadRight);
  int multRight = applyMult(expoRight, rightDriveMult);
  motor[rightDrive] = multRight;
}

/////////////////////////////////////////////////////////////////

void lift(int left, int right) {
  int deadleft = deadzone(left, leftLiftDeadzone);
  int multLeft = applyMult(deadLeft, leftLiftMult);
  motor[leftLift] = multLeft;

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
  motor[clawMotor] = multPower;
}

/////////////////////////////////////////////////////////////////

int leftDriveEncoderNullifier = 0;
int getLeftDriveEncoder() {
  return SensorValue[leftDriveEncoder] - leftDriveEncoderNullifier;
}
void nullifyLeftDriveEncoder() {
  leftDriveEncoderNullifier = getLeftDriveEncoder();
}

/////////////////////////////////////////////////////////////////

int rightDriveEncoderNullifier = 0;
int getRightDriveEncoder() {
  return SensorValue[rightDriveEncoder] - rightDriveEncoderNullifier;
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

void lcdDisplay() {
  clearLCDLine(0);
  clearLCDLine(1);
  displayLCDCenteredString(0, "top");
  displayLCDCenteredString(1, "bottom");
}

/////////////////////////////////////////////////////////////////

void lcdAutonomousSelector() {
  bool centerPressed = nLCDButtons == lcdCenter;
  bool autonomous = bIfiAutonomousMode;

  lcdDisplay();

  while(!(centerPressed || autonomous)) {
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
  leftIntegral += target - actual;
  return Imult * leftIntegral;
}
double rightI(double target, double actual) {
  rightIntegral += target - actual;
  return Imult * rightIntegral;
}

/////////////////////////////////////////////////////////////////

double leftD(double target, double actual) {
  double newError = target - actual;
  double diff = newError - leftError;
  leftError = newError;
  return Dmult * newError;
}
double rightD(double target, double actual) {
  double newError = target - actual;
  double diff = newError - rightError;
  rightError = newError;
  return Dmult * newError;
}

/////////////////////////////////////////////////////////////////

int leftPI(double target, double actual) {
  return (int)(P(target, actual) + leftI(target, actual));
}
int rightPI(double target, double actual) {
  return (int)(P(target, actual) + rightI(target, actual));
}

/////////////////////////////////////////////////////////////////

int rightPD(double target, double actual) {
  return (int)(P(target, actual) + leftD(target, actual));
}
int rightPD(double target, double actual) {
  return (int)(P(target, actual) + leftD(target, actual));
}

/////////////////////////////////////////////////////////////////

int leftID(double target, double actual) {
  return (int)(leftI(target, actual) + leftD(target, actual));
}
int rightID(double target, double actual) {
  return (int)(rightI(target, actual) + rightD(target, actual));
}

/////////////////////////////////////////////////////////////////

int leftPID(double target, double actual) {
  return (int)(P(target, actual) + leftI(target, actual) + leftD(target, actual));
}
int rightPID(double target, double actual) {
  return (int)(P(target, actual) + rightI(target, actual) + rightD(target, actual));
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

task tankDrive() {
  int left = VexRT(Ch3);
  int right = VexRT(Ch2);
  expoDrive(left, right);
}

/////////////////////////////////////////////////////////////////

task joystickDrive() {
  int up = vexRT(Ch3);
  int right = vexRT(Ch4);
  expoDrive(up + right, up - right);
}

/////////////////////////////////////////////////////////////////

task liftControl() {
  int left = vexRT(Ch3Xmtr2);
  int right = vexRT(Ch2Xmtr2);
  lift(left, right);
}

/////////////////////////////////////////////////////////////////

task armControl() {
  int forward = vexRT(Ch4Xmtr2);
}

/////////////////////////////////////////////////////////////////

task baseLiftControl() {
  int power = VexRT(Ch1Xmtr2);
  baseLift(power);
}

/////////////////////////////////////////////////////////////////

task modeSwitch() {
  if(VexRT(Btn6U)) {
      waitUntilFalse(VexRT(Btn6U));
      tankMode = !tankMode;
    }
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

void autoLeftPivotTurn(double degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = -inchesPerPivotDegree * leftTicksPerInch * degrees + getLeftDriveEncoder;
  int rightTicks = (inchesPerPivotDegree * rightTicksPerInch * degrees + getRightDriveEncoder);

  drive(
    leftPID(leftTicks, getLeftDriveEncoder, leftPMult, leftIMult, leftDMult),
    rightPID(rightTicks, getRightDriveEncoder, rightPMult, rightIMult, rightDMult)
  );
}

void autoRightPivotTurn(double degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = (inchesPerPivotDegree * leftTicksPerInch * degrees + getLeftDriveEncoder);
  int rightTicks = -inchesPerPivotDegree * rightTicksPerInch * degrees + getRightDriveEncoder;

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

/////////////////////////////////////////////////////////////////

void autoClaw(int power, double seconds) {
  claw(power);
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
    StartTask(modeSwitch);

    if(tankMode) StartTask(tankDrive);
    else StartTask(joystickDrive);

    //copilot
    StartTask(liftControl);
    StartTask(armControl);
    StartTask(baseLiftControl);
  }
}
