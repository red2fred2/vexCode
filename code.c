#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, dgtl1,  leftDriveEncoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightDriveEncoder, sensorQuadEncoder)
#pragma config(Motor,  port2,           rightLift,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           baseLiftRight, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           armSlide,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           clawMotor,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           leftLift,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           baseLiftLeft,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           leftDrive,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           rightDrive,    tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"


/****************************************************************
////////////////////////// Constants ////////////////////////////
****************************************************************/

//general

const float expoMult                     = 0.039579;
const int expoPow                        = 3;

const float inchesPerWheelRotation       = 1.0;
const float inchesPerSwingTurnDegree     = 1.0;
const float inchesPerPivotDegree         = 1.0;

const float leftEncoderTicksPerRotation  = 100.0;
const float rightEncoderTicksPerRotation = 100.0;

const int leftTicksPerInch = (int)(leftEncoderTicksPerRotation / inchesPerWheelRotation);
const int rightTicksPerInch = (int)(rightEncoderTicksPerRotation / inchesPerWheelRotation);

////////////////////////////////////////////////////////////////

//lcd

const int lcdLeft   = 1;
const int lcdCenter = 2;
const int lcdRight  = 4;
const int lcdDelay  = 5;

string lcdEnterString = "<         Enter        >";
string lcdIsRunningString = "is running!";

////////////////////////////////////////////////////////////////

//drivetrain
const int leftDriveDeadzone  = 10;
const float leftDriveMult   = 1.0;
const int rightDriveDeadzone = 10;
const float rightDriveMult  = 1.0;

const float Pmult           = 1.0;
const float Imult           = 1.0;
const float Dmult           = 1.0;

////////////////////////////////////////////////////////////////

//lift
const int leftLiftDeadzone  = 10;
const float leftLiftMult   = 1.0;
const int rightLiftDeadzone = 10;
const float rightLiftMult  = 1.0;

/////////////////////////////////////////////////////////////////

//arm
const int armDeadzone = 10;
const float armMult  = 1.0;

/////////////////////////////////////////////////////////////////

//baseLift
const int leftBaseLiftDeadzone  = 10;
const float leftBaseLiftMult   = 1.0;
const int rightBaseLiftDeadzone = 10;
const float rightBaseLiftMult  = 1.0;

/////////////////////////////////////////////////////////////////

//claw
const int clawDeadzone = 10;
const float clawMult  = 1.0;


/****************************************************************
////////////////////////// Variables ////////////////////////////
****************************************************************/

float leftIntegral = 0;
float leftError = 0;
float rightIntegral = 0;
float rightError = 0;

bool tankMode = false;

int lcdSelection = 0;

/////////////////////////////////////////////////////////////////

//robotC is dumb... or maybe I am
int autoLiftLeftArg;
int autoLiftRightArg;
int autoLiftSecondsArg;

int autoArmPowerArg;
int autoArmSecondsArg;

int autoBaseLiftLeftArg;
int autoBaseLiftRightArg;
int autoBaseLiftSecondsArg;

int autoClawPowerArg;
int autoClawSecondsArg;

/////////////////////////////////////////////////////////////////

//synchronization locks
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

int applyMult(int value, float multiplier) {
  return (int)(value * multiplier);
}

/////////////////////////////////////////////////////////////////

int expoCurve(int in) {
  float base = expoMult * in;
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
  int deadLeft = deadzone(left, leftLiftDeadzone);
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

  if(in == lcdLeft) lcdSelection--;
  if(in == lcdRight) lcdSelection++;

  //only keep sane values
  if(lcdSelection < 0) lcdSelection = 3;
  if(lcdSelection > 3) lcdSelection = 0;

  waitUntilFalse(in);
}

/////////////////////////////////////////////////////////////////

void lcdDisplay(char * top, char * bottom) {
  clearLCDLine(0);
  clearLCDLine(1);
  displayLCDCenteredString(0, top);
  displayLCDCenteredString(1, bottom);
}

/////////////////////////////////////////////////////////////////

void lcdAutonomousSelector() {
  bool centerPressed = nLCDButtons == lcdCenter;
  bool autonomous = bIfiAutonomousMode;

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


/****************************************************************
///////////////////////// PID Functions /////////////////////////
****************************************************************/

float P(float target, float actual) {
  return Pmult * (target - actual);
}

/////////////////////////////////////////////////////////////////

float leftI(float target, float actual) {
  leftIntegral += target - actual;
  return Imult * leftIntegral;
}
float rightI(float target, float actual) {
  rightIntegral += target - actual;
  return Imult * rightIntegral;
}

/////////////////////////////////////////////////////////////////

float leftD(float target, float actual) {
  float newError = target - actual;
  float diff = newError - leftError;
  leftError = newError;
  return Dmult * diff;
}
float rightD(float target, float actual) {
  float newError = target - actual;
  float diff = newError - rightError;
  rightError = newError;
  return Dmult * diff;
}

/////////////////////////////////////////////////////////////////

int leftPID(float target, float actual) {
  return (int)(P(target, actual) + leftI(target, actual) + leftD(target, actual));
}
int rightPID(float target, float actual) {
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
  int left = vexRT(Ch3);
  int right = vexRT(Ch2);
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
  arm(forward);
}

/////////////////////////////////////////////////////////////////

task baseLiftControl() {
  int power = vexRT(Ch1Xmtr2);
  baseLift(power, power );
}

/////////////////////////////////////////////////////////////////

task modeSwitch() {
  if(vexRT(Btn6U)) {
      waitUntilFalse(vexRT(Btn6U));
      tankMode = !tankMode;
    }
}


/****************************************************************
//////////////////////// Auto Functions /////////////////////////
****************************************************************/

void autoDrive(float inches) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = leftTicksPerInch * inches + getLeftDriveEncoder();
  int rightTicks = rightTicksPerInch * inches + getRightDriveEncoder();

  drive(
    leftPID(leftTicks, getLeftDriveEncoder()),
    rightPID(rightTicks, getRightDriveEncoder())
  );
}

/////////////////////////////////////////////////////////////////

void autoLeftSwingTurn(float degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = inchesPerSwingTurnDegree * leftTicksPerInch * degrees + getLeftDriveEncoder();
  int rightTicks = 0;

  drive(
    leftPID(leftTicks, getLeftDriveEncoder()),
    rightPID(rightTicks, getRightDriveEncoder())
  );
}

void autoRightSwingTurn(float degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = 0;
  int rightTicks = inchesPerSwingTurnDegree * leftTicksPerInch * degrees + getRightDriveEncoder();

  drive(
    leftPID(leftTicks, getLeftDriveEncoder()),
    rightPID(rightTicks, getRightDriveEncoder())
  );
}

/////////////////////////////////////////////////////////////////

void autoLeftPivotTurn(float degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = -(inchesPerPivotDegree * leftTicksPerInch * degrees + getLeftDriveEncoder());
  int rightTicks = (inchesPerPivotDegree * rightTicksPerInch * degrees + getRightDriveEncoder());

  drive(
    leftPID(leftTicks, getLeftDriveEncoder()),
    rightPID(rightTicks, getRightDriveEncoder())
  );
}

void autoRightPivotTurn(float degrees) {
  resetLeftPID();
  resetRightPID();

  int leftTicks = (inchesPerPivotDegree * leftTicksPerInch * degrees + getLeftDriveEncoder());
  int rightTicks = -(inchesPerPivotDegree * rightTicksPerInch * degrees + getRightDriveEncoder());

  drive(
    leftPID(leftTicks, getLeftDriveEncoder()),
    rightPID(rightTicks, getRightDriveEncoder())
  );
}

/////////////////////////////////////////////////////////////////

void autoLift(int left, int right, float seconds) {
  lift(left, right);
  wait1Msec((int)(1000 * seconds));
}

/////////////////////////////////////////////////////////////////

void autoArm(int power, float seconds) {
  arm(power);
  wait1Msec((int)(1000 * seconds));
}

/////////////////////////////////////////////////////////////////

void autoBaseLift(int left, int right, float seconds) {
  baseLift(left, right);
  wait1Msec((int)(1000 * seconds));
}

/////////////////////////////////////////////////////////////////

void autoClaw(int power, float seconds) {
  claw(power);
  wait1Msec((int)(1000 * seconds));
}


/****************************************************************
////////////////////// Async Auto Functions /////////////////////
****************************************************************/

//robotC is stupid, so this garbage has to be here
task asyncAutoLift() {
	autoLift(autoLiftLeftArg, autoLiftRightArg, autoLiftSecondsArg);
	autoLiftFinished = true;
}

/////////////////////////////////////////////////////////////////

task asyncAutoArm() {
	autoArm(autoArmPowerArg, autoArmSecondsArg);
	autoArmFinished = true;
}

/////////////////////////////////////////////////////////////////

task asyncAutoBaseLift() {
	autoBaseLift(autoBaseLiftLeftArg, autoBaseLiftRightArg, autoBaseLiftSecondsArg);
	autoBaseLiftFinished = true;
}

/////////////////////////////////////////////////////////////////

task asyncAutoClaw() {
	autoClaw(autoClawPowerArg, autoClawSecondsArg);
	autoClawFinished = true;
}

/****************************************************************
////////////////////////// Auto Groups //////////////////////////
****************************************************************/

void pickupCone() {
  autoDrive(12);
  autoLift(-127, -127, 1.0);
  autoClaw(-127, 1.0);
  autoLift(127, 127, 1.0);
  autoArm(127, 0.75);
  autoLift(-127, -127, 0.5);
  autoClaw(-127, 1.0);
  autoLift(127, 127, 0.5);
  autoArm(-127, 0.75);
}

void leftAuto() {
  //raise lift to clear base    async
  autoLiftLeftArg = 127;
  autoLiftRightArg = 127;
  autoLiftSecondsArg = 1.0;
  startTask(asyncAutoLift);
  //move arm to front limit     async
  autoArmPowerArg = 127;
  autoArmSecondsArg = 1.0;
  startTask(asyncAutoArm);
  //drive off bar               sync
  autoDrive(4);
  //turn to match tile          sync
  autoLeftSwingTurn(45);

  waitUntilTrue(autoLiftFinished);
  waitUntilTrue(autoArmFinished);

  //open claw for a cone        async
  autoClawPowerArg = 127;
  autoClawSecondsArg = 1.0;
  startTask(asyncAutoClaw);
  //drive to right mobile base  sync
  autoDrive(36);
  //lift base                   sync
  autoBaseLift(127, 127, 0.5);
  //turn to face first cone
  autoLeftPivotTurn(90);

  waitUntilTrue(autoClawFinished);

  //pick up line of cones
  pickupCone();
  autoLeftPivotTurn(90);
  autoDrive(12);
  pickupCone();
  autoDrive(12);
  pickupCone();
  autoDrive(12);
  pickupCone();

  //drive to scoring zone and score
  autoDrive(36);
  autoLeftSwingTurn(45);
  autoBaseLift(-64, -64, 1.0);

  //back away and get ready
  autoDrive(-27);
  autoLeftPivotTurn(90 + 45);

  //drive around the right side
  autoDrive(48);
  autoLeftPivotTurn(90);
  autoDrive(24);

  //grab right center base
  autoLeftSwingTurn(45);
  autoDrive(22);
  autoBaseLift(127, 127, 1.0);

  //grab center cones
  autoLeftPivotTurn(45);
  autoDrive(6);
  pickupCone();
  autoLeftSwingTurn(45);
  autoDrive(18);
  pickupCone();
  autoRightSwingTurn(90);
  autoDrive(18);
  pickupCone();
  autoRightSwingTurn(90);
  autoDrive(18);
  pickupCone();

  //get to right center cones
  autoRightSwingTurn(90);
  autoDrive(36);
  pickupCone();
  autoRightSwingTurn(45);
  autoDrive(12);
  pickupCone();
  autoRightPivotTurn(90);
  autoDrive(12);
  pickupCone();
}

/////////////////////////////////////////////////////////////////

void rightAuto() {

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
    startTask(modeSwitch);

    if(tankMode) startTask(tankDrive);
    else startTask(joystickDrive);

    //copilot
    startTask(liftControl);
    startTask(armControl);
    startTask(baseLiftControl);
  }
}
