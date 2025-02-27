#include <RMCS2303drive.h>
#include <SoftwareSerial.h>

RMCS2303 rmcs;
SoftwareSerial myserial(2, 3);

byte left_motor = 4;
byte right_motor = 7;
int INP_CONTROL_MODE = 257;
int PP_gain = 36;
int PI_gain = 18;
int VF_gain = 38;
int LPR = 13;
int acceleration = 5000;
int speed = 0;

#define joystick_X A0
#define joystick_Y A1

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing RMCS-2303...");

  rmcs.Serial_selection(1);
  rmcs.Serial0(9600);
  rmcs.begin(&myserial, 9600);

  Serial.println("Writing Motor Parameters...");
  rmcs.WRITE_PARAMETER(left_motor, INP_CONTROL_MODE, PP_gain, PI_gain, VF_gain, LPR, acceleration, speed);
  rmcs.WRITE_PARAMETER(right_motor, INP_CONTROL_MODE, PP_gain, PI_gain, VF_gain, LPR, acceleration, speed);

  Serial.println("Reading Motor Parameters...");
  rmcs.READ_PARAMETER(left_motor);
  rmcs.READ_PARAMETER(right_motor);

  pinMode(joystick_X, INPUT);
  pinMode(joystick_Y, INPUT);

  Serial.println("Setup Complete!");
}

void loop() {
  int rawX = analogRead(joystick_X);
  int rawY = analogRead(joystick_Y);

  Serial.print("Raw X: "); Serial.print(rawX);
  Serial.print(" | Raw Y: "); Serial.println(rawY);

  int centerX = 531;
  int centerY = 499;
  int xValue = rawX - centerX;
  int yValue = rawY - centerY;

  int deadZone = 50;
  if (abs(xValue) < deadZone) xValue = 0;
  if (abs(yValue) < deadZone) yValue = 0;

  int motorLeftVal = xValue + yValue;
  int motorRightVal = yValue - xValue;

  byte leftMotorDir = (motorLeftVal < 0) ? 1 : 0;
  byte rightMotorDir = (motorRightVal < 0) ? 1 : 0;

  motorLeftVal = abs(motorLeftVal);
  motorRightVal = abs(motorRightVal);

  Serial.print("Left Motor: ");
  Serial.print(motorLeftVal);
  Serial.print(" (Dir: ");
  Serial.print(leftMotorDir);
  Serial.print("), Right Motor: ");
  Serial.print(motorRightVal);
  Serial.print(" (Dir: ");
  Serial.print(rightMotorDir);
  Serial.println(")");

  Serial.println("Writing Left Motor Speed...");
  rmcs.Speed(left_motor, motorLeftVal * 15);
  rmcs.Enable_Digital_Mode(left_motor, leftMotorDir);

  Serial.println("Writing Right Motor Speed...");
  rmcs.Speed(right_motor, motorRightVal * 15);
  rmcs.Enable_Digital_Mode(right_motor, rightMotorDir);
}
