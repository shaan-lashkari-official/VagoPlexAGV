#include <RMCS2303drive.h>
#include <SoftwareSerial.h>

RMCS2303 rmcs;
SoftwareSerial myserial(2, 3);

byte left_motor = 4;
byte right_motor = 7;
int acceleration = 5000;
int speed = 0;

#define joystick_X A0
#define joystick_Y A1

void setup() {
  Serial.begin(9600);
  rmcs.begin(&myserial, 9600);

  rmcs.WRITE_PARAMETER(left_motor, 257, 36, 18, 38, 13, acceleration, speed);
  rmcs.WRITE_PARAMETER(right_motor, 257, 36, 18, 38, 13, acceleration, speed);

  pinMode(joystick_X, INPUT);
  pinMode(joystick_Y, INPUT);
}

void loop() {
  int rawX = analogRead(joystick_X);  
  int rawY = analogRead(joystick_Y);
  
  int xValue = rawX - 531;
  int yValue = rawY - 499;

  int motorLeft = xValue + yValue;  
  int motorRight = yValue - xValue; 


  byte leftMotorDir = 0;
  byte rightMotorDir = 0;

  if (motorLeft < 0) {
    leftMotorDir = 1;
    motorLeft = -motorLeft;
  }

  if (motorRight < 0) {
    rightMotorDir = 1;
    motorRight = -motorRight;
  }


  Serial.print("L: "); Serial.print(motorLeft);
  Serial.print(" (Dir: "); Serial.print(leftMotorDir);
  Serial.print("), R: "); Serial.print(motorRight);
  Serial.print(" (Dir: "); Serial.print(rightMotorDir);
  Serial.println(")");


  rmcs.Speed(left_motor, motorLeft * 15);
  rmcs.Enable_Digital_Mode(left_motor, leftMotorDir);

  rmcs.Speed(right_motor, motorRight * 15);
  rmcs.Enable_Digital_Mode(right_motor, rightMotorDir);

  delay(100);  
}
