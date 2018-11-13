#include <Arduino.h>
#include <NewPing.h>
#include <SoftwareSerial.h>

#define TX_PIN 5
#define RX_PIN 6
#define MAX_DISTANCE 200
#define FrontRightSensor 53
#define FrontLeftSensor 51
#define RightFrontSensor 49
#define RightBackSensor 47
#define LeftFrontSensor 45
#define LeftBackSensor 43
#define BackRightSensor 41
#define BackLeftSensor 39
#define ParallelConst 100
#define enableA 9
#define in1 7
#define in2 8
#define enableB 10
#define in3 9
#define in4 10
#define MinBuzzThreshold 70

int xAxis, yAxis;
int  x = 0;
int  y = 0;
int motorSpeedA = 0;
int motorSpeedB = 0;

SoftwareSerial BlueTooth(TX_PIN, RX_PIN); // (TXD, RXD) of HC-06

unsigned char BT_input; // to store input character received via BT.
unsigned int Distance[54];  //to store the distances from sensors in [cm]
unsigned long time = millis(); //to store the time

void DistanceCheck(){ //getting distance from all sensors
  for (int i = 39; i<54; i++){
    NewPing sonar(i, i-1, MAX_DISTANCE);
    Distance[i] = sonar.ping_cm(); 
    i++;
  }
} 


void Forward(){
  // Set Motor A forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // Set Motor B forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
  motorSpeedA = map(yAxis, 550, 1023, 0, 255);
  motorSpeedB = map(yAxis, 550, 1023, 0, 255);
}

void Backward(){
  // Set Motor A backward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Set Motor B backward
  digitalWrite(in3, HIGH);
  motorSpeedA = map(yAxis, 470, 0, 0, 255);
  motorSpeedB = map(yAxis, 470, 0, 0, 255);
}

void LeftBackward(){ // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
  int xMapped = map(xAxis, 470, 0, 0, 255); // Move to left - decrease left motor speed, increase right motor speed
  motorSpeedA = motorSpeedA - xMapped;
  motorSpeedB = motorSpeedB + xMapped; // Confine the range from 0 to 255
  if (motorSpeedA < 0)
    motorSpeedA = 0;
  if (motorSpeedB > 255) 
    motorSpeedB = 255;
}

void RightBackward(){
  int xMapped = map(xAxis, 550, 1023, 0, 255);  // Move right - decrease right motor speed, increase left motor speed
  motorSpeedA = motorSpeedA + xMapped;
  motorSpeedB = motorSpeedB - xMapped;  // Confine the range from 0 to 255
  if (motorSpeedA > 255) 
    motorSpeedA = 255;
  if (motorSpeedB < 0) 
    motorSpeedB = 0;
}

void Stop(){
  motorSpeedA = 0;
  motorSpeedB = 0;
}
 
void setup() {
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(9600);
  BlueTooth.begin(9600);
}

void loop() { // Default value - no movement when the Joystick stays in the center
  xAxis = 510;
  yAxis = 510;

  // Read the incoming data from the Smartphone Android App
  while (Serial.available() >= 2) {
    x = Serial.read();
    delay(10);
    y = Serial.read();
  }
  delay(10);
  
  // Makes sure we receive corrent values
  if ((x > 60) & (x < 220)) {
    xAxis = map(x, 220, 60, 1023, 0); // Convert the smartphone X and Y values to 0 - 1023 range, suitable motor for the motor control code below
  }
  if ((y > 60) & (y < 220)) {
    yAxis = map(y, 220, 60, 0, 1023);
  }

  // Y-axis used for forward and backward control
  if (yAxis < 470)
    Backward();
  else if (yAxis > 550)
    Forward();
  // If joystick stays in middle the motors are not moving
  else {
    Stop();
  }

  // X-axis used for left and right control
  if (xAxis < 470) {
    LeftBackward();
  }
  if (xAxis > 550) {  // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    RightBackward();
  }

  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (motorSpeedA < MinBuzzThreshold) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < MinBuzzThreshold) {
    motorSpeedB = 0;
  }
  analogWrite(enableA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enableB, motorSpeedB); // Send PWM signal to motor B
}
