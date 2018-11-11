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

int LeftParallelCheck(){  //checking parallel of left side of traler
  DistanceCheck();
  if (Distance[LeftFrontSensor] && Distance[LeftBackSensor])
    if ((Distance[LeftFrontSensor] - Distance[LeftBackSensor]) > ParallelConst)
      return false;
    else
      return true;
  else
    return false;
}

int RightParallelCheck(){ ////checking parallel of right side of traler
  DistanceCheck();
  if (Distance[RightFrontSensor] && Distance[RightBackSensor])
    if ((Distance[RightFrontSensor] - Distance[RightBackSensor]) > ParallelConst)
      return false;
    else
      return true;
  else
    return false;
}

void Forward(){}
void Backward(){}
void LeftBackward(int Angle){}
void RightBackward(int Angle){}
 
void setup() {
   Serial.begin(9600);
   BlueTooth.begin(9600);
}
 
void loop() 
{
  if (BlueTooth.available()){
    BT_input=(BlueTooth.read());
    switch (BT_input) {
      case 0:
      // statements
      break;
      case 1:
      // statements
      break;
      default:
      // defoult statements
      break;
    } 
  }
}