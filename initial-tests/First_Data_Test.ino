#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "TFLI2C.h"  

TFLI2C lidar;
Servo myservo;  // create servo object to control a servo

const int encoder1 = 2;
const int encoder2 = 3;
const int startBtn = 7;
const int motor = 12;
const int motorDir = 10;

int dir = 1;
int lastEncoded = 0; 
long encoderValue = 0;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000;

int pos = 90;    // variable to store the servo position  

Adafruit_BNO055 bno = Adafruit_BNO055(0x28);

void setup(void) {
  Serial.begin(9600);
  Wire.begin(); 
  myservo.attach(9);

  pinMode(startBtn, INPUT_PULLUP);

  pinMode(motor, OUTPUT); 
  pinMode(motorDir, OUTPUT); 
  pinMode(encoder1, INPUT_PULLUP); 
  pinMode(encoder2, INPUT_PULLUP);
  
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  delay(1000);

  while (true){
    //Serial.println("Waiting for start");
    int pinValue = digitalRead(startBtn);
    if(pinValue != 1){
      Serial.print("Started");
      break;
    }
  }
}

void loop(void) {
  currentMillis = millis();
  if (currentMillis - startMillis >= period){
    startMillis = currentMillis;
    sensors_event_t event; 
    bno.getEvent(&event);

    int16_t lidarDist;
    lidar.getData(lidarDist, 0x10); 
    Serial.print("Lidar:");
    Serial.print(lidarDist);
    /* Display the floating point data */
    int yaw = event.orientation.x;
    Serial.print(" Yaw:");
    Serial.print(event.orientation.x, 4);
    
    Serial.print(" Encoder Value:");
    Serial.print(encoderValue);

    float distance = encoderValue / 45;
    Serial.print(" Distance:");
    Serial.println(distance);
  }

}

void updateEncoder(){
  if(dir == -1) encoderValue --;
  if(dir == 1) encoderValue ++;
}

void turnMotor(){
  int startEncoder = encoderValue;

}