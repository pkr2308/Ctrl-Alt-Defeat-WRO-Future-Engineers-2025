#include <Arduino.h>

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <TFLI2C.h>

TFLI2C lidar;
Servo myservo;

const int encoder1 = 2;
const int encoder2 = 3;
const int startBtn = 8;
const int motor = 12;
const int motorDir = 10;

int dir = 1;
int lastEncoded = 0; 
long encoderValue = 0;
int target = 1000;
int turns = 0;
bool turning = false;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 200;
const unsigned long motorPeriod = 100;

int pos = 90;    // variable to store the servo position  
float yaw;
int threshold = 65;
int16_t lidarDist;
int16_t startDist = 50;

Adafruit_BNO055 bno = Adafruit_BNO055(0x28);

int turnDir = 1;
int targetYaw = 0;
int startYaw = 0;

float distance = 0.0;

// put function declarations here:
void updateEncoder();
void forward(int pwm);
void backward();
void stop();
void steer(int angle);
void checkYaw();
void pStraight();

void setup() {
  // put your setup code here, to run once:
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
  
  delay(2000);

  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  bno.setExtCrystalUse(true);
  lidar.getData(startDist, 0x10);
  /*
  while (true){
    //Serial.println("Waiting for start");
    int pinValue = digitalRead(startBtn);
    Serial.println("Waiting");
    if(pinValue != 1){
      Serial.print("Started");
      break;
    }
  }
  */
  forward(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t event; 
  bno.getEvent(&event);

  yaw = event.orientation.x;

  distance = encoderValue / 45;
  currentMillis = millis();
  
  if (currentMillis - startMillis >= period){
    startMillis = currentMillis;
    
    Serial.print("Lidar:");
    Serial.print(lidarDist);
    
    Serial.print(" Yaw:");
    Serial.print(yaw);

    Serial.print(" Target Yaw:");
    Serial.print(targetYaw);
    
    Serial.print(" Encoder Value:");
    Serial.print(encoderValue);

    Serial.print(" Distance:");
    Serial.print(distance);

    Serial.print(" Direction:");
    Serial.print(dir);

    Serial.print(" Turns:");
    Serial.println(turns);
  }

  checkYaw();

  lidar.getData(lidarDist, 0x10); 
  if ((lidarDist < threshold) && (turning == false) && (turns == 0 or distance > 150)){
    steer(90);
    turning = true;
  }
  
  if (turns == 12 && lidarDist <= startDist && distance > 10){ 
    stop();
    while (true){}
    //Serial.print("Finished");
  }
  //if ( (dir == 1 && encoderValue > target) or (dir == -1 && encoderValue < target)) stop();

  pStraight();
}

// put function definitions here:
void updateEncoder(){
  if(dir == -1) encoderValue --;
  if(dir == 1) encoderValue ++;
}

void forward(int pwm){
  digitalWrite(motorDir,HIGH);
  analogWrite(motor,pwm);
}

void backward(int pwm){
  digitalWrite(motorDir,LOW);
  analogWrite(motor,pwm);
}

void stop(){
  digitalWrite(motor,0);
  digitalWrite(motorDir,LOW);
}

void steer(int angle){
  myservo.write(90 + turnDir*42);
  startYaw = yaw;
  targetYaw = yaw + angle*turnDir;
  if (targetYaw > 360) targetYaw = targetYaw - 360;
  if (targetYaw > 85 && targetYaw < 95) targetYaw = 90;
  else if (targetYaw > 175 && targetYaw < 185) targetYaw = 180;
  else if (targetYaw > 265 && targetYaw < 275) targetYaw = 270;
  else if (targetYaw > 355 or targetYaw < 5) targetYaw = 0;
}

void checkYaw(){
  float difference = targetYaw - yaw;
  if (abs(difference) < 3 && turning == true){
    myservo.write(90);
    turning = false;
    encoderValue = 0;
    distance = 0;
    turns ++;
  }
}

void pStraight(){
  if(turning == false){
    int correction = 0;
    int error = round(targetYaw - yaw);
    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;
    
    if (correction > 0) correction = correction * 1.5; // correction to the right
    else if (correction < 0) correction = correction * 1.7; // correction to the left

    if (correction > 45) correction = 45;
    else if (correction < -45) correction = -45;

    myservo.write(90 + correction);
    // Serial.println("Correction: ");
    // Serial.println(correction);
  }
}