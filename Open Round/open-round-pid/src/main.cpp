#include "Arduino.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BNO055.h"
#include "Servo.h"
#include "TFLI2C.h"
#include "PID_v1.h"
#include "pindefs.hpp"

unsigned long lastLoopTime = 0;
unsigned long currentLoopTime = 0;
const unsigned long loopInterval = 50;

const double PID_P = 5;
const double PID_I = 0.5;
const double PID_D = 0.5;

double yawError = 0;
double yawTarget = 0;
double pidOutput = 0;
double currentYaw = 0;
double adjustedYawTarget = 0;

double distance = 0;
int encoderValue = 0;
int direction = 1;    // 1 for forward, -1 for backward

const double MAX_PID_OUTPUT = 200;
const double MAX_STEERING_DEFLECTION = 45;
const double STEERING_CENTER = 90;
int servoAngle = STEERING_CENTER;
double servoAdjustment = 0;

int16_t lidarDistance = 0;

double getShortestAngleError(double target, double current);
void commandSteering(double pidCommand);
void printDataToOLED();
void drive(int speed);
void updateEncoder();

Adafruit_SSD1306 oled(128, 64, &Wire1, -1);
Adafruit_BNO055 bno = Adafruit_BNO055(0x28);
PID steeringPID(&currentYaw, &pidOutput, &adjustedYawTarget, PID_P, PID_I, PID_D, DIRECT);
TFLI2C lidar;
Servo steeringServo;


void setup(){

  Serial.begin(115200);

  Wire1.setSCL(PIN_I2C1_SCL);
  Wire1.setSDA(PIN_I2C1_SDA);
  Wire.setSCL(PIN_I2C0_SCL);
  Wire.setSDA(PIN_I2C0_SDA);  

  pinMode(PIN_TB6612_BIN1, OUTPUT);
  pinMode(PIN_TB6612_BIN2, OUTPUT);
  pinMode(PIN_TB6612_PWMB, OUTPUT);
  pinMode(PIN_TB6612_STBY, OUTPUT);

  pinMode(PIN_MOTOR_ENCA, INPUT_PULLUP);
  pinMode(PIN_MOTOR_ENCB, INPUT_PULLUP);

  steeringServo.attach(PIN_STEERING_SERVO);

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  if(!bno.begin()) {
    Serial.println("BNO055 not found");
  }

  bno.setExtCrystalUse(true);

  delay(3000);

  steeringPID.SetMode(AUTOMATIC);
  steeringPID.SetOutputLimits(-MAX_PID_OUTPUT, MAX_PID_OUTPUT);
  //steeringPID.SetSampleTime(loopInterval);

  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCA), updateEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCB), updateEncoder, CHANGE);
}

void loop(){

  currentLoopTime = millis();

  if(currentLoopTime - lastLoopTime >= loopInterval) {
    lastLoopTime = currentLoopTime;

    lidar.getData(lidarDistance, 0x20);

    sensors_event_t event; 
    bno.getEvent(&event);

    currentYaw = event.orientation.x;
    yawError = getShortestAngleError(yawTarget, currentYaw);

    adjustedYawTarget = yawTarget;
    if(currentYaw - adjustedYawTarget > 180){
      adjustedYawTarget += 360;
    }
    else if(currentYaw - adjustedYawTarget < -180){
      adjustedYawTarget -= 360;
    }

    Serial.print("yawTarget: ");
    Serial.print(yawTarget);
    Serial.print(" currentYaw: ");
    Serial.print(currentYaw);
    Serial.print(" yawError: ");
    Serial.println(yawError);
    
    steeringPID.Compute();

    commandSteering(pidOutput);

  }

  printDataToOLED();

  drive(150);
  analogWriteResolution(1024);

}

double getShortestAngleError(double target, double current) {
 
  double error = target - current;

  if(error > 180){
    error -= 360;
  }

  else if(error < -180){
    error += 360;
  }

  return error;

}

void commandSteering(double pidCommand){

  servoAdjustment = map(pidCommand, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, -86, 86);
  servoAngle = STEERING_CENTER + servoAdjustment;

  steeringServo.write(servoAngle);

}

void printDataToOLED(){

  oled.clearDisplay();

  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);

  oled.print("Measured Yaw: ");
  oled.println(currentYaw);
  oled.print("Target Yaw: ");
  oled.println(yawTarget);
  oled.print("Yaw Error: ");
  oled.println(yawError);
  oled.print("PID Output: ");
  oled.println(pidOutput);
  oled.print("Servo adj: ");
  oled.println(servoAdjustment);
  oled.print("Servo Angle: ");
  oled.println(servoAngle);
  oled.print("LiDAR Distance: ");
  oled.println(lidarDistance);
  oled.print("Distance: ");
  oled.println(distance);

  oled.display();

}

void drive(int speed){

  digitalWrite(PIN_TB6612_STBY, HIGH);

  if(speed > 0){

    direction = 1;
    digitalWrite(PIN_TB6612_BIN1, LOW);
    digitalWrite(PIN_TB6612_BIN2, HIGH);
    analogWrite(PIN_TB6612_PWMB, speed);

  }
  else{

    direction = -1;
    digitalWrite(PIN_TB6612_BIN1, HIGH);
    digitalWrite(PIN_TB6612_BIN2, LOW);
    analogWrite(PIN_TB6612_PWMB, speed);

  }

  digitalWrite(PIN_TB6612_STBY, LOW);

}

void updateEncoder(){

  if(direction == 1) encoderValue ++;
  if(direction == -1) encoderValue --;

  distance = encoderValue / 45;

}

void setup1(){

  delay(3000);  

}

void loop1(){
    
  delay(3000);
  yawTarget = fmod(yawTarget -= 90, 360.0);

}