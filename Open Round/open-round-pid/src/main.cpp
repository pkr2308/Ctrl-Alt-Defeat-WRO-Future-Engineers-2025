#include "Arduino.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BNO055.h"
#include "TFLI2C.h"
#include "PID_v1.h"
#include "pindefs.hpp"

unsigned long lastLoopTime = 0;
unsigned long currentLoopTime = 0;
const unsigned long loopInterval = 50;

const double PID_P = 2;
const double PID_I = 0;
const double PID_D = 0;

double yawError = 0;
double yawTarget = 0;
double pidOutput = 0;
double currentYaw = 0;

const double MAX_PID_OUTPUT = 200;
const double MAX_STEERING_DEFLECTION = 45;
const double STEERING_CENTER = 90;
int servoAngle = STEERING_CENTER;
double servoAdjustment = 0;


double getShortestAngleError(double target, double current);
void commandSteering(double pidCommand);
void printDataToOLED();
void drive(int speed);

Adafruit_SSD1306 oled(128, 64, &Wire1, -1);
Adafruit_BNO055 bno = Adafruit_BNO055(0x28);
PID steeringPID(&currentYaw, &pidOutput, &yawTarget, PID_P, PID_I, PID_D, DIRECT);


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

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  if(!bno.begin()) {
    Serial.println("BNO055 not found");
  }

  bno.setExtCrystalUse(true);

  steeringPID.SetMode(AUTOMATIC);
  steeringPID.SetOutputLimits(-MAX_PID_OUTPUT, MAX_PID_OUTPUT);
  //steeringPID.SetSampleTime(loopInterval);

}

void loop(){

  currentLoopTime = millis();

  if(currentLoopTime - lastLoopTime >= loopInterval) {
    lastLoopTime = currentLoopTime;

    sensors_event_t event; 
    bno.getEvent(&event);

    currentYaw = event.orientation.x;
    yawError = getShortestAngleError(yawTarget, currentYaw);
    yawTarget = 90;

    Serial.print("yawTarget: ");
    Serial.print(yawTarget);
    Serial.print(" currentYaw: ");
    Serial.print(currentYaw);
    Serial.print(" yawError: ");
    Serial.print(yawError);

    Serial.println();
    
    steeringPID.Compute();

    commandSteering(pidOutput);

  }

  printDataToOLED();

  drive(200);

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

  servoAdjustment = map(pidCommand, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, -90, 90);
  servoAngle = STEERING_CENTER + servoAdjustment;

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

  oled.display();

}

void drive(int speed){

  digitalWrite(PIN_TB6612_STBY, HIGH);

  if(speed > 0){

    digitalWrite(PIN_TB6612_BIN1, HIGH);
    digitalWrite(PIN_TB6612_BIN2, LOW);
    analogWrite(PIN_TB6612_PWMB, speed);

  }
  else{

    digitalWrite(PIN_TB6612_BIN1, LOW);
    digitalWrite(PIN_TB6612_BIN2, HIGH);
    analogWrite(PIN_TB6612_PWMB, speed);

  }

}
