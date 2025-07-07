#include <Arduino.h>
#include "pindefs.h"
#include "SensorManager/SensorManager.h"
#include "CommManager/CommManager.h"
#include "DataProcessor/DataProcessor.h"
#include "drivers/MPU6050.h"
#include "drivers/BMP085.h"
#include "drivers/TCS34725.h"
#include "drivers/TFLuna.h"
#include <Servo.h>

Servo lidarServo;
Servo steeringServo;

DriverMPU6050 sensorMPU6050(&Wire1);
DriverBMP085 sensorBMP085(&Wire1);
DriverTCS34725 sensorTCS34725;
DriverTFLuna sensorTFLuna;
SensorManager sensorManager;
DataProcessor dataProcessor;

void driveFromRX();
void driveFromSerial();
void driveMotor(int throttle);
void updateEncoder();
float getYaw();

volatile int encoderPos;
int throttle;
int steering;

unsigned long prevMillis;

float targetYaw;

void setup(){

  Serial.begin();

  lidarServo.attach(PIN_LIDAR_SERVO);
  steeringServo.attach(PIN_STEERING_SERVO);

  Wire1.setSCL(PIN_I2C1_SCL);
  Wire1.setSDA(PIN_I2C1_SDA);
  Wire.setSCL(PIN_I2C0_SCL);
  Wire.setSDA(PIN_I2C0_SDA);

  sensorManager.addSensor(&sensorMPU6050);
  sensorManager.addSensor(&sensorBMP085);
  sensorManager.addSensor(&sensorTCS34725);
  sensorManager.addSensor(&sensorTFLuna);
  sensorManager.initializeSensors();

  pinMode(PIN_RX_CH1, INPUT);
  pinMode(PIN_RX_CH2, INPUT);

  pinMode(PIN_TB6612_BIN1, OUTPUT);
  pinMode(PIN_TB6612_BIN2, OUTPUT);
  pinMode(PIN_TB6612_PWMB, OUTPUT);
  pinMode(PIN_TB6612_STBY, OUTPUT);

  pinMode(PIN_MOTOR_ENCA, INPUT);
  pinMode(PIN_MOTOR_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCA), updateEncoder, CHANGE);

}

void loop(){

  //driveFromRX();
  //driveFromSerial(); 


  // change target yaw if dist < 80 cm
  
  // steering proportional to ya error
  // yawDiff = targetYaw - getYaw
  // steering = constrain(steering + yawDiff * constant, 0, 180);
  
  targetYaw = 90;

  float yawDiff = targetYaw - getYaw();
  steering = constrain(90 + yawDiff * 1, 0, 180);
  
  Serial.println("Yaw: " + String(getYaw()) + " Target Yaw: " + String(targetYaw) + " Steering: " + String(steering));

}

void driveFromRX(){

  steering = pulseIn(PIN_RX_CH1, HIGH);
  throttle = pulseIn(PIN_RX_CH2, HIGH);

  steering = constrain(map(steering, 1000, 2000, 0, 180), 0, 180);
  throttle = constrain(map(throttle, 1000, 2000, -800, 800), -800, 800);

  steeringServo.write(steering);
  driveMotor(throttle);

}

void driveMotor(int throttle){

  digitalWrite(PIN_TB6612_STBY, HIGH); 
  analogWriteResolution(255);

  if(throttle > 0){

    digitalWrite(PIN_TB6612_BIN1, HIGH);
    digitalWrite(PIN_TB6612_BIN2, LOW);
    analogWrite(PIN_TB6612_PWMB, map(throttle, 0, 1024, 0, 255));

  }
  else{

    digitalWrite(PIN_TB6612_BIN1, LOW);
    digitalWrite(PIN_TB6612_BIN2, HIGH);
    analogWrite(PIN_TB6612_PWMB, map(-throttle, 0, 1024, 0, 255));

  }

}

void updateEncoder(){

  if(throttle > 0){
    encoderPos++;
  }
  else{
    encoderPos--;
  }

}

void driveFromSerial(){

  if(Serial.available()){

    String input = Serial.readString();

    int commaIndex = input.indexOf(',');

    String throttleString = input.substring(0, commaIndex);
    String steeringString = input.substring(commaIndex + 1);

    throttle = throttleString.toInt();
    steering = steeringString.toInt();

  }

}

float getYaw(){

  std::vector<sensors_event_t> sensorData = sensorManager.updateSensors();

  static float yaw = 0.0f;

    for(sensors_event_t event : sensorData){

    if(event.type == SENSOR_TYPE_GYROSCOPE){

      unsigned long currentMillis = millis();
      float dT = (currentMillis - prevMillis) / 1000.0f; // seconds
      prevMillis = currentMillis;

      // event.gyro.z is in rad/s, convert to deg/s and integrate
      yaw -= event.gyro.z * (180.0f / PI) * dT;

    }

  }

  return yaw;

}