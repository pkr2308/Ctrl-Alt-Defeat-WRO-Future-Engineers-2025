#include <Arduino.h>
#include "pindefs.h"
#include "SensorManager/SensorManager.h"
#include "CommManager/CommManager.h"
#include "drivers/MPU6050.h"
#include <Servo.h>

Servo lidarServo;
Servo steeringServo;

DriverMPU6050 sensorMPU6050(&Wire1);
SensorManager sensorManager;


void setup(){

  Serial.begin();
  delay(5000);
  Serial.println("test");

  lidarServo.attach(PIN_LIDAR_SERVO);
  steeringServo.attach(PIN_STEERING_SERVO);

  Wire1.setSCL(PIN_I2C1_SCL);
  Wire1.setSDA(PIN_I2C1_SDA);

  sensorManager.addSensor(&sensorMPU6050);
  sensorManager.initializeSensors();

}

void loop(){



}