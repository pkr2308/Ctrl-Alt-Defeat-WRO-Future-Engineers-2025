#include <Arduino.h>
#include "pindefs.h"
#include "SensorManager/SensorManager.h"
#include "CommManager/CommManager.h"
#include "drivers/MPU6050.h"
#include "drivers/BMP085.h"
#include <Servo.h>

Servo lidarServo;
Servo steeringServo;

DriverMPU6050 sensorMPU6050(&Wire1);
DriverBMP085 sensorBMP085(&Wire1);
SensorManager sensorManager;


void setup(){

  Serial.begin();
  delay(5000);

  lidarServo.attach(PIN_LIDAR_SERVO);
  steeringServo.attach(PIN_STEERING_SERVO);

  Wire1.setSCL(PIN_I2C1_SCL);
  Wire1.setSDA(PIN_I2C1_SDA);

  sensorManager.addSensor(&sensorMPU6050);
  sensorManager.addSensor(&sensorBMP085);
  sensorManager.initializeSensors();

  std::vector<sensors_event_t> sensorData = sensorManager.updateSensors();

  for(auto event : sensorData){

    Serial.println("Sensor Type: " + String(event.type));

    if(event.type == SENSOR_TYPE_ACCELEROMETER){
      Serial.print("Acceleration X: ");
      Serial.println(event.acceleration.x);
      Serial.print("Acceleration Y: ");
      Serial.println(event.acceleration.y);
      Serial.print("Acceleration Z: ");
      Serial.println(event.acceleration.z);
    }
    else if(event.type == SENSOR_TYPE_GYROSCOPE){
      Serial.print("Gyro X: ");
      Serial.println(event.gyro.x);
      Serial.print("Gyro Y: ");
      Serial.println(event.gyro.y);
      Serial.print("Gyro Z: ");
      Serial.println(event.gyro.z);
    }
    else if(event.type == SENSOR_TYPE_PRESSURE){
      Serial.print("Pressure: ");
      Serial.println(event.pressure);
    }

  }


}

void loop(){



}