#include <Arduino.h>
#include "pindefs.h"
#include "SensorManager/SensorManager.h"
#include "CommManager/CommManager.h"
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


void setup(){

  Serial.begin();
  delay(5000);

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
    else if(event.type == SENSOR_TYPE_AMBIENT_TEMPERATURE){
      Serial.print("Temperature: ");
      Serial.println(event.temperature);
    }
    else if(event.type == SENSOR_TYPE_COLOR){
      Serial.print("Color R: ");
      Serial.println(event.color.r);
      Serial.print("Color G: ");
      Serial.println(event.color.g);
      Serial.print("Color B: ");
      Serial.println(event.color.b);
    }
    else if(event.type == SENSOR_TYPE_PROXIMITY){
      Serial.print("Distance: ");
      Serial.println(event.distance);
    }
    else{
      Serial.println("Unknown sensor type");
    }

  }

}

void loop(){



}