#include "Wire.h"    
#include "TFLI2C.h"  

TFLI2C sensor;

void setup() {
  Serial.begin(9600);  
  Wire.begin(); 
  Serial.println("Initialised");
  /*delay(5000); 
  sensor.Set_I2C_Addr(0x20,0x10);
  Serial.println("Set to 20");
  sensor.Save_Settings(0x20);
  delay(10000);
  sensor.Set_I2C_Addr(0x30,0x10);
  Serial.println("Set to 30");
  sensor.Save_Settings(0x30);
  delay(10000);*/ 
}

void loop() {
  int16_t dist_front;
  int16_t dist_right; 
  int16_t dist_left;
  Serial.print(" front:");
  if (sensor.getData(dist_front, 0x20)) {
    Serial.print(dist_front);
  }
  Serial.print(" right:");
  if (sensor.getData(dist_right, 0x10)) {
    Serial.print(dist_right);
  }
  Serial.print(" left:");
  if (sensor.getData(dist_left, 0x30)) {
    Serial.println(dist_left);
  }
  delay(100);
}
