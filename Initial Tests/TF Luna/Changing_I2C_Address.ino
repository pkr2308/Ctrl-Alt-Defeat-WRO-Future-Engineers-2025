#include "Wire.h"    
#include "TFLI2C.h"  

TFLI2C sensor;

void setup() {
  Serial.begin(9600);  
  Wire.begin(); 
  sensor.Set_I2C_Addr(0x30,0x10);
}

void loop() {
  int16_t dist; 
  if (sensor.getData(dist, 0x30)) {
    Serial.print("dist:");
    Serial.println(dist);
  }
  delay(100);
}