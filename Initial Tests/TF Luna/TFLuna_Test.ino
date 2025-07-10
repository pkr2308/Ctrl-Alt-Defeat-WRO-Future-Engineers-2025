#include "Wire.h"    
#include "TFLI2C.h"  

TFLI2C sensor;

void setup() {
  Serial.begin(9600);  
  Wire.begin();
  Serial.println("Serial initialised");          
}

void loop() {
  int16_t dist; 
  if (sensor.getData(dist, 0x10)) {
    Serial.print("dist:");
    Serial.println(dist);
  }
  delay(100);
}
