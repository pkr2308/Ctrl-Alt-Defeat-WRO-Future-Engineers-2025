// This idea did not work
// The wifi was too big a drain on the Arduino, so the round was not being well executed

#include <Arduino.h>

#include <Wire.h>
#include <WiFiS3.h>

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
int turns = 0;
bool turning = false;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 200;

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

char ssid[] = "";        // your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

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

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");

  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();
  printWifiStatus();

  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  bno.setExtCrystalUse(true);
  lidar.getData(startDist, 0x10);
  
  delay(100);
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

  lidar.getData(lidarDist, 0x20); 
  if ((lidarDist < threshold) && (turning == false) && (turns == 0 or distance > 150)){
    steer(90);
    turning = true;
  }
  
  if (turns == 12 && lidarDist <= startDist && distance > 10){ 
    stop();
    while (true){}
    //Serial.print("Finished");
  }

  pStraight();
  wifiData();

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

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void wifiData() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<p style=\"font-size:7vw;\"> Lidar - ");
            client.print(lidarDist);
            client.print(" cm </p>");
            
            client.print("<p style=\"font-size:7vw;\"> Yaw: ");
            client.print(yaw);
            client.print("°</p>");

            client.print("<p style=\"font-size:7vw;\"> Target Yaw: ");
            client.print(targetYaw);
            client.print("°</p>");

            client.print("<p style=\"font-size:7vw;\"> Distance: ");
            client.print(distance);
            client.print(" cm</p>");

            client.print("<p style=\"font-size:7vw;\"> Turns: ");
            client.print(turns);
            client.print("</p>");
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
      
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}
