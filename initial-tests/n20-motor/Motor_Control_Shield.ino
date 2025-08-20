int E1 = 10;
int M1 = 12;

void setup(){
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
}

void loop(){
  digitalWrite(M1,HIGH);
  analogWrite(E1, 1023);
  delay(1000);

  digitalWrite(M1,LOW);
  analogWrite(E1, 0);
  delay(1000);

  digitalWrite(M1,LOW);
  analogWrite(E1, 1023);
  delay(1000);
}