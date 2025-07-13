#include "Arduino.h"

unsigned long lastLoopTime = 0;
unsigned long currentLoopTime = 0;
const unsigned long loopInterval = 50;

const double PID_P = 0;
const double PID_I = 0;
const double PID_D = 0;

double yawError = 0;
double yawTarget = 0;
double pidOutput = 0;
double currentYaw = 0;

const double MAX_PID_OUTPUT = 200;
const double MAX_STEERING_DEFLECTION = 45;
const double STEERING_CENTER = 90;


double getShortestAngleError(double target, double current);
void commandSteering(double pidCommand);

void setup(){

  // init BNO055
  // init PID

}

void loop(){

  currentLoopTime = millis();

  if(currentLoopTime - lastLoopTime >= loopInterval) {
    lastLoopTime = currentLoopTime;

    // Read yaw from BNO055

    yawError = getShortestAngleError(yawTarget, currentYaw);
    
    // Calculate pidOutput

    commandSteering(pidOutput);

  }

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

  double adjustment = (pidCommand / MAX_PID_OUTPUT) * MAX_STEERING_DEFLECTION;

  int servoAngle = STEERING_CENTER + adjustment;
  servoAngle = constrain(servoAngle, 0, 180);


}