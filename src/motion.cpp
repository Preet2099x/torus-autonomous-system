#include <var.h>
#include <Arduino.h>

extern float latestSerialHeading;   // heading coming from main.cpp

static float targetHeading = 0;

static char lastCommand = '0';

// PID gains
const float Kp = 0.8;
const float Ki = 0.02; //if robot starts to oscillate, reducing this value (if still happens reduce kp and kd too)
const float Kd = 0.4;

// PID state
static float integral = 0;
static float prevError = 0;

// control timestep (seconds)
const float dt = 0.1;

void motion(char _data) {
  if(_data == '0') { 
    rpmAlter = false;
    rpmAlter_T = false;
    //Serial5.write(0);
    //Serial5.write(128);

    integral = 0;
    prevError = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
  }  
else if (_data == '1') {
    rpmAlter_T = false;

    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);

    if(_data == '1' && lastCommand != '1') {
        targetHeading = latestSerialHeading;
    }

    float error = targetHeading - latestSerialHeading;

    if(error > 180) error -= 360;
    if(error < -180) error += 360;

    if(abs(error) < 1.0) error = 0;


    // ---- PID ----
    integral += error * dt;

    // Anti-windup protection
    integral = constrain(integral, -50, 50);

    float derivative = (error - prevError) / dt;

    float correction =
          Kp * error
        + Ki * integral
        + Kd * derivative;

    prevError = error;

    // limit steering strength
    correction = constrain(correction, -20, 20);

    int baseRight = 246;
    int baseLeft  = 250;

    int pwmR = baseRight + correction;
    int pwmL = baseLeft  - correction;

    pwmR = constrain(pwmR, 0, 255);
    pwmL = constrain(pwmL, 0, 255);

    analogWrite(pwmPin_R, pwmR);
    analogWrite(pwmPin_L, pwmL);
}
  else if(_data == '2') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? BRW : BRD); //Value RIGHT: 64(Stop) - 127(MAX) CCW
    //Serial5.write(rpmAlter == 0 ? BLW: BLD);  //Value LEFT: 192(Stop) - 255(MAX) CCW
    
    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);
    // analogWrite(pwmPin_R, rpmAlter == 0 ? 202 : 240);
    // analogWrite(pwmPin_L, rpmAlter == 0 ? 211 : 249); 
    analogWrite(pwmPin_R, 250);
    analogWrite(pwmPin_L, 242);
  } else if (_data == '3') {
    rpmAlter = false;
    //Serial5.write(rpmAlter_T == 0 ?TRR:FRD);
    //Serial5.write(rpmAlter_T == 0 ?TRL:BLW);

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 82 : 148);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80: 146);
  } else if (_data == '4') {
    rpmAlter = false;
    //Serial5.write(rpmAlter_T == 0 ?TLR:BRW);
    //Serial5.write(rpmAlter_T == 0 ?TLL:FLD);
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 83 :150);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80 :147);
  }  else if(_data == '5') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? FRW: FRD); //TODO: TO BE changed 
    //Serial5.write(TLL);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 204:245);
    analogWrite(pwmPin_R, 150);  
  } else if(_data == '6') {
    rpmAlter_T = false;
    //Serial5.write(TRR);
    //Serial5.write(rpmAlter == 0 ? FLW : FLD);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 215 :250);
  } else if(_data == '7') {
    rpmAlter_T = false;
    //Serial5.write(TLR);
    //Serial5.write(rpmAlter == 0 ? BLW: BLD);
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 209 : 251);
    analogWrite(pwmPin_R, 150); 
  } else if(_data == '8') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? BRW : BRD);
    //Serial5.write(TRL);
    digitalWrite(dirPin_L, HIGH);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 202 :245);
  } else {} 
  lastCommand = _data;
}

