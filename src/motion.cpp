#include <var.h>
#include <Arduino.h>
#include <math.h>

extern float latestSerialHeading;   // heading coming from main.cpp

static float targetHeading = 0;

static char lastCommand = '0';


// PID gains
const float Kp = 0.2;
const float Ki = 0.02; //if robot starts to oscillate, reducing this value (if still happens reduce kp and kd too)
const float Kd = 0.12;

// PID state
static float integral = 0;
static float prevError = 0;

// control timestep (seconds)
const float dt = 0.1;

// Debug variables
float debug_error = 0;
float debug_targetHeading = 0;
float debug_serialHeading = 0;
float debug_correction = 0;

void motion(char _data) {
  if(_data == '0') { 
    rpmAlter = false;
    rpmAlter_T = false;
    //Serial5.write(0);
    //Serial5.write(128);

    integral = 0;
    prevError = 0;

    debug_error = 0;
    debug_correction = 0;
    debug_targetHeading = latestSerialHeading;
    debug_serialHeading = latestSerialHeading;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
  }  
  //Forward cmd
else if (_data == '1') {
    rpmAlter_T = false;

    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);
    
    if(_data == '1' && lastCommand != '1') {
        targetHeading = latestSerialHeading;
        integral = 0;
        prevError = 0;
    }

    float error = targetHeading - latestSerialHeading;
    
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    const float headingDeadband = 0.05f;
    if (fabsf(error) < headingDeadband) {
      error = 0.0f;
    }


    // ---- PID ----
    integral += error * dt;

    // Anti-windup protection
    integral = constrain(integral, -20, 20);

    float derivative = (error - prevError) / dt;

    float correction =
          Kp * error
        + Ki * integral
        + Kd * derivative;

    prevError = error;

    // limit steering strength
    correction = constrain(correction, -45, 45);

    // store debug values
    debug_error = error;
    debug_targetHeading = targetHeading;
    debug_serialHeading = latestSerialHeading;
    debug_correction = correction;

    const float minStepErrorThreshold = 0.35f;
    if (fabsf(correction) < 1.0f && fabsf(error) >= minStepErrorThreshold) {
      correction = (correction >= 0.0f) ? 1.0f : -1.0f;
    }

    int baseRight = 237;
    int baseLeft  = 242;

    int steeringStep = (int)roundf(correction);
    int pwmR = baseRight - steeringStep;
    int pwmL = baseLeft  + steeringStep;

    pwmR = constrain(pwmR, 0, 255);
    pwmL = constrain(pwmL, 0, 255);

    analogWrite(pwmPin_R, pwmR);
    analogWrite(pwmPin_L, pwmL);
}
  else if (_data == '2') {
      rpmAlter_T = false;

      digitalWrite(dirPin_R, HIGH);
      digitalWrite(dirPin_L, HIGH);

      if(_data == '2' && lastCommand != '2') {
          targetHeading = latestSerialHeading;
          integral = 0;
          prevError = 0;
      }
      float error = targetHeading - latestSerialHeading;
      
      while (error > 180) error -= 360;
      while (error < -180) error += 360;

      const float headingDeadband = 0.05f;
      if (fabsf(error) < headingDeadband) {
        error = 0.0f;
      }

      // ---- PID ----
      integral += error * dt;
      integral = constrain(integral, -20, 20);

      float derivative = (error - prevError) / dt;

      float correction =
            Kp * error
          + Ki * integral
          + Kd * derivative;

      prevError = error;

      correction = constrain(correction, -40, 40);

      const float minStepErrorThreshold = 0.35f;
      if (fabsf(correction) < 1.0f && fabsf(error) >= minStepErrorThreshold) {
        correction = (correction >= 0.0f) ? 1.0f : -1.0f;
      }

      int baseRight = 250;
      int baseLeft  = 242;

      // reverse steering for backward motion
      int steeringStep = (int)roundf(correction);
      int pwmR = baseRight - steeringStep;
      int pwmL = baseLeft  + steeringStep;

      pwmR = constrain(pwmR, 0, 255);
      pwmL = constrain(pwmL, 0, 255);

      analogWrite(pwmPin_R, pwmR);
      analogWrite(pwmPin_L, pwmL);

      // store debug values
      debug_error = error;
      debug_targetHeading = targetHeading;
      debug_serialHeading = latestSerialHeading;
      debug_correction = correction;
  }
  else if (_data == '3') {
    rpmAlter = false;
    //Serial5.write(rpmAlter_T == 0 ?TRR:FRD);
    //Serial5.write(rpmAlter_T == 0 ?TRL:BLW);
    integral = 0;
    prevError = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 82 : 148);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80: 146);
  } else if (_data == '4') {
    rpmAlter = false;
    //Serial5.write(rpmAlter_T == 0 ?TLR:BRW);
    //Serial5.write(rpmAlter_T == 0 ?TLL:FLD);
    integral = 0;
    prevError = 0;
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

