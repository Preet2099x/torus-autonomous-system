#include <var.h>
#include <Arduino.h>
#include <math.h>

extern float latestSerialHeading;   // heading coming from main.cpp

static float targetHeading = 0;

static char lastCommand = '0';

static int currentPWM_L = 0;
static int currentPWM_R = 0;
static int currentDir_L = LOW;
static int currentDir_R = LOW;

static uint32_t lastRampUpdateL = 0;
static uint32_t lastRampUpdateR = 0;

const int START_PWM = 10;      // enough to overcome motor deadzone
const int RAMP_STEP = 2;       // PWM change per ramp interval
const uint32_t RAMP_INTERVAL_MS = 8;

static int rampPWM(int currentPwm, int targetPwm, uint32_t &lastUpdateMs) {
  uint32_t nowMs = millis();
  if ((nowMs - lastUpdateMs) < RAMP_INTERVAL_MS) {
    return constrain(currentPwm, 0, 255);
  }
  lastUpdateMs = nowMs;

  if (currentPwm == 0 && targetPwm > 0) {
    currentPwm = min(START_PWM, targetPwm);
  }

  if (currentPwm < targetPwm) {
    currentPwm += RAMP_STEP;
    if (currentPwm > targetPwm) currentPwm = targetPwm;
  } else if (currentPwm > targetPwm) {
    currentPwm -= RAMP_STEP;
    if (currentPwm < targetPwm) currentPwm = targetPwm;
  }

  return constrain(currentPwm, 0, 255);
}

static void writeRampedMotor(int dirL, int dirR, int targetPwmL, int targetPwmR) {
  targetPwmL = constrain(targetPwmL, 0, 255);
  targetPwmR = constrain(targetPwmR, 0, 255);

  if (dirL != currentDir_L && currentPWM_L > 0) {
    targetPwmL = 0;
  } else if (currentPWM_L == 0 && dirL != currentDir_L) {
    currentDir_L = dirL;
    digitalWrite(dirPin_L, currentDir_L);
  }

  if (dirR != currentDir_R && currentPWM_R > 0) {
    targetPwmR = 0;
  } else if (currentPWM_R == 0 && dirR != currentDir_R) {
    currentDir_R = dirR;
    digitalWrite(dirPin_R, currentDir_R);
  }

  digitalWrite(dirPin_L, currentDir_L);
  digitalWrite(dirPin_R, currentDir_R);

  currentPWM_L = rampPWM(currentPWM_L, targetPwmL, lastRampUpdateL);
  currentPWM_R = rampPWM(currentPWM_R, targetPwmR, lastRampUpdateR);

  analogWrite(pwmPin_L, currentPWM_L);
  analogWrite(pwmPin_R, currentPWM_R);
}


// PID gains

// forward
const float Kp_fwd = 0.04f;
const float Ki_fwd = 0.0f;
const float Kd_fwd = 0.015f;

// backward
const float Kp_rev = 0.07f;
const float Ki_rev = 0.0f;
const float Kd_rev = 0.015f;

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

    currentPWM_L = 0;
    currentPWM_R = 0;

    currentDir_L = LOW;
    currentDir_R = LOW;
    lastRampUpdateL = millis();
    lastRampUpdateR = lastRampUpdateL;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
  }  
  //Forward cmd
else if (_data == '1') {
    rpmAlter_T = false;
    
    if(_data == '1' && lastCommand != '1') {
        targetHeading = latestSerialHeading;
        integral = 0;
        prevError = 0;
    }

    float error = targetHeading - latestSerialHeading;
    
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    const float headingDeadband = 1.0f;
    if (fabsf(error) < headingDeadband) {
      error = 0.0f;
    }


    // ---- PID ----
    integral += error * dt;

    // Anti-windup protection
    integral = constrain(integral, -20, 20);

    float derivative = (error - prevError) / dt;

    float correction =
          Kp_fwd * error
        + Ki_fwd * integral
        + Kd_fwd * derivative;

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

    int steeringStep = (int)(correction * 2.0f);
    int pwmR = baseRight - steeringStep;
    int pwmL = baseLeft  + steeringStep;

    pwmR = constrain(pwmR, 0, 255);
    pwmL = constrain(pwmL, 0, 255);

    writeRampedMotor(LOW, LOW, pwmL, pwmR);
  }
  else if (_data == '2') {
      rpmAlter_T = false;

      if(_data == '2' && lastCommand != '2') {
          targetHeading = latestSerialHeading;
          integral = 0;
          prevError = 0;
      }
      float error = targetHeading - latestSerialHeading;
      
      while (error > 180) error -= 360;
      while (error < -180) error += 360;

      const float headingDeadband = 1.0f;
      if (fabsf(error) < headingDeadband) {
        error = 0.0f;
      }

      // ---- PID ----
      integral += error * dt;
      integral = constrain(integral, -20, 20);

      float derivative = (error - prevError) / dt;

      float correction =
            Kp_rev * error
          + Ki_rev * integral
          + Kd_rev * derivative;

      prevError = error;

      correction = constrain(correction, -40, 40);

      // Avoid quantized lock at ±1.0 by adding smooth error-proportional authority.
      correction += (0.45f * error);
      correction = constrain(correction, -55, 55);

      int baseRight = 247;
      int baseLeft  = 240;

      // reverse steering for backward motion
      int steeringStep = (int)roundf(correction * 2.5f);
      int pwmR = baseRight - steeringStep;
      int pwmL = baseLeft  + steeringStep;

      pwmR = constrain(pwmR, 0, 255);
      pwmL = constrain(pwmL, 0, 255);

      writeRampedMotor(HIGH, HIGH, pwmL, pwmR);

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

    writeRampedMotor(
      LOW,
      HIGH,
      rpmAlter_T == 0 ? 82 : 148,
      rpmAlter_T == 0 ? 80 : 146
    );
  } else if (_data == '4') {
    rpmAlter = false;
    //Serial5.write(rpmAlter_T == 0 ?TLR:BRW);
    //Serial5.write(rpmAlter_T == 0 ?TLL:FLD);
    integral = 0;
    prevError = 0;
    writeRampedMotor(
      HIGH,
      LOW,
      rpmAlter_T == 0 ? 83 : 150,
      rpmAlter_T == 0 ? 80 : 147
    );
  }  else if(_data == '5') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? FRW: FRD); //TODO: TO BE changed 
    //Serial5.write(TLL);
    writeRampedMotor(
      LOW,
      LOW,
      rpmAlter == 0 ? 204 : 245,
      150
    );
  } else if(_data == '6') {
    rpmAlter_T = false;
    //Serial5.write(TRR);
    //Serial5.write(rpmAlter == 0 ? FLW : FLD);
    writeRampedMotor(
      LOW,
      LOW,
      150,
      rpmAlter == 0 ? 215 : 250
    );
  } else if(_data == '7') {
    rpmAlter_T = false;
    //Serial5.write(TLR);
    //Serial5.write(rpmAlter == 0 ? BLW: BLD);
    writeRampedMotor(
      HIGH,
      HIGH,
      rpmAlter == 0 ? 209 : 251,
      150
    );
  } else if(_data == '8') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? BRW : BRD);
    //Serial5.write(TRL);
    writeRampedMotor(
      HIGH,
      digitalRead(dirPin_R),
      150,
      rpmAlter == 0 ? 202 : 245
    );
  } else {} 
  lastCommand = _data;
}

