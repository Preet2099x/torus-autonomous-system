#include <var.h>
#include <Arduino.h>
#include <math.h>

extern float control_left_rpm;
extern float control_right_rpm;
extern float control_heading;
extern float target_heading;
extern bool heading_initialized;

void motion(char _data) {
  static int current_pwm_L = 0;
  static int current_pwm_R = 0;
  static float filtered_heading = 0;

  if(_data == '0') { //TODO Implemtation
    rpmAlter = false;
    rpmAlter_T = false;
    heading_initialized = false;
    filtered_heading = 0;
    current_pwm_L = 0;
    current_pwm_R = 0;
    //Serial5.write(0);
    //Serial5.write(128);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
  }  else if (_data == '1') {
    rpmAlter_T = false;

    if(heading_initialized == false) {
      target_heading = control_heading;
      heading_initialized = true;
    }

    const float base_left_pwm = 250.0f;
    const float base_right_pwm = 246.0f;
    const float Ks = 0.3f;
    const float Kh = 1.0f;
    const float alpha = 0.15f;

    float speed_error = control_left_rpm - control_right_rpm;
    float speed_correction = Ks * speed_error;

    float heading_error = target_heading - control_heading;
    if(heading_error > 180.0f) {
      heading_error -= 360.0f;
    }
    if(heading_error < -180.0f) {
      heading_error += 360.0f;
    }

    if(fabsf(heading_error) < 1.0f) {
      heading_error = 0.0f;
    }

    float heading_correction = Kh * heading_error;
    filtered_heading = filtered_heading + alpha * (heading_correction - filtered_heading);

    float raw_target_pwm_L = base_left_pwm - filtered_heading - speed_correction;
    float raw_target_pwm_R = base_right_pwm + filtered_heading + speed_correction;

    int target_pwm_L = (int)raw_target_pwm_L;
    int target_pwm_R = (int)raw_target_pwm_R;

    if(target_pwm_L < 0) {
      target_pwm_L = 0;
    } else if(target_pwm_L > 255) {
      target_pwm_L = 255;
    }

    if(target_pwm_R < 0) {
      target_pwm_R = 0;
    } else if(target_pwm_R > 255) {
      target_pwm_R = 255;
    }

    const int step = 3;
    if(current_pwm_L < target_pwm_L) {
      current_pwm_L += step;
      if(current_pwm_L > target_pwm_L) {
        current_pwm_L = target_pwm_L;
      }
    } else if(current_pwm_L > target_pwm_L) {
      current_pwm_L -= step;
      if(current_pwm_L < target_pwm_L) {
        current_pwm_L = target_pwm_L;
      }
    }

    if(current_pwm_R < target_pwm_R) {
      current_pwm_R += step;
      if(current_pwm_R > target_pwm_R) {
        current_pwm_R = target_pwm_R;
      }
    } else if(current_pwm_R > target_pwm_R) {
      current_pwm_R -= step;
      if(current_pwm_R < target_pwm_R) {
        current_pwm_R = target_pwm_R;
      }
    }

    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);
    analogWrite(pwmPin_L, current_pwm_L);
    analogWrite(pwmPin_R, current_pwm_R);
  } else if(_data == '2') {
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
}

