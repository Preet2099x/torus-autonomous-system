#include <var.h>
#include <Arduino.h>


void updateEncoder_L() {
  int MSB = digitalRead(encoderPin_1_L); //MSB = most significant bit
  int LSB = digitalRead(encoderPin_2_L); //LSB = least significant bit

  // ── Correct gray-code counting (for distance) ──
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded_L << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_L ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_L --;

  // ── Legacy counting (for RPM — uses old formula with stale LSB) ──
  int encodedLegacy = (MSB << 1) | lastLSB_L;
  int sumLegacy = (lastEncoded_L << 2) | encodedLegacy;

  if(sumLegacy == 0b1101 || sumLegacy == 0b0100 || sumLegacy == 0b0010 || sumLegacy == 0b1011) encoderValueLegacy_L ++;
  if(sumLegacy == 0b1110 || sumLegacy == 0b0111 || sumLegacy == 0b0001 || sumLegacy == 0b1000) encoderValueLegacy_L --;

  lastEncoded_L = encoded; //store this value for next time
  lastLSB_L = LSB;
}

void updateEncoder_R() {
  int MSB = digitalRead(encoderPin_1_R); //MSB = most significant bit
  int LSB = digitalRead(encoderPin_2_R); //LSB = least significant bit

  // ── Correct gray-code counting (for distance) ──
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded_R << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_R ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_R --;

  // ── Legacy counting (for RPM — uses old formula with stale LSB) ──
  int encodedLegacy = (MSB << 1) | lastLSB_R;
  int sumLegacy = (lastEncoded_R << 2) | encodedLegacy;

  if(sumLegacy == 0b1101 || sumLegacy == 0b0100 || sumLegacy == 0b0010 || sumLegacy == 0b1011) encoderValueLegacy_R ++;
  if(sumLegacy == 0b1110 || sumLegacy == 0b0111 || sumLegacy == 0b0001 || sumLegacy == 0b1000) encoderValueLegacy_R --;

  lastEncoded_R = encoded; //store this value for next time
  lastLSB_R = LSB;
}