#include <var.h>
#include <maf.h>
#include <string.h>
#include <stdlib.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <structure.h>
#include <i2c_driver.h>
#include <TeensyThreads.h>
#include <i2c_driver_wire.h>
#include <Wire.h>

int SLAVE_ADDRESS = 0x72;

uint8_t BNO_ADDR =       0x28;  // I2C address of BNO
uint8_t ACC_DATA_X_LSB = 0x08;  // BNO register Acceleration Data X LSB
uint8_t CALIB_STAT =     0x35;  // BNO register SYS Calib Status <7:6>, GYR Calib Status <5:4>, ACC Calib Status <3:2>, MAG Calib Status <1:0>
uint8_t OPR_MODE   =     0x3D; // BNO register Operation Mode <3:0>
uint8_t NDOF       =     0x0C;

imuData imu;

bool ifone = false;
int FLW = 0;
int FRW = 0;
int BLW = 0;
int BRW = 0;

int FLD = 0;
int FRD = 0;
int BLD = 0;
int BRD = 0;

int TRR = 0;
int TRL = 0;
int TLR = 0;
int TLL = 0;

int pwmPin_R = 14; //22
int dirPin_R = 20; //22
int pwmPin_L = 23; //11
int dirPin_L = 22; //11

extern float debug_error;
extern float debug_targetHeading;
extern float debug_serialHeading;
extern float debug_correction;


unsigned int addressFLW =  0;
unsigned int addressFRW =  2;
unsigned int addressBLW =  4;
unsigned int addressBRW =  6;

unsigned int addressFLD =  8;
unsigned int addressFRD = 10;
unsigned int addressBLD = 12;
unsigned int addressBRD = 14;

unsigned int addressTRR = 16;
unsigned int addressTRL = 18;
unsigned int addressTLR = 20;
unsigned int addressTLL = 22;

int encoderPin_1_L = 7;
int encoderPin_2_L = 8;

int encoderPin_1_R = 5;
int encoderPin_2_R = 4;

volatile int lastEncoded_L = 0;
volatile long encoderValue_L = 0;
long lastencoderValue_L = 0;
int lastMSB_L = 0;
int lastLSB_L = 0;

volatile int lastEncoded_R = 0;
volatile long encoderValue_R = 0;
long lastencoderValue_R = 0;
int lastMSB_R = 0;
int lastLSB_R = 0;

//DIR and PWM Data
int    delayBrake = 50;
int pin_Emergency = 15;

//Handle Print Vsriable
bool     printAlter = false;
bool emergencyAlter = true;
bool  systemCounter = false;

//Control Variable
char  data = '0';
int rpmAlter_T = 0;
int rpmAlter = 0;
int _dirData = 0;

//Time Variable
float timeConstant = 100; 
float startTime, elaspedTime = 0, currentTime;
float rpmScale_L = 1.93534;
float rpmScale_R = 1.94640;
float latestSerialHeading = 0.0f;
static float lastValidHeading = 0.0f;
static bool hasValidHeading = false;

static const uint8_t MAIN_BNO055_ADDR = 0x28;

static int16_t mainBnoRead16(uint8_t reg) {
  Wire1.beginTransmission(MAIN_BNO055_ADDR);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(MAIN_BNO055_ADDR, (uint8_t)2);

  if (Wire1.available() < 2) {
    return 0;
  }

  uint8_t lsb = Wire1.read();
  uint8_t msb = Wire1.read();
  return (int16_t)(lsb | (msb << 8));
}

static void mainBnoWrite8(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MAIN_BNO055_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
}

static void mainBnoInit() {
  Wire1.begin();
  delay(700);

  mainBnoWrite8(0x3D, 0x00);
  delay(20);

  mainBnoWrite8(0x3E, 0x00);
  delay(10);

  mainBnoWrite8(0x07, 0x00);
  mainBnoWrite8(0x3D, 0x0C);
  delay(20);

  Serial.println("BNO055 ready");
}

static void updateLatestHeadingFromBno() {
  float heading = mainBnoRead16(0x1A) / 16.0f;

  while (heading < 0.0f) heading += 360.0f;
  while (heading >= 360.0f) heading -= 360.0f;

  latestSerialHeading = heading;
}

//Time Setup Control Counter
float timeConstantControlCounter = 1000; 
float startTimeControlCounter, elaspedTimeControlCounter = 0, currentTimeControlCounter;

void setup() {
  //Serial Begin
  Serial.begin(115200);
  mainBnoInit();
  //Serial5.begin(115200);//TODO: For Simplified Serial
  kire.begin(SLAVE_ADDRESS); 
  //initWire();
  
   //Motor Pin Setup
   pinMode(dirPin_L, OUTPUT);
   pinMode(pwmPin_L, OUTPUT);
   
   pinMode(dirPin_R, OUTPUT);
   pinMode(pwmPin_R, OUTPUT);

  //initializeBNO055();
  pinMode(pin_Emergency, INPUT_PULLDOWN);

  //Time Setup
  startTime = millis();

  //Time Setup Control Counter
  startTimeControlCounter = millis();


  digitalWrite(dirPin_L, LOW);
  digitalWrite(dirPin_R, LOW);
  digitalWrite(pwmPin_L, LOW);
  digitalWrite(pwmPin_R, LOW);

  //Encoder Setup
  pinMode(encoderPin_1_L, INPUT); 
  pinMode(encoderPin_2_L, INPUT);

  digitalWrite(encoderPin_1_L, HIGH); 
  digitalWrite(encoderPin_2_L, HIGH); 
  attachInterrupt(encoderPin_1_L, updateEncoder_L, CHANGE); 
  attachInterrupt(encoderPin_2_L, updateEncoder_L, CHANGE);

  pinMode(encoderPin_1_R, INPUT); 
  pinMode(encoderPin_2_R, INPUT);

  digitalWrite(encoderPin_1_R, HIGH); 
  digitalWrite(encoderPin_2_R, HIGH); 
  attachInterrupt(encoderPin_1_R, updateEncoder_R, CHANGE); 
  attachInterrupt(encoderPin_2_R, updateEncoder_R, CHANGE);

  //Read Permanent Data 
  readEEPROM();

  Serial5.write(0);
  Serial5.write(192);
  /*
  kire.setClock(400 * 1000);
  kire.begin();                // initialize I2C
  bno_write(BNO_ADDR, OPR_MODE, NDOF);
  delay(1000);

  threads.addThread(thread_func);
  */
}

void loop() { 

  kire.onReceive(receiveEvent);

  // Ignore heading packets like H:120.8 so embedded digits are not treated as drive commands.
  static bool sawHeadingPrefix = false;
  static bool parsingHeadingPacket = false;
  static char headingPacket[20];
  static int headingPacketIndex = 0;

  //Handle Fliter
  static SMA<20, double, double> filter_L;
  double rpm_L = 0;
  double avgRPM_L = 0;

  static SMA<20, double, double> filter_R;
  double rpm_R = 0;
  double avgRPM_R = 0;

  int emergency = analogRead(pin_Emergency);//TODO: Comment if emergency is removed 
  
  //Handle  Time
  currentTime = millis(); 
  elaspedTime = currentTime - startTime;

  //Time Setup Control Counter
  currentTimeControlCounter = millis(); 
  elaspedTimeControlCounter = currentTimeControlCounter - startTimeControlCounter;

  
  //Handle Serial Commands
  if(Serial.available()) {
    if(systemCounter == false) {
      while (Serial.available()) {
        int _data = Serial.read();

        if (parsingHeadingPacket) {
          bool isHeadingNumberChar = (_data >= '0' && _data <= '9') || _data == '.' || _data == '-' || _data == '+';

          if (_data == '\n' || _data == '\r' || !isHeadingNumberChar) {
            if (headingPacketIndex > 0) {
              headingPacket[headingPacketIndex] = '\0';
              float newHeading = atof(headingPacket);
              // normalize
              while (newHeading < 0) newHeading += 360;
              while (newHeading >= 360) newHeading -= 360;

              if (!hasValidHeading) {
                lastValidHeading = newHeading;
                hasValidHeading = true;
              } else {
                // reject impossible jumps while allowing normal heading changes
                float diff = newHeading - lastValidHeading;
                if (diff > 180) diff -= 360;
                if (diff < -180) diff += 360;

                // guard-rail filter (was too strict at 10 deg and froze updates)
                if (fabsf(diff) <= 45.0f) {
                  lastValidHeading = newHeading;
                }
              }
            }
            headingPacketIndex = 0;
            parsingHeadingPacket = false;
            continue;
          }

          if (isHeadingNumberChar) {
            if (headingPacketIndex < (int)sizeof(headingPacket) - 1) {
              headingPacket[headingPacketIndex++] = (char)_data;
            }
            continue;
          }
        }

        if (sawHeadingPrefix) {
          if (_data == ':') {
            parsingHeadingPacket = true;
            headingPacketIndex = 0;
            sawHeadingPrefix = false;
            continue;
          }
          sawHeadingPrefix = false;
        }

        if (_data == 'H' || _data == 'h') {
          sawHeadingPrefix = true;
          continue;
        }

        if(_data == char('m')) {
          Serial.print(getTeensySerial());
          Serial.print(" | ");
          Serial.println("Motion Module");
          data = char('0');
        } else if(_data == char('s')) {
          systemCounter = true;
          printAlter = false; //TODO: Can be Removed for fast testing
          data = '0';//TODO: Can be Removed for fast testing
          printSetting();
        } else if(_data == char('p')) {
          printAlter =  !printAlter;
        } else if(_data == char('a')) {
          if (data != '3' || data != '4' || data != '0') {
              rpmAlter = !rpmAlter; 
        } } else if(_data == char('b')) {
          if (data != '1' || data != '2' || data != '0') {
              rpmAlter_T = !rpmAlter_T; 
        } } else if(_data != 10) {
          data = _data;
          if(_data == data && elaspedTimeControlCounter < timeConstantControlCounter) {
            startTimeControlCounter = currentTimeControlCounter;
          } else {
            data = _data;
            startTimeControlCounter = currentTimeControlCounter;
          }
        }
      }
    } else {
      String _data = Serial.readString();
      _data = _data.remove(_data.length()-1, 1);
      if(_data.length() == 1 && _data == "s") {
        Serial.println("Chaging to Control Mode");
        systemCounter = false;
      } else if(_data.length() >= 1) {
        updatedEEPROM(_data);
      }
    } 
  }

  if(elaspedTimeControlCounter > timeConstantControlCounter) {
     //data = '0'; //Commenting for Testing //TODO:
    startTimeControlCounter = currentTimeControlCounter;
  }
  
  if(emergency > 900) { 
    rpmAlter = false;
    //Serial5.write(0);
    //Serial5.write(128);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
    data = '0';
  } else if (emergency < 900) {
    updateLatestHeadingFromBno();
    motion(data);
  } 
  
  if (elaspedTime > timeConstant) {
    startTime = currentTime;

    //RPM Calculation
    rpm_L = ((abs(encoderValue_L)* 60 * handletime(timeConstant)) / 4000.00) * rpmScale_L / 1.6; 
    rpm_R = ((abs(encoderValue_R)* 60 * handletime(timeConstant)) / 4000.00) * rpmScale_R / 1.6;
    //rpm = (No of Pluses/Total Pules) * 1sec 
    //Total Pulse = Pulse * 4 where is changes in both the phases
    //SEC -> MilliSec 60*100 -> TimeConstant will be 100
    //SEC -> MilliSec 60*1000 -> TimeConstant will be 10
    
    avgRPM_L = filter_L(rpm_L);
    avgRPM_R = filter_R(rpm_R);


    if(printAlter == true) {  
      // leftRPM | rightRPM | heading | target | error | correction
      //Serial.print(rpmAlter);
      //Serial.print(" | ");
      //Serial.print(rpmAlter_T);
      //Serial.print(" | ");
      Serial.printf(
      "%c | %6.2f | %6.2f | H:%6.1f | T:%6.1f | E:%6.2f | C:%7.3f\n",
      data,
      avgRPM_L,
      avgRPM_R,
      latestSerialHeading,
      debug_targetHeading,
      debug_error,
      debug_correction
      );
      /*Serial.print(" | ");
      Serial.print((s.calib_stat >> 6) & 3);
      Serial.print(" | ");
      Serial.print((s.calib_stat >> 4) & 3);
      Serial.print(" | ");
      Serial.print((s.calib_stat >> 2) & 3);
      Serial.print(" | ");
      Serial.println((s.calib_stat >> 0) & 3); */
   }

   encoderValue_L = encoderValue_R = 0;

  }

}
