#include <var.h>
#include <string.h>
#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

float calcAlphaEMA(float fn) {
    if (fn <= 0)
        return 1;
    // α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
    const float c = cos(2 * float(M_PI) * fn);
    return c - 1 + sqrt(c * c - 4 * c + 3);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double handletime(float data) {
    if(data == 100) {
        return 10;
    } else if(data == 10) {
        return 100;
    } else {
        return 0;
    }
}

uint32_t getTeensySerial() {
	uint32_t num;
	num = HW_OCOTP_MAC0 & 0xFFFFFF;
	if (num < 10000000) {  
      num = num * 10;
      return num;
	}  else { 
      return num;
  }
}

//void(* resetFunc) (void) = 0;
void receiveEvent(int bytesReceived) {
 String receivedData = "";
    while(kire.available()) {
        char c = kire.read();
        receivedData += c;
        Serial.println(receivedData);
    }
    Serial.println(data);
int val = mapStringToInt(receivedData);
 if(val > 0){turn = true;}
 else if(val ==0)(turn = false);
    if(receivedData == "a") {
        if (data != 3 || data != 4 || data != 0) {
            rpmAlter = !rpmAlter; 
      } } else if(receivedData == 'b') {
        if (data != 2 || data != 1 || data != 0) {
            rpmAlter_T = !rpmAlter_T; 
      } } else if(receivedData == "1") { 
            data = 1;            
        }else if(receivedData == "0") { 
            data = 0;            
        }else if(receivedData == "2") { 
            data = 2;            
        }else if(receivedData == "5") { 
            data = 5;            
        }else if(receivedData == "6") { 
            data = 6;            
        }else if(receivedData == "7") { 
            data = 7;            
        }else if(receivedData == "8") { 
            data = 8;            
        }else if (turn == true)
        {
          data = val;
         motion(data);
       }
         startTimeControlCounter = currentTimeControlCounter;
    }

int mapStringToInt(String str) {
         if(str == "11"){  return 11;} 
    else if(str == "12"){  return 12;} 
    else if(str == "13"){  return 13;}    // Map "11" to 'A'
    else if(str == "14"){   return 14;}   // Map "220" to 'B'
    else if(str == "15"){  return 15;}
    else if(str == "16"){  return 16;}
    else if(str == "17"){   return 17;}
    else if(str == "18"){   return 18;}
    else if(str == "19"){  return 19;}
    else if(str == "20"){   return 20;}
    else if(str == "21"){  return 21;}
    else if(str == "22"){   return 22;}
    else if(str == "23"){   return 23;}
    else if(str == "24"){   return 24;}
    else if(str == "25"){   return 25;}
    else if(str == "26"){   return 26;}
    else if(str == "27"){   return 27;}
    else if(str == "28"){   return 28;}
    else if(str == "29"){   return 29;}
    else if(str == "30"){   return 30;}
    else if(str == "110"){   return 110;}
    else if(str == "111"){   return 111;}
    else if(str == "112"){   return 112;}
    else if(str == "113"){   return 113;}
    else if(str == "114"){   return 114;}
    else if(str == "115"){   return 115;}
    else if(str == "116"){   return 116;}
    else if(str == "117"){   return 117;}
    else if(str == "118"){   return  118;}
    else if(str == "119"){   return 119;}
    else if(str == "120"){   return 120;}
    else if(str == "121"){   return 121;}
    else if(str == "122"){   return 122;}
    else if(str == "123"){   return 123;}
    else if(str == "124"){   return 124;}
    else if(str == "125"){   return 125;}
    else if(str == "126"){   return 126;}
    else if(str == "127"){   return 127;}
    else if(str == "128"){   return 128;}
    else if(str == "129"){   return 129;}
    else if(str == "130"){   return 130;}
    else if(str == "210"){   return 210;}
    else if(str == "211"){   return 211;}
    else if(str == "212"){   return 212;}
    else if(str == "213"){   return 213;}
    else if(str == "214"){   return 214;}
    else if(str == "215"){   return 215;}
    else if(str == "216"){   return 216;}
    else if(str == "217"){   return 217;}
    else if(str == "218"){   return 218;}
    else if(str == "219"){   return 219;}
    else if(str == "220"){   return 220;}
    else if(str == "221"){   return 221;}
    else if(str == "222"){   return 222;}
    else if(str == "223"){   return 223;}
    else if(str == "224"){   return 224;}
    else if(str == "225"){   return 225;}
    else if(str == "226"){   return 226;}
    else if(str == "227"){   return 227;}
    else if(str == "228"){   return 228;}
    else if(str == "229"){   return 229;}
    else if(str == "230"){   return 230;}

    else { return 0;}
  }