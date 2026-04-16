#include <var.h>
#include <string.h>
#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

static bool isMappedCommandValue(int value) {
    const bool range11to30 = (value >= 11 && value <= 30);
    const bool range110to130 = (value >= 110 && value <= 130);
    const bool range210to230 = (value >= 210 && value <= 230);
    return range11to30 || range110to130 || range210to230;
}

static bool isSingleDigitDriveCommand(const String &receivedData) {
    return receivedData == "0" ||
                 receivedData == "1" ||
                 receivedData == "2" ||
                 receivedData == "5" ||
                 receivedData == "6" ||
                 receivedData == "7" ||
                 receivedData == "8";
}

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
    (void)bytesReceived;

    String receivedData = "";
    while (kire.available()) {
        receivedData += (char)kire.read();
    }

    int mappedValue = mapStringToInt(receivedData);
    turn = (mappedValue > 0);

    if (receivedData == "a") {
        if (data != 3 || data != 4 || data != 0) {
            rpmAlter = !rpmAlter;
        }
    } else if (receivedData == "b") {
        if (data != 2 || data != 1 || data != 0) {
            rpmAlter_T = !rpmAlter_T;
        }
    } else if (isSingleDigitDriveCommand(receivedData)) {
        data = receivedData.toInt();
    } else if (turn == true) {
        data = mappedValue;
        motion(data);
    }

    startTimeControlCounter = currentTimeControlCounter;
}

int mapStringToInt(String str) {
    if (str.length() == 0) {
        return 0;
    }

    const int parsedValue = str.toInt();
    if (String(parsedValue) != str) {
        return 0;
    }

    return isMappedCommandValue(parsedValue) ? parsedValue : 0;
}