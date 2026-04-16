#include <var.h>
#include <EEPROM.h>
#include <Arduino.h>

static bool applySettingUpdate(const String &key, const String &payload, const char *name, int &target, unsigned int address) {
  if (key != name) {
    return false;
  }
  target = handleEEPROMwrite(payload, address);
  return true;
}


void printSetting() {
  Serial.println("Chaging to Setting Mode");
  Serial.print(" T -> Trun \n F -> Forward \n R -> Right \n L -> Left \n B -> Backward \n W -> Work \n D -> Drive \n");
  Serial.println("Enter With as shown with the range below."); 
  Serial.println();
  Serial.print(" TRR : Values form 0 - 63    \n TRL : Values form 192 - 255\n"); 
  Serial.print(" TLR : Values form 64 - 127  \n TLL : Values form 128 - 191\n");
  Serial.println();
  Serial.print(" FRW : Values form 0 - 40    \n FLW : Values form 128 - 168\n"); 
  Serial.print(" BRW : Values form 64 - 104  \n BLW : Values form 192 - 232\n");
  Serial.println();
  Serial.print(" FRD : Values form 40 - 63    \n FLD : Values form 168 - 191\n"); 
  Serial.print(" BRD : Values form 104 - 127  \n BLD : Values form 232 - 255\n");
}

void updatedEEPROM(String _data) {
  const int separatorIndex = _data.indexOf(':');
  if (separatorIndex < 0) {
    Serial.println("Enter Correct Variable");
    return;
  }

  const String key = _data.substring(0, separatorIndex);

  if (applySettingUpdate(key, _data, "TRR", TRR, addressTRR)) return;
  if (applySettingUpdate(key, _data, "TRL", TRL, addressTRL)) return;
  if (applySettingUpdate(key, _data, "TLR", TLR, addressTLR)) return;
  if (applySettingUpdate(key, _data, "TLL", TLL, addressTLL)) return;
  if (applySettingUpdate(key, _data, "FRW", FRW, addressFRW)) return;
  if (applySettingUpdate(key, _data, "FLW", FLW, addressFLW)) return;
  if (applySettingUpdate(key, _data, "BRW", BRW, addressBRW)) return;
  if (applySettingUpdate(key, _data, "BLW", BLW, addressBLW)) return;
  if (applySettingUpdate(key, _data, "FRD", FRD, addressFRD)) return;
  if (applySettingUpdate(key, _data, "FLD", FLD, addressFLD)) return;
  if (applySettingUpdate(key, _data, "BRD", BRD, addressBRD)) return;
  if (applySettingUpdate(key, _data, "BLD", BLD, addressBLD)) return;

  Serial.println("Enter Correct Variable");
}

int handleEEPROMwrite(String _data, unsigned int address) {
  EEPROM.update(address, _data.substring(_data.indexOf(':')).remove(0,1).toInt());
  Serial.print("Data Updated | ");
  Serial.print(_data.substring(0,_data.indexOf(':')));
  Serial.print(" : ");
  Serial.println(EEPROM.read(address));
  return EEPROM.read(address);
}

void readEEPROM() { //If Address id updated. Updated the same at Readme File!!
  FLW = EEPROM.read(addressFLW);
  FRW = EEPROM.read(addressFRW);
  BLW = EEPROM.read(addressBLW);
  BRW = EEPROM.read(addressBRW);

  FLD = EEPROM.read(addressFLD);
  FRD = EEPROM.read(addressFRD);
  BLD = EEPROM.read(addressBLD);
  BRD = EEPROM.read(addressBRD);

  TRR = EEPROM.read(addressTRR);
  TRL = EEPROM.read(addressTRL);
  TLR = EEPROM.read(addressTLR);
  TLL = EEPROM.read(addressTLL);
}