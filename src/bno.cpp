#include <var.h>
#include <string.h>
#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data)  {
  kire1.beginTransmission(i2c_addr);
  kire1.write(reg);
  kire1.write(data);
  kire1.endTransmission(true);  // send stop
}

void bno_read_multiple(uint8_t i2c_addr, uint8_t reg, uint8_t *buf, uint8_t length) {
  for (uint32_t n=0; n<length; n++) {
    if ((n & 31) == 0) {
      kire1.beginTransmission(i2c_addr);
      kire1.write(reg+n);
      kire1.endTransmission(false);  // send restart
      kire1.requestFrom(i2c_addr, min(length-n, 32), true);
    }
    
    while(kire1.available()) {
    *buf++ = kire1.read();
    }
  }
}

void thread_func() {
  while(1) bno_read_multiple(BNO_ADDR, ACC_DATA_X_LSB, (uint8_t*)&imu, sizeof imu);
}