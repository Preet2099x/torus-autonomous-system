// Lightweight shim to provide kire/kire1 TwoWire aliases
#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <Wire.h>

extern TwoWire &kire;
extern TwoWire &kire1;

#endif // I2C_DRIVER_H
