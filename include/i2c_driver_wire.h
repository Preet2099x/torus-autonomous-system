// Optional helper header for I2C wire utility functions
#ifndef I2C_DRIVER_WIRE_H
#define I2C_DRIVER_WIRE_H

// Provide a lightweight init helper if needed by the project.
inline void initWire() {
  // default to starting both Wire ports; user can call explicitly if desired
  Wire.begin();
#if defined(Wire1)
  Wire1.begin();
#endif
}

#endif // I2C_DRIVER_WIRE_H
