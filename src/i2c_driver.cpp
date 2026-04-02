#include <i2c_driver.h>

// Bind the aliases to the Teensy/Arduino Wire instances
// On Teensy boards Wire and Wire1 are available; fall back to Wire if Wire1 missing.
TwoWire &kire = Wire;
#if defined(Wire1)
TwoWire &kire1 = Wire1;
#else
TwoWire &kire1 = Wire;
#endif
