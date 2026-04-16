#ifndef DISTANCE_H
#define DISTANCE_H

#include <Arduino.h>

// ─────────────────────────────────────────────
//  TUNE THESE FOR YOUR ROBOT
// ─────────────────────────────────────────────

// Measure your wheel diameter in metres (e.g. 0.15 = 15 cm)
#define WHEEL_DIAMETER_M     0.15f

// Raw encoder ticks per one full rotation of the ENCODER SHAFT
// 4-channel quadrature, 1000 PPR → 4000 ticks/rev
#define TICKS_PER_ENC_REV    4000.0f

// ── Mechanical ratio ──────────────────────────────────────────────────────
// Encoder is physically separated from the wheel.
// Tachometer tests confirmed:  encoder_rpm / 1.6 = actual wheel_rpm
// i.e. the encoder shaft spins 1.6× faster than the wheel shaft.
// Must apply this the same way as in your RPM formula in main.cpp.
#define MECH_RATIO           1.6f

// rpmScale_L/R removed — were compensating for old broken encoder formula
// and are no longer needed with correct gray-code encoder

// ── Complementary filter weight ──────────────────────────────────────────
// 0.95 = trust encoder 95%, trust accelerometer 5%
// Raise toward 1.0 on clean flat floors.
// Lower toward 0.85 on carpet / ramps where slip is more likely.
#define COMP_FILTER_ALPHA    0.95f

// ── BNO055 LIA scale ─────────────────────────────────────────────────────
// Default: 1 m/s² = 100 LSB
#define BNO_LIA_SCALE        100.0f

// ── Acceleration deadband (m/s²) ─────────────────────────────────────────
// Values below this are clamped to zero to suppress noise at rest
#define ACCEL_DEADBAND       0.08f

// ─────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────

/** Call once in setup() after Wire1 / BNO055 is initialised. */
void distanceInit();

/**
 * Call inside `if (elaspedTime > timeConstant)` BEFORE resetting encoderValue_L/R.
 *
 * @param encTicks_L  current encoderValue_L
 * @param encTicks_R  current encoderValue_R
 * @param dt_s        elapsed time in SECONDS  (pass elaspedTime / 1000.0f)
 */
void distanceUpdate(long encTicks_L, long encTicks_R, float dt_s);

/** Total fused distance in metres since init / last reset. */
float distanceGetTotal_m();

/** Current fused velocity in m/s. */
float distanceGetVelocity_ms();

/** Encoder-only distance in metres (debug). */
float distanceGetEncoder_m();

/** Accelerometer-only distance in metres (debug — drifts over time). */
float distanceGetAccel_m();

/** Zero all accumulators — call at the start of a new measurement segment. */
void distanceReset();

#endif // DISTANCE_H