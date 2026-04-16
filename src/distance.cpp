#include "distance.h"
#include <Wire.h>
#include <math.h>

// ─────────────────────────────────────────────
//  BNO055 — Linear Acceleration registers (gravity already removed in NDOF)
// ─────────────────────────────────────────────
static const uint8_t BNO_ADDR       = 0x28;
static const uint8_t LIA_X_LSB      = 0x28;   // forward axis  ← change to 0x2A (Y)
                                               // or 0x2C (Z) if your BNO is mounted
                                               // differently. Drive fwd and check sign.

// ─────────────────────────────────────────────
//  Pre-computed: effective ticks per WHEEL revolution
//
//  With the correct gray-code encoder the ticks map directly:
//      wheel_revs = ticks / (TICKS_PER_ENC_REV * MECH_RATIO)
// ─────────────────────────────────────────────
static const float EFF_TICKS_PER_WHEEL_REV =
    TICKS_PER_ENC_REV * MECH_RATIO;   // 4000 * 1.6 = 6400 ticks

static const float WHEEL_CIRC_M = float(M_PI) * WHEEL_DIAMETER_M;

// ─────────────────────────────────────────────
//  State
// ─────────────────────────────────────────────
static float s_totalDist_m     = 0.0f;
static float s_encoderDist_m   = 0.0f;
static float s_accelDist_m     = 0.0f;
static float s_velocity_ms     = 0.0f;
static float s_accelVel_ms     = 0.0f;   // integrated accel velocity

// ─────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────

static int16_t bnoRead16(uint8_t reg) {
    Wire1.beginTransmission(BNO_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission();
    Wire1.requestFrom(BNO_ADDR, (uint8_t)2);
    if (Wire1.available() < 2) return 0;
    uint8_t lsb = Wire1.read();
    uint8_t msb = Wire1.read();
    return (int16_t)(lsb | (msb << 8));
}

static float readLIA_ms2() {
    float a = (float)bnoRead16(LIA_X_LSB) / BNO_LIA_SCALE;
    return (fabsf(a) < ACCEL_DEADBAND) ? 0.0f : a;
}

/**
 * Convert raw encoder ticks → actual wheel distance (metres).
 *
 * With the correct gray-code encoder:
 *   wheel_revs = ticks / (TICKS_PER_ENC_REV * MECH_RATIO)
 *   distance   = wheel_revs * wheel_circumference
 */
static float ticksToMetres(long ticks_L, long ticks_R) {
    float revsL = (float)abs(ticks_L) / EFF_TICKS_PER_WHEEL_REV;
    float revsR = (float)abs(ticks_R) / EFF_TICKS_PER_WHEEL_REV;
    float avgRevs = (revsL + revsR) * 0.5f;
    return avgRevs * WHEEL_CIRC_M;
}

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────

void distanceInit() {
    distanceReset();
}

void distanceUpdate(long encTicks_L, long encTicks_R, float dt_s) {
    if (dt_s <= 0.0f) return;

    // ── 1. Encoder distance this interval ─────────────────────────────────
    //
    // ticksToMetres() already applies:
    //   - mechanical ratio (/ 1.6)
    // so this is the TRUE wheel distance, not the encoder-shaft distance.
    //
    float encDist = ticksToMetres(encTicks_L, encTicks_R);
    float encVel  = encDist / dt_s;              // m/s
    s_encoderDist_m += encDist;

    // ── 2. Accelerometer distance this interval ────────────────────────────
    //
    // BNO055 NDOF mode gives Linear Acceleration (LIA) with gravity removed.
    // Integrate once → velocity, use velocity × dt → distance.
    //
    float accel = readLIA_ms2();
    s_accelVel_ms += accel * dt_s;

    // Velocity reset guard: if encoder says stopped AND accel is quiet,
    // the robot is truly stationary — reset integrated velocity to prevent
    // drift accumulation.
    if (encVel < 0.005f && accel == 0.0f) {
        s_accelVel_ms = 0.0f;
    }

    float accelDist = fabsf(s_accelVel_ms) * dt_s;
    s_accelDist_m  += accelDist;

    // ── 3. Complementary filter ────────────────────────────────────────────
    //
    //   fused = α × encoder  +  (1-α) × accelerometer
    //
    // At α=0.95 the encoder dominates.  The accelerometer's 5% contribution
    // catches brief wheel-slip events where encoder ticks under-count.
    //
    float fusedDist = COMP_FILTER_ALPHA       * encDist
                    + (1.0f - COMP_FILTER_ALPHA) * accelDist;

    s_totalDist_m += fusedDist;

    s_velocity_ms  = COMP_FILTER_ALPHA       * encVel
                   + (1.0f - COMP_FILTER_ALPHA) * fabsf(s_accelVel_ms);
}

float distanceGetTotal_m()     { return s_totalDist_m;    }
float distanceGetVelocity_ms() { return s_velocity_ms;    }
float distanceGetEncoder_m()   { return s_encoderDist_m;  }
float distanceGetAccel_m()     { return s_accelDist_m;    }

void distanceReset() {
    s_totalDist_m   = 0.0f;
    s_encoderDist_m = 0.0f;
    s_accelDist_m   = 0.0f;
    s_velocity_ms   = 0.0f;
    s_accelVel_ms   = 0.0f;
}