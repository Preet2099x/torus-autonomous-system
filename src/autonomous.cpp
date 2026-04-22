#include "autonomous.h"
#include "distance.h"
#include <var.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

extern float latestSerialHeading;   // from main.cpp

// ═════════════════════════════════════════════
//  MEASURED DISTANCE
// ═════════════════════════════════════════════

static float  s_measuredDist_m     = 0.0f;   // accumulated since last reset
static float  s_measuredBaseline_m = 0.0f;   // distanceGetTotal_m() at reset
static bool   s_wasStopped         = true;    // was the robot stopped (cmd 0)?
static int    s_prevCmd            = 0;

void measuredDistanceInit() {
    s_measuredDist_m     = 0.0f;
    s_measuredBaseline_m = 0.0f;
    s_wasStopped         = true;
    s_prevCmd            = 0;
}

void measuredDistanceUpdate(int currentCmd, float fusedDist_m) {
    bool isMoving = (currentCmd == 1 || currentCmd == 2 || currentCmd == 31 || currentCmd == 32);

    // Detect transition: was stopped → now moving
    if (isMoving && s_wasStopped) {
        // New movement segment — reset baseline
        s_measuredBaseline_m = fusedDist_m;
        s_measuredDist_m     = 0.0f;
    }

    if (isMoving) {
        s_measuredDist_m = fusedDist_m - s_measuredBaseline_m;
    }

    // Track stopped state
    s_wasStopped = (currentCmd == 0);
    s_prevCmd    = currentCmd;
}

float measuredDistanceGet_m() {
    return s_measuredDist_m;
}

void measuredDistanceReset() {
    s_measuredDist_m     = 0.0f;
    s_measuredBaseline_m = distanceGetTotal_m();
}

// ═════════════════════════════════════════════
//  AUTONOMOUS TRACK-LINE SYSTEM
// ═════════════════════════════════════════════

static TrackSegment s_track[MAX_TRACK_SEGMENTS];
static int          s_trackLen       = 0;
static int          s_currentSeg     = 0;
static bool         s_running        = false;
static bool         s_paused         = false;
static unsigned long s_pauseStart    = 0;
static unsigned long s_trackStartMs  = 0;
static unsigned long s_trackPausedMs = 0;

// Per-segment state
static float  s_segStartDist_m      = 0.0f;   // distanceGetTotal at segment start
static float  s_segStartHeading     = 0.0f;   // heading at segment start
static float  s_segTargetHeading    = 0.0f;   // target heading for rotation segments
static float  s_segTotalAngle       = 0.0f;   // total degrees to rotate (positive)
static float  s_segAccumAngle       = 0.0f;   // accumulated signed rotation (+ = CW)
static float  s_segPrevHeading      = 0.0f;   // previous heading for delta tracking
static bool   s_segInitialised      = false;
static unsigned long s_segStartMs   = 0;
static unsigned long s_segPausedMs  = 0;

// Chunked linear movement state (for F/B segments)
static float  s_linearRemaining_m    = 0.0f;
static float  s_linearChunkStart_m   = 0.0f;
static float  s_linearChunkTarget_m  = 0.0f;
static float  s_linearCumulative_m   = 0.0f;

// Settling state: brief pause between segments to let motors/IMU settle
static bool   s_settling            = false;
static unsigned long s_settleStart  = 0;
static const unsigned long SETTLE_TIME_MS = 150;  // ms to pause between segments

// Heading-turn tolerance (degrees)
static const float HEADING_TOLERANCE = 1.5f;

// Rotation braking compensation (degrees)
// Stop command is issued slightly early to account for inertial turn overshoot.
static const float ROT_BRAKING_OVERSHOOT_DEG = 2.0f;

// Distance braking compensation (metres)
// The robot overshoots slightly after motors stop due to inertia.
// This value is subtracted from the target so the robot hits the exact mark.
static const float BRAKING_OVERSHOOT_FWD_M  = 0.02f;
static const float BRAKING_OVERSHOOT_BACK_M = 0.05f;

// Deceleration approach distance (metres)
// When within this distance of target, apply slow speed
static const float DECEL_APPROACH_FWD_M  = 0.15f;
static const float DECEL_APPROACH_BACK_M = 0.24f;

// Linear chunk size (metres)
static const float LINEAR_CHUNK_M = 1.0f;

// ── helpers ──────────────────────────────────

static float normaliseHeading(float h) {
    while (h < 0.0f)    h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

/// Signed shortest-arc difference (from → to), range [-180, +180]
static float headingDiff(float from, float to) {
    float d = to - from;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// (delta-accumulation is used instead of point-to-point comparison
//  to avoid 180° ambiguity bugs)

// ── parsing ──────────────────────────────────

bool autonomousStartTrack(const char* trackStr) {
    s_trackLen   = 0;
    s_currentSeg = 0;
    s_running    = false;
    s_paused     = false;
    s_trackStartMs = 0;
    s_trackPausedMs = 0;
    s_segInitialised = false;

    if (!trackStr || trackStr[0] == '\0') return false;

    // Make a mutable copy
    char buf[256];
    strncpy(buf, trackStr, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // Tokenise by comma
    char* token = strtok(buf, ",");
    while (token && s_trackLen < MAX_TRACK_SEGMENTS) {
        // Skip leading whitespace
        while (*token == ' ') token++;

        TrackSegment seg;
        seg.value = 0.0f;

        char code = toupper(token[0]);
        switch (code) {
            case 'F':
                seg.type  = SEG_FORWARD;
                seg.value = atof(token + 1);
                break;
            case 'B':
                seg.type  = SEG_BACKWARD;
                seg.value = atof(token + 1);
                break;
            case 'R':
                seg.type  = SEG_ROTATE_R;
                seg.value = atof(token + 1);
                break;
            case 'L':
                seg.type  = SEG_ROTATE_L;
                seg.value = atof(token + 1);
                break;
            case 'S':
                seg.type  = SEG_STOP;
                seg.value = 0;
                break;
            default:
                Serial.printf("[AUTO] Unknown segment code '%c'\n", code);
                return false;
        }

        s_track[s_trackLen++] = seg;
        token = strtok(NULL, ",");
    }

    if (s_trackLen == 0) return false;

    // Print parsed track
    Serial.println("[AUTO] Track loaded:");
    for (int i = 0; i < s_trackLen; i++) {
        const TrackSegment& s = s_track[i];
        switch (s.type) {
            case SEG_FORWARD:  Serial.printf("  %d: FWD  %.2f m\n",  i, s.value); break;
            case SEG_BACKWARD: Serial.printf("  %d: BACK %.2f m\n",  i, s.value); break;
            case SEG_ROTATE_R: Serial.printf("  %d: ROT_R %.1f deg\n", i, s.value); break;
            case SEG_ROTATE_L: Serial.printf("  %d: ROT_L %.1f deg\n", i, s.value); break;
            case SEG_STOP:     Serial.printf("  %d: STOP\n", i); break;
        }
    }

    s_running        = true;
    s_paused         = false;
    s_trackStartMs   = millis();
    s_trackPausedMs  = 0;
    s_segInitialised = false;
    s_settling       = false;
    Serial.println("[AUTO] Track started.");
    return true;
}

// ── runtime ──────────────────────────────────

void autonomousInit() {
    s_running    = false;
    s_paused     = false;
    s_trackStartMs = 0;
    s_trackPausedMs = 0;
    s_trackLen   = 0;
    s_currentSeg = 0;
    s_segInitialised = false;
    s_segStartMs = 0;
    s_segPausedMs = 0;
    s_settling       = false;
    measuredDistanceInit();
}

void autonomousUpdate(float currentHeading, float currentDist_m) {
    if (!s_running) return;
    if (s_paused) {
        data = 0;
        return;
    }
    if (s_currentSeg >= s_trackLen) {
        // Track finished
        data = 0;
        s_running = false;
        unsigned long totalMs = millis() - s_trackStartMs;
        if (totalMs >= s_trackPausedMs) totalMs -= s_trackPausedMs;
        else totalMs = 0;
        Serial.printf("[AUTO] Track complete. Total active time: %.3f s\n", totalMs / 1000.0f);
        return;
    }

    TrackSegment& seg = s_track[s_currentSeg];

    // ── Initialise segment on first entry ────────────
    if (!s_segInitialised) {
        s_segStartDist_m   = currentDist_m;
        s_segStartHeading  = currentHeading;
        s_segStartMs       = millis();
        s_segPausedMs      = 0;

        switch (seg.type) {
            case SEG_FORWARD:
                Serial.printf("[AUTO] Seg %d: Forward %.2f m\n", s_currentSeg, seg.value);
                s_linearRemaining_m   = seg.value;
                s_linearChunkStart_m  = currentDist_m;
                s_linearChunkTarget_m = (s_linearRemaining_m > LINEAR_CHUNK_M) ? LINEAR_CHUNK_M : s_linearRemaining_m;
                s_linearCumulative_m  = 0.0f;
                measuredDistanceReset();
                Serial.printf("[AUTO]   chunk target %.2f m, remaining %.2f m\n",
                              s_linearChunkTarget_m, s_linearRemaining_m);
                data = 1;
                break;
            case SEG_BACKWARD:
                Serial.printf("[AUTO] Seg %d: Backward %.2f m\n", s_currentSeg, seg.value);
                s_linearRemaining_m   = seg.value;
                s_linearChunkStart_m  = currentDist_m;
                s_linearChunkTarget_m = (s_linearRemaining_m > LINEAR_CHUNK_M) ? LINEAR_CHUNK_M : s_linearRemaining_m;
                s_linearCumulative_m  = 0.0f;
                measuredDistanceReset();
                Serial.printf("[AUTO]   chunk target %.2f m, remaining %.2f m\n",
                              s_linearChunkTarget_m, s_linearRemaining_m);
                data = 2;
                break;
            case SEG_ROTATE_R:
                // CW rotation: target = current + degrees
                s_segTotalAngle    = seg.value;
                s_segTargetHeading = normaliseHeading(currentHeading + seg.value);
                s_segAccumAngle    = 0.0f;
                s_segPrevHeading   = currentHeading;
                Serial.printf("[AUTO] Seg %d: Rotate R %.1f° (start %.1f° → target %.1f°)\n",
                              s_currentSeg, seg.value, currentHeading, s_segTargetHeading);
                data = 11;
                break;
            case SEG_ROTATE_L:
                // CCW rotation: target = current - degrees
                s_segTotalAngle    = seg.value;
                s_segTargetHeading = normaliseHeading(currentHeading - seg.value);
                s_segAccumAngle    = 0.0f;
                s_segPrevHeading   = currentHeading;
                Serial.printf("[AUTO] Seg %d: Rotate L %.1f° (start %.1f° → target %.1f°)\n",
                              s_currentSeg, seg.value, currentHeading, s_segTargetHeading);
                data = 21;
                break;
            case SEG_STOP:
                Serial.println("[AUTO] Seg: STOP");
                data = 0;
                s_running = false;
                {
                    unsigned long segMs = millis() - s_segStartMs;
                    if (segMs >= s_segPausedMs) segMs -= s_segPausedMs;
                    else segMs = 0;
                    Serial.printf("[AUTO] Seg %d time: %.3f s\n", s_currentSeg, segMs / 1000.0f);
                }
                {
                    unsigned long totalMs = millis() - s_trackStartMs;
                    if (totalMs >= s_trackPausedMs) totalMs -= s_trackPausedMs;
                    else totalMs = 0;
                    Serial.printf("[AUTO] Track complete. Total active time: %.3f s\n", totalMs / 1000.0f);
                }
                return;
        }
        s_segInitialised = true;
    }

    // ── Handle settling pause between segments ────────
    if (s_settling) {
        data = 0;  // motors off during settle
        if ((millis() - s_settleStart) >= SETTLE_TIME_MS) {
            s_settling = false;
            s_segInitialised = false;
            s_currentSeg++;
            if (s_currentSeg >= s_trackLen) {
                s_running = false;
                unsigned long totalMs = millis() - s_trackStartMs;
                if (totalMs >= s_trackPausedMs) totalMs -= s_trackPausedMs;
                else totalMs = 0;
                Serial.printf("[AUTO] Track complete. Total active time: %.3f s\n", totalMs / 1000.0f);
            }
        }
        return;
    }

    // Re-assert active segment motion command every loop.
    // This is required for pause/resume because pause forces data=0.
    switch (seg.type) {
        case SEG_FORWARD:
            data = 1;
            break;
        case SEG_BACKWARD:
            data = 2;
            break;
        case SEG_ROTATE_R:
            data = 11;
            break;
        case SEG_ROTATE_L:
            data = 21;
            break;
        case SEG_STOP:
            data = 0;
            break;
    }

    // ── Check if segment is complete ─────────────────
    bool complete = false;

    switch (seg.type) {
        case SEG_FORWARD:
        case SEG_BACKWARD: {
            float travelled = currentDist_m - s_linearChunkStart_m;
            float remaining = s_linearChunkTarget_m - travelled;
            bool isBackward = (seg.type == SEG_BACKWARD);
            float decelApproach = isBackward ? DECEL_APPROACH_BACK_M : DECEL_APPROACH_FWD_M;
            float brakingOvershoot = isBackward ? BRAKING_OVERSHOOT_BACK_M : BRAKING_OVERSHOOT_FWD_M;

            // Decelerate while approaching target to reduce overshoot.
            if (remaining <= decelApproach) {
                data = (seg.type == SEG_FORWARD) ? 31 : 32;
            } else {
                data = (seg.type == SEG_FORWARD) ? 1 : 2;
            }

            // Effective target accounts for braking overshoot
            float effectiveTarget = s_linearChunkTarget_m - brakingOvershoot;
            if (effectiveTarget < 0.0f) effectiveTarget = 0.0f;

            if (travelled >= effectiveTarget) {
                s_linearCumulative_m += travelled;
                s_linearRemaining_m -= s_linearChunkTarget_m;
                if (s_linearRemaining_m <= 0.001f) {
                    complete = true;
                    Serial.printf("[AUTO] Seg %d done — cumulative %.3f m (target %.3f m)\n",
                                  s_currentSeg, s_linearCumulative_m, seg.value);
                } else {
                    s_linearChunkStart_m  = currentDist_m;
                    s_linearChunkTarget_m = (s_linearRemaining_m > LINEAR_CHUNK_M) ? LINEAR_CHUNK_M : s_linearRemaining_m;
                    measuredDistanceReset();
                    Serial.printf("[AUTO] Seg %d chunk complete — cumulative %.3f m, next chunk %.2f m, remaining %.2f m\n",
                                  s_currentSeg, s_linearCumulative_m, s_linearChunkTarget_m, s_linearRemaining_m);
                }
            }
            break;
        }
        case SEG_ROTATE_R: {
            // CW rotation: accumulate small heading deltas each loop
            float delta = headingDiff(s_segPrevHeading, currentHeading); // signed
            s_segAccumAngle += delta;   // positive = CW progress
            s_segPrevHeading = currentHeading;

            float swept = s_segAccumAngle;  // CW is positive
            float effectiveTarget = s_segTotalAngle - ROT_BRAKING_OVERSHOOT_DEG;
            if (effectiveTarget < 0.0f) effectiveTarget = 0.0f;
            if (swept >= (effectiveTarget - HEADING_TOLERANCE)) {
                complete = true;
                Serial.printf("[AUTO] Seg %d done — heading %.1f° (target %.1f°, swept %.1f°)\n",
                              s_currentSeg, currentHeading, s_segTargetHeading, swept);
            }
            break;
        }
        case SEG_ROTATE_L: {
            // CCW rotation: accumulate small heading deltas each loop
            float delta = headingDiff(s_segPrevHeading, currentHeading); // signed
            s_segAccumAngle += delta;   // negative = CCW progress
            s_segPrevHeading = currentHeading;

            float swept = -s_segAccumAngle;  // flip sign: CCW progress is positive
            float effectiveTarget = s_segTotalAngle - ROT_BRAKING_OVERSHOOT_DEG;
            if (effectiveTarget < 0.0f) effectiveTarget = 0.0f;
            if (swept >= (effectiveTarget - HEADING_TOLERANCE)) {
                complete = true;
                Serial.printf("[AUTO] Seg %d done — heading %.1f° (target %.1f°, swept %.1f°)\n",
                              s_currentSeg, currentHeading, s_segTargetHeading, swept);
            }
            break;
        }
        case SEG_STOP:
            complete = true;
            break;
    }

    if (complete) {
        unsigned long segMs = millis() - s_segStartMs;
        if (segMs >= s_segPausedMs) segMs -= s_segPausedMs;
        else segMs = 0;
        Serial.printf("[AUTO] Seg %d time: %.3f s\n", s_currentSeg, segMs / 1000.0f);

        // Enter settling phase: stop motors briefly before next segment
        data = 0;
        s_settling    = true;
        s_settleStart = millis();
    }
}

void autonomousAbort() {
    s_running = false;
    s_paused  = false;
    data = 0;
    Serial.println("[AUTO] Aborted.");
}

void autonomousSetPaused(bool paused) {
    if (!s_running) return;
    if (s_paused == paused) return;

    if (paused) {
        s_paused = true;
        s_pauseStart = millis();
        data = 0;
        Serial.printf("[AUTO] Paused at segment %d.\n", s_currentSeg);
    } else {
        unsigned long pausedFor = millis() - s_pauseStart;
        s_trackPausedMs += pausedFor;
        if (s_segInitialised) {
            s_segPausedMs += pausedFor;
        }
        if (s_settling) {
            s_settleStart += pausedFor;
        }
        s_paused = false;
        Serial.printf("[AUTO] Resumed at segment %d.\n", s_currentSeg);
    }
}

void autonomousTogglePaused() {
    autonomousSetPaused(!s_paused);
}

bool autonomousIsPaused() {
    return s_paused;
}

bool autonomousIsRunning() {
    return s_running;
}

int autonomousCurrentSegment() {
    return s_currentSeg;
}

float autonomousLinearCumulative_m() {
    if (!s_running || s_currentSeg >= s_trackLen) return 0.0f;

    SegmentType type = s_track[s_currentSeg].type;
    if (type != SEG_FORWARD && type != SEG_BACKWARD) return 0.0f;

    return s_linearCumulative_m + measuredDistanceGet_m();
}
