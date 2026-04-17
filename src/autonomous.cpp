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
    bool isMoving = (currentCmd == 1 || currentCmd == 2);

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

// Per-segment state
static float  s_segStartDist_m      = 0.0f;   // distanceGetTotal at segment start
static float  s_segStartHeading     = 0.0f;   // heading at segment start
static float  s_segTargetHeading    = 0.0f;   // target heading for rotation segments
static bool   s_segInitialised      = false;

// Live segment error (remaining distance or heading)
static float  s_segError            = 0.0f;

// Last completed segment final error (overshoot = positive for distance)
static float  s_lastSegError        = 0.0f;

// Settling state — robot pauses between segments so encoders/distance settle
static bool     s_settling          = false;
static uint32_t s_settleStart       = 0;

// ── helpers ──────────────────────────────────

static float normaliseHeading(float h) {
    while (h < 0.0f)    h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

static float headingDiff(float from, float to) {
    float d = to - from;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// ── parsing ──────────────────────────────────

bool autonomousStartTrack(const char* trackStr) {
    s_trackLen   = 0;
    s_currentSeg = 0;
    s_running    = false;
    s_segInitialised = false;
    s_settling   = false;
    s_segError   = 0.0f;
    s_lastSegError = 0.0f;

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
    Serial.println("──────────────────────────────────");
    Serial.println("[AUTO] Track loaded:");
    for (int i = 0; i < s_trackLen; i++) {
        const TrackSegment& s = s_track[i];
        switch (s.type) {
            case SEG_FORWARD:  Serial.printf("  %d: FWD   %.2f m\n",   i, s.value); break;
            case SEG_BACKWARD: Serial.printf("  %d: BACK  %.2f m\n",   i, s.value); break;
            case SEG_ROTATE_R: Serial.printf("  %d: ROT_R %.1f deg\n", i, s.value); break;
            case SEG_ROTATE_L: Serial.printf("  %d: ROT_L %.1f deg\n", i, s.value); break;
            case SEG_STOP:     Serial.printf("  %d: STOP\n", i); break;
        }
    }
    Serial.println("──────────────────────────────────");

    s_running        = true;
    s_segInitialised = false;
    Serial.println("[AUTO] Track STARTED");
    return true;
}

// ── runtime ──────────────────────────────────

void autonomousInit() {
    s_running    = false;
    s_trackLen   = 0;
    s_currentSeg = 0;
    s_segInitialised = false;
    s_settling   = false;
    s_segError   = 0.0f;
    s_lastSegError = 0.0f;
    measuredDistanceInit();
}

void autonomousUpdate(float currentHeading, float currentDist_m) {
    if (!s_running) return;

    // ── Settling phase: robot is stopped between segments ────────
    if (s_settling) {
        data = 0;  // keep motors off
        if ((millis() - s_settleStart) >= SEGMENT_SETTLE_MS) {
            s_settling = false;
            // Log the final settled distance for the just-completed segment
            Serial.printf("[AUTO] Settle done. Total dist: %.3f m\n", currentDist_m);

            if (s_currentSeg >= s_trackLen) {
                s_running = false;
                Serial.println("[AUTO] ═══ Track COMPLETE ═══");
                return;
            }
        } else {
            return;  // still settling, do nothing
        }
    }

    if (s_currentSeg >= s_trackLen) {
        data = 0;
        s_running = false;
        Serial.println("[AUTO] ═══ Track COMPLETE ═══");
        return;
    }

    TrackSegment& seg = s_track[s_currentSeg];

    // ── Initialise segment on first entry ────────────
    if (!s_segInitialised) {
        s_segStartDist_m   = currentDist_m;
        s_segStartHeading  = currentHeading;

        switch (seg.type) {
            case SEG_FORWARD:
                Serial.printf("[AUTO] ▶ Seg %d/%d: Forward %.2f m  (start dist: %.3f m)\n",
                              s_currentSeg + 1, s_trackLen, seg.value, currentDist_m);
                data = 1;
                break;
            case SEG_BACKWARD:
                Serial.printf("[AUTO] ▶ Seg %d/%d: Backward %.2f m  (start dist: %.3f m)\n",
                              s_currentSeg + 1, s_trackLen, seg.value, currentDist_m);
                data = 2;
                break;
            case SEG_ROTATE_R:
                s_segTargetHeading = normaliseHeading(currentHeading + seg.value);
                Serial.printf("[AUTO] ▶ Seg %d/%d: Rotate R %.1f° → target %.1f°  (current: %.1f°)\n",
                              s_currentSeg + 1, s_trackLen, seg.value,
                              s_segTargetHeading, currentHeading);
                data = 11;
                break;
            case SEG_ROTATE_L:
                s_segTargetHeading = normaliseHeading(currentHeading - seg.value);
                Serial.printf("[AUTO] ▶ Seg %d/%d: Rotate L %.1f° → target %.1f°  (current: %.1f°)\n",
                              s_currentSeg + 1, s_trackLen, seg.value,
                              s_segTargetHeading, currentHeading);
                data = 21;
                break;
            case SEG_STOP:
                Serial.println("[AUTO] ■ STOP segment reached");
                data = 0;
                s_running = false;
                Serial.println("[AUTO] ═══ Track COMPLETE ═══");
                return;
        }
        s_segInitialised = true;
    }

    // ── Compute live error ───────────────────────────
    bool complete = false;

    switch (seg.type) {
        case SEG_FORWARD:
        case SEG_BACKWARD: {
            float travelled = currentDist_m - s_segStartDist_m;
            float remaining = seg.value - travelled;
            s_segError = remaining;   // positive = still to go, negative = overshoot

            // Complete when we have covered the FULL target distance (no tolerance)
            if (travelled >= seg.value) {
                float overshoot = travelled - seg.value;
                s_lastSegError = overshoot;
                complete = true;
                Serial.printf("[AUTO] ✓ Seg %d done | target: %.3f m | actual: %.3f m | error: %+.3f m\n",
                              s_currentSeg + 1, seg.value, travelled, overshoot);
            }
            break;
        }
        case SEG_ROTATE_R:
        case SEG_ROTATE_L: {
            float remaining = headingDiff(currentHeading, s_segTargetHeading);
            s_segError = remaining;

            if (fabsf(remaining) <= HEADING_TOLERANCE_DEG) {
                s_lastSegError = remaining;
                complete = true;
                Serial.printf("[AUTO] ✓ Seg %d done | heading: %.1f° | target: %.1f° | error: %+.1f°\n",
                              s_currentSeg + 1, currentHeading, s_segTargetHeading, remaining);
            }
            break;
        }
        case SEG_STOP:
            complete = true;
            break;
    }

    if (complete) {
        // Stop motors and enter settling phase
        data = 0;
        s_segError = 0.0f;

        s_currentSeg++;
        s_segInitialised = false;

        // Enter settling phase — wait for robot to physically stop
        s_settling    = true;
        s_settleStart = millis();
        Serial.printf("[AUTO] Settling %d ms before next segment...\n", SEGMENT_SETTLE_MS);
    }
}

void autonomousAbort() {
    s_running  = false;
    s_settling = false;
    data = 0;
    s_segError = 0.0f;
    Serial.println("[AUTO] ✕ ABORTED");
}

bool autonomousIsRunning() {
    return s_running || s_settling;
}

int autonomousCurrentSegment() {
    return s_currentSeg;
}

int autonomousTotalSegments() {
    return s_trackLen;
}

float autonomousGetSegmentError() {
    return s_segError;
}

float autonomousGetLastSegError() {
    return s_lastSegError;
}
