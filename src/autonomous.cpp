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
static float  s_pausePreservedDist = 0.0f;   // distance saved across pauses
static bool   s_wasStopped         = true;    // was the robot stopped (cmd 0)?
static int    s_prevCmd            = 0;

void measuredDistanceInit() {
    s_measuredDist_m     = 0.0f;
    s_measuredBaseline_m = 0.0f;
    s_pausePreservedDist = 0.0f;
    s_wasStopped         = true;
    s_prevCmd            = 0;
}

void measuredDistanceUpdate(int currentCmd, float fusedDist_m) {
    bool isMoving = (currentCmd == 1 || currentCmd == 2);

    // Detect transition: was stopped → now moving
    if (isMoving && s_wasStopped) {
        // New movement segment — reset baseline
        s_measuredBaseline_m = fusedDist_m;
        s_measuredDist_m     = s_pausePreservedDist;  // carry over paused distance
    }

    if (isMoving) {
        s_measuredDist_m = s_pausePreservedDist + (fusedDist_m - s_measuredBaseline_m);
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
static float  s_segTotalAngle       = 0.0f;   // total degrees to rotate (positive)
static float  s_segAccumAngle       = 0.0f;   // accumulated signed rotation (+ = CW)
static float  s_segPrevHeading      = 0.0f;   // previous heading for delta tracking
static bool   s_segInitialised      = false;

// Settling state: brief pause between segments to let motors/IMU settle
static bool   s_settling            = false;
static unsigned long s_settleStart  = 0;
static const unsigned long SETTLE_TIME_MS = 150;  // ms to pause between segments

// Pause state: freeze autonomous execution and preserve all data
static bool   s_paused              = false;
static int    s_pausedCmd           = 0;       // motor command active before pause
static float  s_pauseSegDist_m      = 0.0f;    // segment dist already covered before pause
static float  s_pauseSegAccumAngle  = 0.0f;    // rotation accumulated before pause

// Heading-turn tolerance (degrees)
static const float HEADING_TOLERANCE = 0.1f;

// Rotation braking compensation (degrees)
// Stop command is issued slightly early to account for inertial turn overshoot.
static const float ROT_BRAKING_OVERSHOOT_DEG = 3.55f;
// Distance braking compensation (metres)
// The robot overshoots slightly after motors stop due to inertia.
// This value is subtracted from the target so the robot hits the exact mark.
static const float BRAKING_OVERSHOOT_M = 0.02f;

// Deceleration approach distance (metres)
// When within this distance of target, apply slow speed
static const float DECEL_APPROACH_M = 0.15f;

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

        // Trim trailing whitespace
        size_t tokenLen = strlen(token);
        while (tokenLen > 0 && (token[tokenLen - 1] == ' ' || token[tokenLen - 1] == '\t')) {
            token[--tokenLen] = '\0';
        }

        TrackSegment seg;
        seg.value = 0.0f;

        // Special continuous circle commands: RC / LC
        // Reversed mapping by request:
        //   RC -> left-circle path, LC -> right-circle path
        if ((toupper(token[0]) == 'R') && (toupper(token[1]) == 'C') && token[2] == '\0') {
            seg.type = SEG_CIRCLE_L;
        } else if ((toupper(token[0]) == 'L') && (toupper(token[1]) == 'C') && token[2] == '\0') {
            seg.type = SEG_CIRCLE_R;
        } else {
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
                if (token[1] == '\0') {
                    Serial.println("[AUTO] Invalid token 'R' (use R<deg> or RC).");
                    return false;
                }
                seg.type  = SEG_ROTATE_R;
                seg.value = atof(token + 1);
                break;
            case 'L':
                if (token[1] == '\0') {
                    Serial.println("[AUTO] Invalid token 'L' (use L<deg> or LC).");
                    return false;
                }
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
            case SEG_CIRCLE_R: Serial.printf("  %d: CIRCLE_R (continuous)\n", i); break;
            case SEG_CIRCLE_L: Serial.printf("  %d: CIRCLE_L (continuous)\n", i); break;
            case SEG_STOP:     Serial.printf("  %d: STOP\n", i); break;
        }
    }

    s_running        = true;
    s_segInitialised = false;
    s_settling       = false;
    Serial.println("[AUTO] Track started.");
    return true;
}

// ── runtime ──────────────────────────────────

void autonomousInit() {
    s_running    = false;
    s_paused     = false;
    s_trackLen   = 0;
    s_currentSeg = 0;
    s_segInitialised = false;
    s_settling       = false;
    measuredDistanceInit();
}

void autonomousUpdate(float currentHeading, float currentDist_m) {
    if (!s_running || s_paused) return;
    if (s_currentSeg >= s_trackLen) {
        // Track finished
        data = 0;
        s_running = false;
        Serial.println("[AUTO] Track complete.");
        return;
    }

    TrackSegment& seg = s_track[s_currentSeg];

    // ── Initialise segment on first entry ────────────
    if (!s_segInitialised) {
        s_segStartDist_m   = currentDist_m;
        s_segStartHeading  = currentHeading;

        switch (seg.type) {
            case SEG_FORWARD:
                Serial.printf("[AUTO] Seg %d: Forward %.2f m\n", s_currentSeg, seg.value);
                data = 1;
                break;
            case SEG_BACKWARD:
                Serial.printf("[AUTO] Seg %d: Backward %.2f m\n", s_currentSeg, seg.value);
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
            case SEG_CIRCLE_R:
                Serial.printf("[AUTO] Seg %d: Right circle (continuous)\n", s_currentSeg);
                data = 121;
                break;
            case SEG_CIRCLE_L:
                Serial.printf("[AUTO] Seg %d: Left circle (continuous)\n", s_currentSeg);
                data = 111;
                break;
            case SEG_STOP:
                Serial.println("[AUTO] Seg: STOP");
                data = 0;
                s_running = false;
                Serial.println("[AUTO] Track complete.");
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
                Serial.println("[AUTO] Track complete.");
            }
        }
        return;
    }

    // ── Check if segment is complete ─────────────────
    bool complete = false;

    switch (seg.type) {
        case SEG_FORWARD:
        case SEG_BACKWARD: {
            float travelled = currentDist_m - s_segStartDist_m;
            // Effective target accounts for braking overshoot
            float effectiveTarget = seg.value - BRAKING_OVERSHOOT_M;
            if (effectiveTarget < 0.0f) effectiveTarget = 0.0f;

            if (travelled >= effectiveTarget) {
                complete = true;
                Serial.printf("[AUTO] Seg %d done — travelled %.3f m (target %.3f m)\n",
                              s_currentSeg, travelled, seg.value);
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
        case SEG_CIRCLE_R:
            data = 121;
            break;
        case SEG_CIRCLE_L:
            data = 111;
            break;
        case SEG_STOP:
            complete = true;
            break;
    }

    if (complete) {
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

bool autonomousIsRunning() {
    return s_running;
}

int autonomousCurrentSegment() {
    return s_currentSeg;
}

// ── Pause / Resume ───────────────────────────

void autonomousPause() {
    if (!s_running || s_paused) return;

    s_paused    = true;
    s_pausedCmd = data;      // remember what we were doing

    // Snapshot progress so we can rebase on resume
    TrackSegment& seg = s_track[s_currentSeg];
    if (seg.type == SEG_FORWARD || seg.type == SEG_BACKWARD) {
        // distance already covered in this segment
        s_pauseSegDist_m = distanceGetTotal_m() - s_segStartDist_m;
    }
    if (seg.type == SEG_ROTATE_R || seg.type == SEG_ROTATE_L) {
        s_pauseSegAccumAngle = s_segAccumAngle;
    }

    data = 0;                // stop motors
    Serial.println("[AUTO] Paused.");
}

void autonomousResume() {
    if (!s_running || !s_paused) return;

    // Rebase segment start so remaining distance is correct
    TrackSegment& seg = s_track[s_currentSeg];
    if (seg.type == SEG_FORWARD || seg.type == SEG_BACKWARD) {
        // New baseline = current total dist minus what we already covered
        s_segStartDist_m = distanceGetTotal_m() - s_pauseSegDist_m;
    }
    if (seg.type == SEG_ROTATE_R || seg.type == SEG_ROTATE_L) {
        // Restore accumulated angle and reset heading tracker to current
        s_segAccumAngle  = s_pauseSegAccumAngle;
        s_segPrevHeading = latestSerialHeading;  // avoid delta jump
    }

    data      = s_pausedCmd; // restore motor command
    s_paused  = false;
    Serial.printf("[AUTO] Resumed (cmd=%d).\n", data);
}

void autonomousTogglePause() {
    if (s_paused) {
        autonomousResume();
    } else {
        autonomousPause();
    }
}

bool autonomousIsPaused() {
    return s_paused;
}
