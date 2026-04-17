#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <Arduino.h>

// ─────────────────────────────────────────────
//  Measured Distance — tracks accumulated distance
//  during forward (cmd 1) or backward (cmd 2) motion.
//  Resets automatically when a new 1/2 command is
//  issued after the robot was stopped (cmd 0).
// ─────────────────────────────────────────────

/** Call once in setup(). */
void measuredDistanceInit();

/**
 * Call every loop iteration (or at least every distance-update cycle).
 * @param currentCmd  the current motion command (data variable)
 * @param fusedDist_m total fused distance from distanceGetTotal_m()
 */
void measuredDistanceUpdate(int currentCmd, float fusedDist_m);

/** Returns the distance measured since the last auto-reset (metres). */
float measuredDistanceGet_m();

/** Manually reset the measured distance counter. */
void measuredDistanceReset();

// ─────────────────────────────────────────────
//  Autonomous Track-Line System
//
//  Input a sequence of waypoints via Serial:
//    T:F20,R90,F10,R90,S
//
//  Segment types:
//    F<metres>   — drive Forward  the given distance
//    B<metres>   — drive Backward the given distance
//    R<degrees>  — Rotate right (clockwise) by degrees
//    L<degrees>  — rotate Left  (counter-clockwise) by degrees
//    S           — Stop (end of track)
//
//  Example: "T:F20,R90,F10,R90,S"
//    → go forward 20 m
//    → turn right 90°
//    → go forward 10 m
//    → turn right 90°
//    → stop
// ─────────────────────────────────────────────

/** Maximum segments in a single track program. */
#define MAX_TRACK_SEGMENTS 32

/** Settling time between segments (ms) — lets robot fully stop & distance stabilise */
#define SEGMENT_SETTLE_MS  500

/** Heading tolerance for rotation segments (degrees) */
#define HEADING_TOLERANCE_DEG  2.0f

enum SegmentType : uint8_t {
    SEG_FORWARD,     // drive forward  N metres
    SEG_BACKWARD,    // drive backward N metres
    SEG_ROTATE_R,    // rotate right (CW)  N degrees
    SEG_ROTATE_L,    // rotate left  (CCW) N degrees
    SEG_STOP         // end of track
};

struct TrackSegment {
    SegmentType type;
    float       value;  // metres or degrees depending on type
};

/** Call once in setup(). */
void autonomousInit();

/**
 * Call every loop iteration BEFORE motion().
 * When a track is active this function overrides `data` to drive the robot.
 *
 * @param currentHeading  latest heading in degrees (0-360)
 * @param currentDist_m   total fused distance from distanceGetTotal_m()
 */
void autonomousUpdate(float currentHeading, float currentDist_m);

/**
 * Parse a track-line string and start autonomous execution.
 * Format: "F20,R90,F10,R90,S"  (the "T:" prefix is stripped by the caller)
 * Returns true if parsing succeeded.
 */
bool autonomousStartTrack(const char* trackStr);

/** Abort the current autonomous track immediately. */
void autonomousAbort();

/** Returns true if the autonomous system is currently running a track. */
bool autonomousIsRunning();

/** Returns the index of the segment currently being executed (0-based). */
int  autonomousCurrentSegment();

/** Returns total number of segments in the loaded track. */
int  autonomousTotalSegments();

/**
 * Returns the live error for the current segment:
 *   - For F/B segments: remaining distance in metres (positive = still to go)
 *   - For R/L segments: remaining angle in degrees
 *   - 0 if no track is running
 */
float autonomousGetSegmentError();

/**
 * Returns the final error of the LAST completed segment:
 *   - For F/B: actual distance − target distance (positive = overshoot)
 *   - For R/L: actual heading error at completion
 */
float autonomousGetLastSegError();

#endif // AUTONOMOUS_H
