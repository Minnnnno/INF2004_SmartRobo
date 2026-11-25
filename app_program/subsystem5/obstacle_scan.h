/*
 *  obstacle_scan.h
 *  Subsystem 5 -- obstacle width, position and which side to pass it on,
 *  from a servo-swept ultrasonic ranger.
 *
 *  Pure computation, no hardware: the caller supplies the servo and ping
 *  callbacks, so the same code runs on the robot and in the host tests
 *  (subsystem5_test.c).
 *
 *  Servo angles are degrees as mounted on the robot: 20 = left,
 *  90 = straight ahead, 160 = right.  Distances are millimetres measured
 *  from the servo axis.  Lateral positions are positive to the LEFT, the
 *  same sign the robot uses for headings and turns (steering.h).
 */

#ifndef S5_OBSTACLE_SCAN_H
#define S5_OBSTACLE_SCAN_H

#include "../../include/float_types.h"
#include <stdbool.h>
#include <stdint.h>

/* ---- sweep geometry ---- */
#define S5_ANGLE_LEFT_DEG   20
#define S5_ANGLE_CENTRE_DEG 90
#define S5_ANGLE_RIGHT_DEG  160
#define S5_COARSE_STEP_DEG  10
#define S5_FINE_STEP_DEG    1
#define S5_COARSE_POINTS \
    ((S5_ANGLE_RIGHT_DEG - S5_ANGLE_LEFT_DEG) / S5_COARSE_STEP_DEG + 1)
#define S5_FINE_MAX_POINTS \
    ((S5_ANGLE_CENTRE_DEG - S5_ANGLE_LEFT_DEG) / S5_FINE_STEP_DEG)
/* Each edge is also walked back inward from clear space, so the servo
   reaches it moving the other way; averaging the two cancels gear backlash. */
#define S5_INWARD_MAX_STEPS 20

/* A point belongs to the obstacle when its forward distance (the part of
   the range pointing straight ahead) is within this fraction of the 90 deg
   forward distance.  Forward distance, not raw range: the raw range to a
   flat face grows as 1/cos(angle), e.g. 400 mm becomes 522 mm at 40 deg,
   which would wrongly look like a gap behind the obstacle.  A fraction, not
   fixed mm, so the rule is equally strict at any stop distance: 0.3 is
   ~80 mm at 250 mm, ~54 mm at the 150 mm stop. */
#define S5_EDGE_THRESHOLD_FRACTION 0.3
/* The forward-distance rule above misses a face that slopes towards the robot
   (a tilted box, or a robot angled to it after a curve): its forward distance
   changes along the face, so it looks like a gap.  While walking an edge, a
   point whose range is within this of the previous point on the obstacle is the
   same surface.  An edge is the range jumping by more, so a box in front of a
   distant wall still ends where it does.  Sloping surfaces run on to the limit
   of the sweep: that edge is "not found" and that side is not passed. */
#define S5_CONTINUITY_MM 25.0

/* ---- per-angle measurement ---- */
#define S5_SAMPLES_PER_POINT 5U /* pings at every angle */
#define S5_RETRY_SAMPLES     5U /* extra pings when the first ones disagree */
#define S5_MAX_SAMPLES       10U
/* pings this close are one echo (sensor ~3 mm) */
#define S5_CLUSTER_TOL_MM 10.0
#define S5_MIN_AGREE      3U /* and at least this many must agree */

/* Servo axis to the sensor, along the beam: the sensor faces straight out
   along the horn arm, so the offset simply adds to every range and the
   geometry is centred on the axis. */
#define S5_SENSOR_OFFSET_MM 30.0

/* The ultrasonic beam is a cone, so the obstacle is still heard after the
   centre line has passed its edge and the width reads too large.  Each
   edge is moved this many degrees back towards 90.  Calibrate with an
   object of known width (README.md).  22 deg: a 200 mm box 252 mm away
   was still heard 17 and 27 deg past its edges (2026-11-03); not yet
   recalibrated for the 150 mm stop.  The host
   tests override it with 0 to check the geometry with an ideal ray. */
#ifndef S5_BEAM_CORRECTION_DEG
#define S5_BEAM_CORRECTION_DEG 22.0
#endif

/* Centred when the two edges are this close to equally far from 90 deg.
   Five scans of centred objects put the obstacle's middle 3.6 deg apart
   (standard deviation), i.e. ~7 deg in this difference; 12 deg (~1.7 sd)
   is ~19 mm of centre offset at the 150 mm stop. */
#define S5_CENTRE_TOLERANCE_DEG 12.0

/* ---- avoidance: which side to pass on ---- */
#define S5_ROBOT_HALF_WIDTH_MM 80.0 /* robot is 160 mm wide */
#define S5_SERVO_AHEAD_MM      60.0 /* servo axis ahead of the axle: MEASURE */
/* kept from the obstacle and anything beside it */
#define S5_PASS_MARGIN_MM 50.0
/* Something beside the obstacle blocks that side only if its forward
   distance is within this of the obstacle's face. */
#define S5_PASS_LOOKAHEAD_MM 300.0
/* Off-axis echoes from the obstacle's own corners are "clear" but no
   further than its farthest point plus this; they are not a second object. */
#define S5_SPILL_RANGE_MM 50.0
/* Side to pass a centred obstacle on, when both sides have room */
#define S5_DEFAULT_AVOID S5_AVOID_RIGHT

/* ---- the face: how the robot is tilted to the obstacle ----
   Two line sensors give the robot's position, not its angle, so it can stop
   tilted to the obstacle, most of all just after a curve.  Readings of a flat
   face depend only on the angle from its perpendicular, so they rise equally
   either side of it, and the beam hears a corner at that corner's own range
   however far past it the beam points.  So from the ranges alone:
     tilt     the midpoint of where the range rises through one level each side
     distance the shortest range (D)
     reach    of each corner along the face: sqrt(range^2 - D^2)
   None of that needs the width of the beam, so unlike the edge angles it needs
   no S5_BEAM_CORRECTION_DEG, and a corner still heard at the limit of the sweep
   is still placed. */
/* the shortest range is this near straight ahead */
#define S5_FACE_SEARCH_DEG 40
/* a bigger step between neighbours ends the face */
#define S5_FACE_JUMP_MM 45.0
/* a last reading this far past the one before is a long grazing echo, not the
 * corner */
#define S5_FACE_TAIL_MM 20.0
/* ... unless that reading is still on the plane fitted so far: a flat wall's
   range rises fast between coarse angles.  Between what a beam this wide would
   read and a little over the ray along its axis. */
#define S5_FACE_PLANE_BEAM_DEG 25.0
#define S5_FACE_PLANE_OVER     1.1
#define S5_FACE_MIN_POINTS     3U
/* less rise on either side says nothing of the tilt */
#define S5_FACE_MIN_RISE_MM  5.0
#define S5_FACE_MAX_TILT_DEG 45.0 /* a fit past this is not believed */
/* with both edges seen, average in their tilt if this close */
#define S5_FACE_EDGE_AGREE_DEG 10.0
/* A face still at the sweep limit has a corner there if its range has stopped
   rising; if it is still rising, the corner is beyond the sweep. */
#define S5_FACE_FLAT_MM  12.0
#define S5_FACE_FLAT_DEG 20
/* Centred when the corners' reaches differ by no more than this fraction of
   D, twice over: about 20 mm off centre at the 150 mm stop. */
#define S5_FACE_CENTRE_FRACTION 0.11
/* A centred face is passed on the side that was closer when the robot stopped
   (it points away from that side), if it was tilted by at least this. */
#define S5_FACE_TILT_MIN_DEG 5.0
/* A reading this close past the end of the face can be the corner's own echo:
   it only blocks that side if a reading further out, within the second angle,
   puts something at about the same range. */
#define S5_FACE_EDGE_ECHO_DEG    3
#define S5_FACE_EDGE_CONFIRM_DEG 5
/* Added to every tilt.  MEASURE: the tilt a square-on box reads, with the sign
   reversed.  Logged scans lean one way, which may be servo trim. */
#ifndef S5_FACE_TILT_BIAS_DEG
#define S5_FACE_TILT_BIAS_DEG 0.0
#endif

/* Log only (the "curve check" line): nothing is decided from it. */
/* A curve is a heading change of S5_CURVE_MIN_DEG within the last
   S5_CURVE_WINDOW_MM of travel, remembered for S5_CURVE_HOLD_MM more.
   Above the line follower's zigzag on a straight: tune from the
   "curve check" line the scan prints. */
#define S5_CURVE_MIN_DEG    25.0
#define S5_CURVE_WINDOW_MM  300
#define S5_CURVE_HOLD_MM    300.0
#define S5_CURVE_SPACING_MM 20
#define S5_CURVE_SAMPLES    (S5_CURVE_WINDOW_MM / S5_CURVE_SPACING_MM + 1)

typedef enum
{
    S5_ECHO_OK,      /* a clear majority of pings agree */
    S5_ECHO_NONE,    /* most pings heard nothing */
    S5_ECHO_UNSTABLE /* echoes, but no majority agrees */
} s5_echo_t;

typedef struct
{
    s5_echo_t status;
    float64_t range_mm;   /* mean of the agreeing pings */
    unsigned  support;    /* pings in the agreeing group */
    unsigned  valid;      /* pings that heard an echo */
    unsigned  total;      /* pings taken */
    float64_t nearest_mm; /* nearest and furthest echo of all (valid > 0) */
    float64_t farthest_mm;
} s5_estimate_t;

/* Stored ranges: 0.25 mm steps, 0..4000 mm in 14 bits */
#define S5_RANGE_STEPS_PER_MM 4.0
#define S5_RANGE_CODE_MAX     16000U
uint16_t  s5_range_code(float64_t range_mm); /* rounded, clamped */
float64_t s5_range_mm(unsigned code);

/* A stored reading, 32 bits:
     0..13 range code   14..21 angle - S5_ANGLE_LEFT_DEG   22..25 support
     26..29 valid       30 total (0: S5_SAMPLES_PER_POINT, 1: with the retry)
     31 the obstacle
   The status follows from support, valid and total; nearest_mm and farthest_mm
   are not kept.  Owner: obstacle_task only, so changing a bit needs no lock. */
typedef struct
{
    uint32_t bits;
} s5_point_t;

s5_point_t s5_point_pack(int angle_deg, const s5_estimate_t * p_estimate,
                         bool b_object);
int        s5_point_angle(s5_point_t point);
float64_t  s5_point_range(s5_point_t point);
unsigned   s5_point_support(s5_point_t point);
unsigned   s5_point_valid(s5_point_t point);
unsigned   s5_point_total(s5_point_t point);
s5_echo_t  s5_point_status(s5_point_t point);
bool       s5_point_usable(s5_point_t point); /* as s5_estimate_usable() */
bool       s5_point_object(s5_point_t point);
void       s5_point_set_object(s5_point_t * p_point);

enum
{
    S5_LEFT  = 0,
    S5_RIGHT = 1
};
#define S5_SIDES 2U

typedef struct
{
    bool    b_found  : 1;    /* clear space seen beyond the edge */
    bool    b_inward : 1;    /* the inward walk found the obstacle again */
    uint8_t last_object_deg; /* outermost angle still on the obstacle */
    /* first clear angle past it (= last when not found) */
    uint8_t first_clear_deg;
    /* midpoint walking outward (last_object..first_clear) */
    float64_t outward_deg;
    /* midpoint walking back inward (= outward_deg if not done) */
    float64_t inward_deg;
    /* edge estimate: mean of both walks, after beam correction */
    float64_t angle_deg;
    float64_t range_mm;   /* obstacle range at last_object_deg */
    float64_t lateral_mm; /* edge position, + left */
} s5_edge_t;

typedef enum
{
    S5_POSITION_UNKNOWN, /* no obstacle, or both edges beyond the sweep */
    S5_POSITION_CENTRE,
    S5_POSITION_LEFT, /* extends further left than right */
    S5_POSITION_RIGHT
} s5_position_t;

typedef struct
{
    s5_position_t position;
    /* left edge, degrees left of straight ahead */
    float64_t left_deg;
    /* right edge, degrees right of straight ahead */
    float64_t right_deg;
    float64_t difference_deg; /* left_deg - right_deg: + further left */
    /* an edge lay beyond the sweep: that angle is a minimum */
    bool b_bounded;
} s5_placement_t;

typedef struct
{
    /* false: too little to go on, the edge angles decide */
    bool b_valid : 1;
    /* that corner has been placed: s5_face_corner() */
    bool b_corner_left  : 1;
    bool b_corner_right : 1;
    /* [S5_LEFT], [S5_RIGHT]: servo angle where the face was last heard */
    uint8_t end_deg[2];
    /* bearing of the face's perpendicular from straight ahead, + left.
     * Negative: the robot points left of square to the face */
    float64_t tilt_deg;
    float64_t distance_mm; /* servo axis to the face, along the perpendicular */
    /* left corner, along the face from the perpendicular's foot */
    float64_t left_mm;
    float64_t right_mm; /* right corner, the same way */
    float64_t width_mm;
    float64_t offset_mm; /* the face's middle from that foot, + left */
    float64_t far_mm;    /* longest range on the face */
} s5_face_t;

static inline bool
s5_face_corner (const s5_face_t * p_face, unsigned side)
{
    return ((unsigned)S5_LEFT == side) ? p_face->b_corner_left
                                       : p_face->b_corner_right;
}

typedef enum
{
    S5_AVOID_NONE, /* nothing ahead, or no side has room */
    S5_AVOID_LEFT,
    S5_AVOID_RIGHT
} s5_avoid_side_t;

typedef struct
{
    s5_avoid_side_t side;
    /* Sideways move of the robot's centre that clears each side of the
       obstacle by S5_PASS_MARGIN_MM.  A minimum if that edge lay beyond
       the sweep. */
    float64_t shift_mm[2]; /* [S5_LEFT], [S5_RIGHT], both >= 0 */
    /* Nearest other echo beside the obstacle on each side, mm from the
       robot's centre line; INFINITY when none was heard. */
    float64_t clear_mm[2];
    bool      b_room[2]; /* edge found, and the robot fits past it */
} s5_avoid_t;

typedef struct
{
    bool    b_valid; /* an obstacle echoed straight ahead */
    uint8_t coarse_count;
    uint8_t fine_count[2];
    /* fine[side][0..this) walked outward, the rest inward */
    uint8_t    fine_outward[2];
    s5_point_t centre;
    s5_point_t coarse[S5_COARSE_POINTS];
    s5_point_t fine[2][S5_FINE_MAX_POINTS + S5_INWARD_MAX_STEPS];
    float64_t  centre_forward_mm;
    s5_edge_t  edge[2];   /* [S5_LEFT], [S5_RIGHT] */
    float64_t  width_mm;  /* lower bound unless both ends are known */
    float64_t  offset_mm; /* obstacle centre, + left */
    /* tilt, distance and corners; decides when valid */
    s5_face_t face;
    /* turned before this scan to face it, + left; the caller sets it */
    float64_t squared_deg;
    /* centre / left / right: from the face when valid, else the edge angles;
     * its degrees are always the edges' */
    s5_placement_t placement;
    s5_avoid_t     avoid; /* which side to pass on */
} s5_scan_t;

/* One ping.  Returns true with *p_range_mm set when an echo came back. */
typedef bool (*s5_ping_fn_t)(void * p_context, float64_t * p_range_mm);

/* Point the sensor at angle_deg and measure there.  Return false to abort. */
typedef bool (*s5_measure_fn_t)(void * p_context, int angle_deg,
                                s5_estimate_t * p_estimate);

float64_t s5_forward_mm(float64_t range_mm, float64_t angle_deg);
float64_t s5_lateral_mm(float64_t range_mm, float64_t angle_deg);

/* Robust "mode with a tolerance": the largest group of pings that agree
   within S5_CLUSTER_TOL_MM wins and is averaged.  Missed pings count as
   votes for "nothing there".  echo: bit i set when ping i heard something. */
void s5_estimate(const float64_t range_mm[], uint16_t echo, unsigned count,
                 s5_estimate_t * p_out);
/* TRUE when the estimate is a usable echo (an unstable one counts only if
   its group outvotes the silent pings). */
bool s5_estimate_usable(const s5_estimate_t * p_estimate);
bool s5_point_is_object(const s5_estimate_t * p_estimate, int angle_deg,
                        float64_t centre_forward_mm);

/* S5_SAMPLES_PER_POINT pings, plus S5_RETRY_SAMPLES more if they disagree. */
void s5_sample_point(s5_ping_fn_t p_ping, void * p_context,
                     s5_estimate_t * p_out);

/* Centre reading, coarse sweep 20 -> 160 deg, then a 1 deg walk outward
   from each coarse edge until clear space is seen twice in a row, and back
   inward until the obstacle is seen twice in a row.
   Returns false only if measure() aborted; check p_scan->b_valid. */
bool s5_scan_run(s5_scan_t * p_scan, s5_measure_fn_t p_measure,
                 void * p_context);

/* Compare how far each edge is from straight ahead.  Within
   S5_CENTRE_TOLERANCE_DEG is centre.  An edge beyond the sweep is only a
   minimum angle, so it can show the obstacle is to that side but never
   that it is centred.  s5_scan_run fills p_scan->placement with this. */
void s5_placement(const s5_scan_t * p_scan, s5_placement_t * p_out);

/* Pass on s5_preferred_side(), the other side if that one has no room, NONE
   if neither has.  A side has room when its end is known and the nearest echo
   beside the obstacle leaves the robot S5_PASS_MARGIN_MM.  With a valid face
   all of it is measured square to the face, from the axle.  s5_scan_run fills
   p_scan->avoid with this. */
void s5_avoidance(const s5_scan_t * p_scan, s5_avoid_t * p_out);

/* The side to try first: away from the way the obstacle reaches further; a
   centred face on the side that was closer at the stop; else S5_DEFAULT_AVOID.
 */
s5_avoid_side_t s5_preferred_side(const s5_scan_t * p_scan);
/* Just enough to correct the robot's angle: the coarse sweep and the face fit,
   with no fine scan, width or decision.  s5_scan_run() measures the obstacle.
   False: the robot moved or the sensor failed. */
bool s5_tilt_scan(s5_scan_t * p_scan, s5_measure_fn_t p_measure,
                  void * p_context);
/* The tilt the robot stopped with: what is left, plus what it turned to square
 * up */
float64_t s5_start_tilt_deg(const s5_scan_t * p_scan);

/* The face from the coarse sweep and the outward fine walks.  s5_scan_run
   fills p_scan->face with this. */
void s5_face_fit(const s5_scan_t * p_scan, s5_face_t * p_out);

/* Recent heading changes, from Motion Control's path and heading.  Path is
   whole mm and heading 0.1 deg, both modulo 2^16: only differences are used. */
typedef struct
{
    uint16_t path_mm[S5_CURVE_SAMPLES];
    uint16_t heading_ddeg[S5_CURVE_SAMPLES];
    uint16_t now_mm;     /* path at the last note */
    uint16_t until_mm;   /* counts as just after a curve up to this path */
    uint16_t swing_ddeg; /* largest heading change in the window, last note */

    uint16_t count  : 5;
    uint16_t next   : 4;
    uint16_t marked : 1; /* until_mm means something; cleared once passed */
} s5_curve_t;

void s5_curve_reset(s5_curve_t * p_curve);
void s5_curve_note(s5_curve_t * p_curve, float64_t path_mm,
                   float64_t heading_deg);
/* Treat the next S5_CURVE_HOLD_MM as just after a curve, e.g. after a rejoin */
void s5_curve_mark(s5_curve_t * p_curve, float64_t path_mm);
bool s5_curve_recent(const s5_curve_t * p_curve);

#endif /* S5_OBSTACLE_SCAN_H */

/*** end of file ***/
