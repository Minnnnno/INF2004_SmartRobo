/*
 *	detour_plan.h
 *	Subsystem 5 -- the geometry of driving round a scanned obstacle and back
 *	to the line.
 *
 *	Pure computation, no hardware, so the host tests (subsystem5_test.c)
 *	run the same code as the robot.  detour.c drives the plan.
 *
 *	Frame: the robot's pose when it stopped at the obstacle.  x is forward
 *	along the original line, y is to the LEFT (the sign obstacle_scan.h and
 *	steering.h use), headings are degrees, positive left.  Positions are of
 *	the axle centre, the point Motion Control turns about.
 *
 *	1. Back up if there is no room to turn, turn out by turn_deg, drive
 *	   diagonal_mm, turn back parallel to the line: the robot's centre is
 *	   then shift_mm to the side, clear of the obstacle's edge.
 *	2. Step forward, looking sideways at the obstacle; s5_pass_observe()
 *	   finds where it ends.  A missed echo alone never ends it.
 *	3. Drive on until the robot's rear corner clears that end
 *	   (s5_pass_clear_x), turn in by S5_RETURN_DEG and drive back across
 *	   the line, stopping when the line sensors see it.
 */

#ifndef S5_DETOUR_PLAN_H
#define S5_DETOUR_PLAN_H

#include "obstacle_scan.h"

/* ---- robot dimensions, from the axle centre: MEASURE THESE ---- */
#define S5_SERVO_AHEAD_MM	60.0	/* servo axis ahead of the axle */
#define S5_FRONT_MM		100.0	/* front of the robot ahead of the axle */
#define S5_REAR_MM		60.0	/* back of the robot behind the axle */
#define S5_IR_AHEAD_MM		80.0	/* line sensors ahead of the axle */

/* ---- going out ---- */
#define S5_TURN_MARGIN_MM	30.0	/* kept from the face by the front corners while turning */
#define S5_OUT_MIN_DEG		45.0	/* outward turn: this, or steeper when there is little room */
#define S5_OUT_MAX_DEG		75.0	/* steepest outward turn; back up rather than exceed it */
#define S5_BACKUP_MAX_MM	150.0	/* refuse a detour that needs more reversing */
/* Stopped ultrasonic checks along the diagonal and before passing. Logical
   servo angles are 60, 90, 120 deg (left to right).  With S5_GUARD_BEAM_HALF_DEG
   below, 30 deg spacing still leaves no gap between the beams. */
#define S5_DIAGONAL_STEP_MM	400.0 /* longest diagonal leg between stops; a typical diagonal is one leg */
#define S5_DIAGONAL_EXTRA_MM	50.0 /* driven beyond the planned diagonal, for extra room off the obstacle */
#define S5_MIN_DIAGONAL_STEP_MM	10.0 /* shorter diagonal steps do not move the wheels (4 mm gave 0 counts) */
#define S5_GUARD_SWEEP_DEG	30
#define S5_GUARD_SWEEP_STEP_DEG	30
/* An echo may come from inside the beam rather than its axis. Expand the
   tested corridor by this angular uncertainty; this is a conservative guard,
   not a replacement for the calibrated width scan. */
#define S5_GUARD_BEAM_HALF_DEG	15.0

/* ---- passing ---- */
/* The pass is one continuous move while the one servo-mounted sensor alternates
   between the side and ahead.  One side + ahead cycle: two 70 deg swings (380 ms
   each) and 5 pings at each end (300 ms each).  The geometry tests check that
   this spacing survives a missed echo: 150 mm does not, so raising the drive
   speed much past 100 mm/s needs a shorter cycle. */
#define S5_PASS_CYCLE_S		1.4
#define S5_PASS_STEP_MM		(S5_DRIVE_SPEED_MM_S * S5_PASS_CYCLE_S)	/* travel between looks */
#define S5_PASS_MAX_MM		800.0	/* give up if the obstacle has not ended after this */
/* Taken off the distance driven past the obstacle's end before turning back.
   It uses up the rear-corner swing and the margin there, relying on the sideways
   gap instead: see check_pass() in subsystem5_test.c. */
#define S5_PASS_TRIM_MM		150.0
/* The servo's limits (20/160 deg) make "sideways" 70 deg off straight ahead */
#define S5_SIDE_LOOK_DEG	(S5_ANGLE_CENTRE_DEG - S5_ANGLE_LEFT_DEG)
/* A side echo this much further out than the obstacle's edge is still the
   obstacle; anything further out is something else (or nothing). */
#define S5_SIDE_SEE_MM		150.0
#define S5_PASS_CONFIRM		2U	/* clear side looks in a row that end the obstacle */

/* ---- coming back ---- */
#define S5_RETURN_DEG		45.0	/* turn in towards the line by this */
#define S5_RETURN_EXTRA_MM	120.0	/* drive this far past the expected crossing before giving up */
/* Black seen further than this from where odometry puts the line is not it */
#define S5_LINE_WINDOW_MM	100.0
#define S5_DRIVE_SPEED_MM_S	80.0	/* backing up, diagonal, passing (motion limit is 250) */
#define S5_LINE_SEARCH_SPEED_MM_S 60.0	/* the leg that watches for the line: slow enough to stop on it */

typedef struct {
	bool		b_ok;		/* false: no safe detour, stay stopped */
	s5_avoid_side_t	side;
	double		shift_mm;	/* sideways move of the centre, + left */
	double		edge_mm;	/* lateral position of the edge being passed */
	double		face_mm;	/* obstacle face, x */
	double		backup_mm;	/* reverse this far first (>= 0) */
	double		turn_deg;	/* outward turn, + left */
	double		diagonal_mm;	/* drive this far after the turn */
	double		out_x_mm;	/* centre x when back parallel to the line */
} s5_detour_plan_t;

/* Plan step 1 from a finished scan.  Needs a pass side with room. */
bool	s5_detour_plan(const s5_scan_t *p_scan, s5_detour_plan_t *p_plan);

typedef struct {
	double		rear_mm;	/* furthest x the obstacle has been seen at */
	double		end_mm;		/* x where the first of the latest clear looks crossed
					   its side: the obstacle ends before this */
	double		gap_mm;		/* centre line to the obstacle's side, last look */
	unsigned	clear_run;	/* clear looks in a row past rear_mm */
	unsigned	seen;		/* looks that heard the obstacle */
	bool		b_ended;
} s5_pass_t;

void	s5_pass_init(s5_pass_t *p_pass, const s5_detour_plan_t *p_plan);
/* One look at S5_SIDE_LOOK_DEG towards the obstacle with the robot parallel
   to the line at (x_mm, y_mm).  b_echo/range_mm: the agreed reading, range
   from the transducer (as returned by s5_us_ping). The sensor's offset from
   the servo axis is applied here. Returns true once the obstacle has ended. */
bool	s5_pass_observe(s5_pass_t *p_pass, const s5_detour_plan_t *p_plan,
			double x_mm, double y_mm, bool b_echo, double range_mm);
/* Centre x from which turning in cannot swing the rear into the obstacle
   (once s5_pass_observe has returned true) */
double	s5_pass_clear_x(const s5_pass_t *p_pass);

/* Length of the return leg from lateral y_mm at S5_RETURN_DEG, with
   S5_RETURN_EXTRA_MM to spare */
double	s5_return_length(double y_mm);
/* TRUE when the line sensors are near enough where the line should be
   for black under them to be the line */
bool	s5_line_expected(double y_mm, double heading_deg);

#endif /* S5_DETOUR_PLAN_H */
