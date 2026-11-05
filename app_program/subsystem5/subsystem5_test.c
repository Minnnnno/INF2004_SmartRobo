/*
 *	subsystem5_test.c
 *	Host tests for the subsystem 5 width, position and pass-side
 *	algorithm (obstacle_scan.c) and the detour geometry (detour_plan.c).
 *	Not part of the firmware: CMakeLists.txt excludes subsystem*_test.c.
 *
 *	Build and run with any native C compiler, from the repository root.
 *	The beam correction is calibrated for the real sensor's cone; the ideal
 *	ray here needs none, so it is overridden to 0:
 *	  gcc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow \
 *	      -DS5_BEAM_CORRECTION_DEG=0.0 \
 *	      app_program/subsystem5/obstacle_scan.c \
 *	      app_program/subsystem5/detour_plan.c \
 *	      app_program/subsystem5/subsystem5_test.c -lm -o s5_tests && ./s5_tests
 *
 *	A simulated box face sits in front of the servo, optionally with a second
 *	object beside it; each ping is traced along the servo bearing, with
 *	optional noise, stray echoes, a beam cone and servo gear backlash.
 */

#include "obstacle_scan.h"
#include "detour_plan.h"
#include "../../tests/subsystem5/face_scans.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEG_TO_RAD	(0.017453292519943295)

typedef struct {
	double		depth_mm;	/* box front face, forward of the servo axis */
	double		left_mm;	/* box edges, lateral, + left */
	double		right_mm;
	double		wall_mm;	/* background wall forward distance, 0 = none */
	double		noise_mm;	/* uniform +/- noise on every echo */
	double		cone_deg;	/* beam half-angle, 0 = ideal ray */
	double		backlash_deg;	/* horn lags this far behind the direction of travel */
	double		other_depth_mm;	/* second object beside the first, 0 = none */
	double		other_left_mm;
	double		other_right_mm;
	unsigned	stray_every;	/* every Nth ping is a stray echo or a miss */
	unsigned	seed;
	unsigned	pings;
	unsigned	measures;
	unsigned	abort_after;	/* measure() fails after this many, 0 = never */
	int		commanded_deg;	/* last servo command */
	int		direction;	/* +1 / -1: last move towards higher / lower angles */
	double		servo_deg;	/* where the horn really points */
} world_t;

static unsigned	failures;
static unsigned	checks;

static void check(bool b_ok, const char *p_name)
{
	checks++;
	if (!b_ok) {
		failures++;
		printf("FAIL  %s\n", p_name);
	}
}

static void check_near(double actual, double expected, double tolerance, const char *p_name)
{
	checks++;
	if (!(fabs(actual - expected) <= tolerance)) {
		failures++;
		printf("FAIL  %s: got %.1f, expected %.1f +/- %.1f\n", p_name, actual, expected, tolerance);
	}
}

static double world_random(world_t *p_world)	/* 0..1, deterministic */
{
	p_world->seed = p_world->seed * 1103515245U + 12345U;
	return (double)((p_world->seed >> 8) & 0xffffU) / 65535.0;
}

/* Echo from one flat face square to the robot: the nearest point of it
   inside the beam answers first. */
static bool face_echo(const world_t *p_world, double depth_mm, double left_mm, double right_mm,
		      double *p_range_mm)
{
	double	bearing = ((double)S5_ANGLE_CENTRE_DEG - p_world->servo_deg) * DEG_TO_RAD;
	double	cone = p_world->cone_deg * DEG_TO_RAD;
	/* the sensor rides on the horn, S5_SENSOR_OFFSET_MM out along the beam */
	double	sensor_fwd = S5_SENSOR_OFFSET_MM * cos(bearing);
	double	sensor_lat = S5_SENSOR_OFFSET_MM * sin(bearing);
	double	face_left = atan2(left_mm - sensor_lat, depth_mm - sensor_fwd);
	double	face_right = atan2(right_mm - sensor_lat, depth_mm - sensor_fwd);
	double	low = fmax(bearing - cone, face_right);
	double	high = fmin(bearing + cone, face_left);
	double	nearest;

	if (low > high) {
		return false;
	}
	nearest = (low > 0.0) ? low : ((high < 0.0) ? high : 0.0);
	*p_range_mm = (depth_mm - sensor_fwd) / cos(nearest);
	return true;
}

static bool world_ping(void *p_context, double *p_range_mm)
{
	world_t	*p_world = (world_t *)p_context;
	double	bearing = ((double)S5_ANGLE_CENTRE_DEG - p_world->servo_deg) * DEG_TO_RAD;
	double	sensor_fwd = S5_SENSOR_OFFSET_MM * cos(bearing);
	double	other_mm = 0.0;
	bool	b_other = (p_world->other_depth_mm > 0.0) &&
			  face_echo(p_world, p_world->other_depth_mm, p_world->other_left_mm,
				    p_world->other_right_mm, &other_mm);

	p_world->pings++;
	if ((p_world->stray_every != 0U) && (p_world->pings % p_world->stray_every == 0U)) {
		if (world_random(p_world) < 0.5) {
			return false;				/* missed echo */
		}
		*p_range_mm = 150.0 + 1350.0 * world_random(p_world);	/* stray echo */
		return true;
	}
	if (face_echo(p_world, p_world->depth_mm, p_world->left_mm, p_world->right_mm, p_range_mm)) {
		if (b_other) {
			*p_range_mm = fmin(*p_range_mm, other_mm);
		}
	} else if (b_other) {
		*p_range_mm = other_mm;
	} else if (p_world->wall_mm > 0.0) {
		*p_range_mm = (p_world->wall_mm - sensor_fwd) / cos(bearing);
	} else {
		return false;
	}
	*p_range_mm += p_world->noise_mm * (2.0 * world_random(p_world) - 1.0);
	return true;
}

static bool world_measure(void *p_context, int angle_deg, s5_estimate_t *p_estimate)
{
	world_t *p_world = (world_t *)p_context;

	p_world->measures++;
	if ((p_world->abort_after != 0U) && (p_world->measures > p_world->abort_after)) {
		return false;
	}
	if (angle_deg != p_world->commanded_deg) {
		p_world->direction = (angle_deg > p_world->commanded_deg) ? 1 : -1;
	}
	p_world->commanded_deg = angle_deg;
	p_world->servo_deg = (double)angle_deg - p_world->backlash_deg * (double)p_world->direction;
	s5_sample_point(world_ping, p_world, p_estimate);
	return true;
}

/* A flat face square to the line, seen by a robot yawed yaw_deg (+ left), the
   axis 180 mm from the face, as at the 150 mm stop.  An ideal ray from the servo
   axis, which the robot's turn leaves on the line. */
typedef struct {
	double	yaw_deg;
	double	half_width_mm;
	double	centre_mm;	/* middle of the face from the perpendicular's foot, + left */
} yawed_t;

static bool yawed_measure(void *p_context, int angle_deg, s5_estimate_t *p_estimate)
{
	const yawed_t	*p_yawed = (const yawed_t *)p_context;
	double		yaw = p_yawed->yaw_deg * DEG_TO_RAD;
	double		ray = ((double)S5_ANGLE_CENTRE_DEG - (double)angle_deg) * DEG_TO_RAD + yaw;
	double		face = 180.0 * cos(yaw);
	double		axis = face / cos(ray);
	double		range[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	bool		echo[5] = {false, false, false, false, false};
	unsigned	i;

	if ((cos(ray) > 0.0) && (fabs(axis * sin(ray) - p_yawed->centre_mm) <= p_yawed->half_width_mm)) {
		for (i = 0U; i < 5U; i++) {
			range[i] = axis - S5_SENSOR_OFFSET_MM;
			echo[i] = true;
		}
	}
	s5_estimate(range, echo, 5U, p_estimate);
	return true;
}

static world_t box(double depth_mm, double left_mm, double right_mm, double wall_mm)
{
	world_t world = {0};

	world.depth_mm = depth_mm;
	world.left_mm = left_mm;
	world.right_mm = right_mm;
	world.wall_mm = wall_mm;
	world.seed = 12345U;
	return world;
}

static void test_estimate(void)
{
	s5_estimate_t	e;
	const double	agree[5] = {400.0, 401.0, 399.0, 400.0, 402.0};
	const double	stray[5] = {400.0, 250.0, 401.0, 399.0, 900.0};
	const double	spread[5] = {300.0, 400.0, 500.0, 600.0, 700.0};
	const double	tie[4] = {600.0, 601.0, 400.0, 401.0};
	const bool	all[5] = {true, true, true, true, true};
	const bool	silent[5] = {false, true, false, true, false};
	const bool	one_miss[5] = {true, true, true, true, false};

	s5_estimate(agree, all, 5U, &e);
	check(e.status == S5_ECHO_OK && e.support == 5U, "estimate: five agreeing pings");
	check_near(e.range_mm, 400.4, 0.01, "estimate: mean of the agreeing pings");

	s5_estimate(stray, all, 5U, &e);
	check(e.status == S5_ECHO_OK && e.support == 3U, "estimate: two stray pings outvoted");
	check_near(e.range_mm, 400.0, 0.01, "estimate: strays excluded from the mean");

	s5_estimate(agree, silent, 5U, &e);
	check(e.status == S5_ECHO_NONE && !s5_estimate_usable(&e), "estimate: mostly silent is no echo");

	s5_estimate(agree, one_miss, 5U, &e);
	check(e.status == S5_ECHO_OK && e.valid == 4U, "estimate: one miss tolerated");

	s5_estimate(spread, all, 5U, &e);
	check(e.status == S5_ECHO_UNSTABLE && s5_estimate_usable(&e),
	      "estimate: no agreement is unstable but beats silence");

	s5_estimate(tie, all, 4U, &e);
	check_near(e.range_mm, 400.5, 0.01, "estimate: equal groups prefer the nearer echo");

	s5_estimate(agree, silent, 0U, &e);
	check(e.status == S5_ECHO_NONE && e.total == 0U, "estimate: no pings");
}

static void test_geometry(void)
{
	check_near(s5_lateral_mm(400.0, 90.0), 0.0, 1e-9, "geometry: 90 deg is straight ahead");
	check_near(s5_forward_mm(400.0, 90.0), 400.0 + S5_SENSOR_OFFSET_MM, 1e-9,
		   "geometry: forward at 90 deg includes the horn offset");
	check(s5_lateral_mm(400.0, 50.0) > 0.0, "geometry: 50 deg is left (+)");
	check(s5_lateral_mm(400.0, 130.0) < 0.0, "geometry: 130 deg is right (-)");
	check_near(s5_forward_mm(400.0 / cos(40.0 * DEG_TO_RAD) - S5_SENSOR_OFFSET_MM, 50.0), 400.0, 1e-9,
		   "geometry: flat face has constant forward distance");
}

static void run_box(world_t *p_world, double tolerance_mm, const char *p_name)
{
	static s5_scan_t	scan;
	char			name[96];
	double			width = p_world->left_mm - p_world->right_mm;
	double			offset = 0.5 * (p_world->left_mm + p_world->right_mm);

	check(s5_scan_run(&scan, world_measure, p_world), p_name);
	check(scan.b_valid, p_name);
	check(scan.edge[S5_LEFT].b_found && scan.edge[S5_RIGHT].b_found, p_name);
	(void)snprintf(name, sizeof(name), "%s: width", p_name);
	check_near(scan.width_mm, width, tolerance_mm, name);
	(void)snprintf(name, sizeof(name), "%s: centre offset", p_name);
	check_near(scan.offset_mm, offset, tolerance_mm, name);
	(void)snprintf(name, sizeof(name), "%s: distance", p_name);
	check_near(scan.centre.estimate.range_mm, p_world->depth_mm - S5_SENSOR_OFFSET_MM, 5.0, name);
	check(scan.coarse_count == (unsigned)S5_COARSE_POINTS, p_name);
	check(scan.fine_count[S5_LEFT] > 0U && scan.fine_count[S5_RIGHT] > 0U, p_name);
	printf("ok    %-34s width %6.1f mm (true %6.1f), %3u pings\n",
	       p_name, scan.width_mm, width, p_world->pings);
}

static void test_scans(void)
{
	static s5_scan_t	scan;
	world_t			world;

	world = box(400.0, 150.0, -150.0, 1500.0);
	run_box(&world, 10.0, "centred 300 mm box, far wall");

	world = box(400.0, 250.0, -50.0, 1500.0);
	run_box(&world, 10.0, "off-centre box");

	world = box(400.0, 30.0, -30.0, 0.0);
	run_box(&world, 10.0, "narrow 60 mm post, open space");

	world = box(400.0, 120.0, -90.0, 0.0);
	world.noise_mm = 4.0;
	world.stray_every = 4U;
	run_box(&world, 20.0, "noise and stray echoes");	/* 400 mm off: the 150 mm stop is sharper */

	/* edges at ~38.7 deg: raw range there is +112 mm over the 90 deg reading */
	world = box(400.0, 320.0, -320.0, 0.0);
	run_box(&world, 20.0, "wide 640 mm flat face");

	world = box(400.0, 2000.0, -2000.0, 0.0);
	check(s5_scan_run(&scan, world_measure, &world) && scan.b_valid, "wall: scanned");
	check(!scan.edge[S5_LEFT].b_found && !scan.edge[S5_RIGHT].b_found, "wall: edges beyond the sweep");
	check_near(scan.width_mm, 2.0 * 400.0 * tan((S5_ANGLE_CENTRE_DEG - S5_ANGLE_LEFT_DEG) * DEG_TO_RAD), 2.0, "wall: width is the sweep lower bound");

	world = box(400.0, -100.0, -300.0, 0.0);
	check(s5_scan_run(&scan, world_measure, &world) && !scan.b_valid, "nothing ahead: no width");

	world = box(400.0, 150.0, -150.0, 0.0);
	world.abort_after = 5U;
	check(!s5_scan_run(&scan, world_measure, &world), "abort: measure failure stops the scan");

	/* A beam cone hears the box past its edges: why S5_BEAM_CORRECTION_DEG exists */
	world = box(400.0, 150.0, -150.0, 0.0);
	world.cone_deg = 5.0;
	check(s5_scan_run(&scan, world_measure, &world) && scan.b_valid, "cone: scanned");
	{
		double by_edges = scan.edge[S5_LEFT].lateral_mm - scan.edge[S5_RIGHT].lateral_mm;

		check(by_edges > 300.0 + 20.0 &&
		      by_edges < 300.0 + 2.0 * 400.0 * tan(5.0 * DEG_TO_RAD) + 15.0,
		      "cone: edge angles overestimate by about the beam");
		/* the corners' ranges do not care how wide the beam is */
		check(scan.face.b_valid, "cone: face fitted");
		check_near(scan.width_mm, 300.0, 15.0, "cone: the face's width does not depend on the beam");
		printf("ok    %-34s edges %6.1f mm, face %6.1f mm (true  300.0)\n", "5 deg beam cone",
		       by_edges, scan.width_mm);
	}
}

static void run_placement(world_t *p_world, s5_position_t expected, bool b_bounded,
			  const char *p_name)
{
	static s5_scan_t scan;

	check(s5_scan_run(&scan, world_measure, p_world) && scan.b_valid, p_name);
	check(scan.placement.position == expected, p_name);
	check(scan.placement.b_bounded == b_bounded, p_name);
	printf("ok    %-34s left %4.1f deg, right %4.1f deg, difference %+5.1f\n", p_name,
	       scan.placement.left_deg, scan.placement.right_deg, scan.placement.difference_deg);
}

static void test_placement(void)
{
	static s5_scan_t	scan;
	world_t			world;
	double			expected;

	world = box(400.0, 150.0, -150.0, 1500.0);
	run_placement(&world, S5_POSITION_CENTRE, false, "placement: centred box");

	world = box(400.0, 155.0, -145.0, 0.0);		/* 5 mm off: inside tolerance */
	run_placement(&world, S5_POSITION_CENTRE, false, "placement: 5 mm off is centre");

	world = box(400.0, 250.0, -50.0, 0.0);
	run_placement(&world, S5_POSITION_LEFT, false, "placement: box to the left");
	expected = (atan(250.0 / 400.0) - atan(50.0 / 400.0)) / DEG_TO_RAD;
	world = box(400.0, 250.0, -50.0, 0.0);
	(void)s5_scan_run(&scan, world_measure, &world);
	check_near(scan.placement.difference_deg, expected, 1.0, "placement: left difference in degrees");

	world = box(400.0, 50.0, -250.0, 0.0);
	run_placement(&world, S5_POSITION_RIGHT, false, "placement: box to the right");

	world = box(400.0, 210.0, -90.0, 0.0);		/* 60 mm off: ~15 deg */
	run_placement(&world, S5_POSITION_LEFT, false, "placement: 60 mm off is left");

	world = box(400.0, 180.0, -120.0, 0.0);		/* 30 mm off: ~7.5 deg, inside 12 */
	run_placement(&world, S5_POSITION_CENTRE, false, "placement: 30 mm off is centre");

	world = box(400.0, 2000.0, -100.0, 0.0);
	run_placement(&world, S5_POSITION_LEFT, true, "placement: left edge beyond sweep");

	world = box(400.0, 100.0, -2000.0, 0.0);
	run_placement(&world, S5_POSITION_RIGHT, true, "placement: right edge beyond sweep");

	/* Left corner placed, right past the sweep but already reaching as far: the
	   right can only reach further, so that is what it is, never "centred" */
	world = box(400.0, 1050.0, -2000.0, 0.0);
	run_placement(&world, S5_POSITION_RIGHT, true, "placement: near-equal, one beyond");

	world = box(400.0, 2000.0, -2000.0, 0.0);
	run_placement(&world, S5_POSITION_UNKNOWN, true, "placement: wider than the sweep");

	/* An unfound edge is a minimum: it cannot make the other side "further" */
	memset(&scan, 0, sizeof(scan));
	scan.b_valid = true;
	scan.edge[S5_LEFT].b_found = true;
	scan.edge[S5_LEFT].angle_deg = 40.0;		/* 50 deg left */
	scan.edge[S5_RIGHT].b_found = false;
	scan.edge[S5_RIGHT].angle_deg = 110.0;		/* at least 20 deg right */
	s5_placement(&scan, &scan.placement);
	check(scan.placement.position == S5_POSITION_UNKNOWN, "placement: no LEFT from a minimum right edge");

	scan.b_valid = false;
	s5_placement(&scan, &scan.placement);
	check(scan.placement.position == S5_POSITION_UNKNOWN, "placement: no obstacle");
}

/* Gear backlash shifts every edge outward on the walk out and inward on the
   walk back; the mean of the two must cancel it. */
static void test_backlash(void)
{
	static s5_scan_t	scan;
	world_t			world;
	unsigned		side;

	world = box(400.0, 150.0, -150.0, 0.0);
	world.backlash_deg = 3.0;
	run_box(&world, 15.0, "3 deg backlash, centred box");
	world = box(400.0, 150.0, -150.0, 0.0);
	world.backlash_deg = 3.0;
	check(s5_scan_run(&scan, world_measure, &world) && scan.b_valid, "backlash: scanned");
	for (side = S5_LEFT; side <= S5_RIGHT; side++) {
		check(scan.edge[side].b_inward, "backlash: each edge found again walking inward");
		check_near(fabs(scan.edge[side].outward_deg - scan.edge[side].inward_deg),
			   2.0 * world.backlash_deg, 1.0, "backlash: the two walks differ by twice the lag");
	}
	check(scan.placement.position == S5_POSITION_CENTRE, "backlash: centred box still centre");
}

static void run_avoid(world_t *p_world, s5_avoid_side_t expected, const char *p_name)
{
	static s5_scan_t scan;

	check(s5_scan_run(&scan, world_measure, p_world) && scan.b_valid, p_name);
	check(scan.avoid.side == expected, p_name);
	printf("ok    %-34s pass %-5s shift L %5.0f R %5.0f mm, beside L %6.0f R %6.0f mm\n", p_name,
	       (scan.avoid.side == S5_AVOID_LEFT) ? "left" :
	       ((scan.avoid.side == S5_AVOID_RIGHT) ? "right" : "none"),
	       scan.avoid.shift_mm[S5_LEFT], scan.avoid.shift_mm[S5_RIGHT],
	       scan.avoid.clear_mm[S5_LEFT], scan.avoid.clear_mm[S5_RIGHT]);
}

static void test_avoidance(void)
{
	static s5_scan_t	scan;
	world_t			world;
	double			margin = S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM;

	/* The obstacle blocks the left: pass right, and vice versa */
	world = box(400.0, 250.0, -50.0, 0.0);
	run_avoid(&world, S5_AVOID_RIGHT, "avoid: obstacle to the left");
	world = box(400.0, 250.0, -50.0, 0.0);
	(void)s5_scan_run(&scan, world_measure, &world);
	check_near(scan.avoid.shift_mm[S5_RIGHT], 50.0 + margin, 10.0, "avoid: right shift clears the edge");
	check_near(scan.avoid.shift_mm[S5_LEFT], 250.0 + margin, 10.0, "avoid: left shift clears the edge");

	world = box(400.0, 50.0, -250.0, 0.0);
	run_avoid(&world, S5_AVOID_LEFT, "avoid: obstacle to the right");

	world = box(400.0, 150.0, -150.0, 1500.0);
	run_avoid(&world, S5_DEFAULT_AVOID, "avoid: centred takes the default");

	/* Obstacle to the left, but a second object 250 mm beyond it on the
	   right leaves no room there: pass left instead */
	world = box(400.0, 150.0, -50.0, 0.0);
	world.other_depth_mm = 650.0;
	world.other_left_mm = -150.0;
	world.other_right_mm = -400.0;
	run_avoid(&world, S5_AVOID_LEFT, "avoid: no room on the right");

	/* The same object far out to the side is no obstacle to passing */
	world = box(400.0, 150.0, -50.0, 0.0);
	world.other_depth_mm = 650.0;
	world.other_left_mm = -600.0;
	world.other_right_mm = -900.0;
	run_avoid(&world, S5_AVOID_RIGHT, "avoid: room beside a far object");

	/* ... and one far behind the obstacle does not count at all */
	world = box(400.0, 150.0, -50.0, 0.0);
	world.other_depth_mm = 400.0 + S5_PASS_LOOKAHEAD_MM + 200.0;
	world.other_left_mm = -150.0;
	world.other_right_mm = -400.0;
	run_avoid(&world, S5_AVOID_RIGHT, "avoid: object far behind ignored");

	world = box(400.0, 2000.0, -2000.0, 0.0);
	run_avoid(&world, S5_AVOID_NONE, "avoid: wall, no way around");

	scan.b_valid = false;
	s5_avoidance(&scan, &scan.avoid);
	check(scan.avoid.side == S5_AVOID_NONE, "avoid: nothing ahead");
}

static void check_plan_geometry(const s5_detour_plan_t *p_plan, const char *p_name);

/* The face, from an ideal ray: tilt, distance, corners and the side passed on */
static void test_face(void)
{
	static s5_scan_t	scan;
	static const double	yaws[] = {0.0, 10.0, -10.0, 20.0, -20.0, 30.0, -30.0};
	s5_detour_plan_t	plan;
	unsigned		i;

	for (i = 0U; i < sizeof(yaws) / sizeof(yaws[0]); i++) {
		yawed_t	yawed = {yaws[i], 100.0, 0.0};
		double	face = 180.0 * cos(yaws[i] * DEG_TO_RAD);
		double	heading = yaws[i] * DEG_TO_RAD;	/* the robot's, square to the face */
		double	start_y, servo_y;
		double	end_y;

		check(s5_scan_run(&scan, yawed_measure, &yawed) && scan.b_valid, "face: scanned");
		check(scan.face.b_valid, "face: fitted");
		check_near(scan.face.tilt_deg, -yaws[i], 4.0, "face: tilt");
		check_near(scan.face.distance_mm, face, 3.0, "face: distance");
		check_near(scan.face.width_mm, 200.0, 12.0, "face: width");
		check_near(scan.width_mm, scan.face.width_mm, 1e-9, "face: reported width");
		check(scan.face.b_corner[S5_LEFT] && scan.face.b_corner[S5_RIGHT], "face: both corners placed");
		check(scan.placement.position == S5_POSITION_CENTRE, "face: centred");
		/* Centred: the closer side, or the default when it is square */
		check(scan.avoid.side == ((yaws[i] > 0.0) ? S5_AVOID_RIGHT :
					  (yaws[i] < 0.0) ? S5_AVOID_LEFT : S5_DEFAULT_AVOID), "face: side");

		/* The plan is the square-on one, in the face's frame */
		check(s5_detour_plan(&scan, &plan), "face: plan");
		check_near(plan.tilt_deg, scan.face.tilt_deg, 1e-9, "face: plan's frame");
		start_y = -S5_IR_AHEAD_MM * sin(heading) ;
		check_near(plan.start_y_mm, start_y, 6.0, "face: sensors start on the line");
		servo_y = start_y + S5_SERVO_AHEAD_MM * sin(heading);
		end_y = servo_y + ((plan.side == S5_AVOID_LEFT) ? 100.0 : -100.0);
		check_near(plan.edge_mm, end_y, 12.0, "face: the end being passed");
		check_near(fabs(plan.shift_mm - plan.edge_mm), S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM, 1e-9,
			   "face: passes it by half the robot and the margin");
		check_near(plan.face_mm, face + S5_SERVO_AHEAD_MM * cos(heading), 3.0, "face: where the face is");
		/* The moves reach that y, and stay short of the face */
		check_plan_geometry(&plan, "face: plan geometry");
		printf("ok    %-34s tilt %+5.1f deg, width %5.1f mm, pass %s, turn %+5.1f, back %3.0f\n",
		       "face: tilted box", scan.face.tilt_deg, scan.face.width_mm,
		       (plan.side == S5_AVOID_LEFT) ? "left" : "right", plan.turn_deg, plan.backup_mm);
	}

	/* The angle alone: coarse readings only, no fine scan and no decision */
	for (i = 0U; i < sizeof(yaws) / sizeof(yaws[0]); i++) {
		yawed_t yawed = {yaws[i], 100.0, 0.0};

		check(s5_tilt_scan(&scan, yawed_measure, &yawed) && scan.face.b_valid, "tilt scan: fitted");
		check_near(scan.face.tilt_deg, -yaws[i], 6.0, "tilt scan: tilt");
		check(scan.fine_count[S5_LEFT] == 0U && scan.fine_count[S5_RIGHT] == 0U, "tilt scan: no fine points");
		check(scan.avoid.side == S5_AVOID_NONE, "tilt scan: decides nothing");
	}

	/* Off centre, tilted or not: pass on the side it reaches less */
	for (i = 0U; i < 3U; i++) {
		yawed_t yawed = {yaws[i], 120.0, 60.0};

		check(s5_scan_run(&scan, yawed_measure, &yawed) && scan.face.b_valid, "face: reaching left");
		check(scan.placement.position == S5_POSITION_LEFT, "face: it reaches left");
		check_near(scan.face.offset_mm, 60.0, 15.0, "face: by how much");
		check(scan.avoid.side == S5_AVOID_RIGHT, "face: pass right");
		yawed.centre_mm = -60.0;
		check(s5_scan_run(&scan, yawed_measure, &yawed) && scan.face.b_valid, "face: reaching right");
		check(scan.placement.position == S5_POSITION_RIGHT, "face: it reaches right");
		check(scan.avoid.side == S5_AVOID_LEFT, "face: pass left");
	}

	/* A wall has no corners to place: no way round, as ever */
	{
		world_t world = box(180.0, 2000.0, -2000.0, 0.0);

		check(s5_scan_run(&scan, world_measure, &world) && scan.face.b_valid, "face: wall");
		check(!scan.face.b_corner[S5_LEFT] && !scan.face.b_corner[S5_RIGHT], "face: wall has no corners");
		check(scan.avoid.side == S5_AVOID_NONE, "face: no way round a wall");
	}

	/* Too little rise to tell: the edge angles decide */
	{
		world_t world = box(400.0, 250.0, -50.0, 0.0);

		check(s5_scan_run(&scan, world_measure, &world) && !scan.face.b_valid, "face: no fit");
		check(scan.avoid.side == S5_AVOID_RIGHT, "face: edge angles decide");
		world = box(400.0, 30.0, -30.0, 0.0);
		check(s5_scan_run(&scan, world_measure, &world) && !scan.face.b_valid, "face: no fit on a thin post");
	}

	/* Something nearer off to one side is not what the robot stopped for */
	{
		world_t world = box(180.0, 100.0, -100.0, 0.0);

		world.other_depth_mm = 60.0;
		world.other_left_mm = 400.0;
		world.other_right_mm = 150.0;
		check(s5_scan_run(&scan, world_measure, &world) && scan.b_valid, "face: neighbour");
		if (scan.face.b_valid) {
			check_near(scan.face.distance_mm, 180.0, 3.0, "face: fitted to what is ahead");
		}
	}

	scan.b_valid = false;
	s5_face_fit(&scan, &scan.face);
	check(!scan.face.b_valid, "face: nothing ahead");
}

/* Fill a scan's readings from a logged one */
static void face_fill(s5_scan_t *p_scan, const face_scan_t *p_logged)
{
	unsigned i;

	memset(p_scan, 0, sizeof(*p_scan));
	p_scan->b_valid = true;
	for (i = 0U; i < p_logged->coarse_count; i++) {
		s5_point_t *p_point = &p_scan->coarse[p_scan->coarse_count++];
		bool b_echo = p_logged->p_coarse[i].range_mm >= 0.0;

		p_point->angle_deg = p_logged->p_coarse[i].angle_deg;
		p_point->estimate.status = b_echo ? S5_ECHO_OK : S5_ECHO_NONE;
		p_point->estimate.range_mm = b_echo ? p_logged->p_coarse[i].range_mm : 0.0;
	}
	for (i = 0U; i < p_logged->out_count; i++) {
		unsigned side = (p_logged->p_out[i].angle_deg < S5_ANGLE_CENTRE_DEG) ? (unsigned)S5_LEFT : (unsigned)S5_RIGHT;
		s5_point_t *p_point = &p_scan->fine[side][p_scan->fine_count[side]++];
		bool b_echo = p_logged->p_out[i].range_mm >= 0.0;

		p_point->angle_deg = p_logged->p_out[i].angle_deg;
		p_point->estimate.status = b_echo ? S5_ECHO_OK : S5_ECHO_NONE;
		p_point->estimate.range_mm = b_echo ? p_logged->p_out[i].range_mm : 0.0;
		p_scan->fine_outward[side] = p_scan->fine_count[side];
	}
}

/* Scans from the robot, as logged: the fit on real echoes */
static void test_face_logged(void)
{
	static s5_scan_t	scan;
	unsigned		i;
	char			name[64];

	for (i = 0U; i < sizeof(face_scans) / sizeof(face_scans[0]); i++) {
		const face_scan_t *p_logged = &face_scans[i];

		face_fill(&scan, p_logged);
		s5_face_fit(&scan, &scan.face);
		(void)snprintf(name, sizeof(name), "logged %s", p_logged->p_name);
		check(scan.face.b_valid, name);
		if (p_logged->b_200mm_box) {
			/* The coarse sweep alone is enough to correct the angle */
			static s5_scan_t coarse;
			face_scan_t only = *p_logged;

			only.out_count = 0U;
			face_fill(&coarse, &only);
			s5_face_fit(&coarse, &coarse.face);
			check(coarse.face.b_valid, name);
			check_near(coarse.face.tilt_deg, scan.face.tilt_deg, 3.0, name);
		}
		check_near(scan.face.tilt_deg, p_logged->tilt_deg, 0.2, name);
		check_near(scan.face.distance_mm, p_logged->distance_mm, 0.2, name);
		check_near(scan.face.left_mm, p_logged->left_mm, 0.2, name);
		check_near(scan.face.right_mm, p_logged->right_mm, 0.2, name);
		if (p_logged->b_200mm_box) {
			check_near(scan.face.width_mm, 200.0, 40.0, name);
			/* Robots this tilted left can still be on the rise at 160 deg:
			   that corner stays unplaced, and the right is not passed */
			check(scan.face.b_corner[S5_LEFT], name);
		}
		printf("ok    %-34s tilt %+6.1f deg, %3.0f mm away, reaches %3.0f left %3.0f right, width %3.0f mm\n",
		       name, scan.face.tilt_deg, scan.face.distance_mm, scan.face.left_mm, scan.face.right_mm,
		       scan.face.width_mm);
	}
}

/* Drive this far at this heading rate (deg per mm), noting every 6 mm as the
   task does at line speed.  Returns the path reached. */
static double curve_drive(s5_curve_t *p_curve, double path_mm, double *p_heading_deg, double mm, double deg_per_mm)
{
	double end = path_mm + mm;

	while (path_mm < end) {
		path_mm += 6.0;
		*p_heading_deg += 6.0 * deg_per_mm;
		s5_curve_note(p_curve, path_mm, *p_heading_deg);
	}
	return path_mm;
}

static void test_curve(void)
{
	static s5_curve_t	curve;
	double			path = 0.0;
	double			heading = 0.0;
	unsigned		i;

	s5_curve_reset(&curve);
	check(!s5_curve_recent(&curve), "curve: not at the start");
	path = curve_drive(&curve, path, &heading, 1000.0, 0.0);
	check(!s5_curve_recent(&curve), "curve: not on a straight");
	check_near(curve.swing_deg, 0.0, 1e-9, "curve: a straight has no swing");

	/* The follower's zigzag: +/- 8 deg every 100 mm */
	for (i = 0U; i < 20U; i++) {
		path = curve_drive(&curve, path, &heading, 50.0, 0.16);
		path = curve_drive(&curve, path, &heading, 50.0, -0.16);
	}
	check(!s5_curve_recent(&curve), "curve: a zigzag is not a curve");

	/* 90 deg over 150 mm, then straight: still recent well after, then not */
	path = curve_drive(&curve, path, &heading, 150.0, 0.6);
	check(s5_curve_recent(&curve), "curve: seen");
	path = curve_drive(&curve, path, &heading, S5_CURVE_HOLD_MM, 0.0);
	check(s5_curve_recent(&curve), "curve: remembered past the end of the curve");
	path = curve_drive(&curve, path, &heading, (double)S5_CURVE_WINDOW_MM + S5_CURVE_HOLD_MM + 12.0, 0.0);
	check(!s5_curve_recent(&curve), "curve: forgotten");
	check_near(curve.swing_deg, 0.0, 1e-9, "curve: old samples dropped");

	/* Either direction */
	path = curve_drive(&curve, path, &heading, 150.0, -0.6);
	check(s5_curve_recent(&curve), "curve: to the right too");
	path = curve_drive(&curve, path, &heading, 2.0 * ((double)S5_CURVE_WINDOW_MM + S5_CURVE_HOLD_MM), 0.0);
	check(!s5_curve_recent(&curve), "curve: right forgotten");

	/* Gentle: 15 deg within the window is below the threshold, 30 is not */
	path = curve_drive(&curve, path, &heading, 1500.0, 0.05);
	check(!s5_curve_recent(&curve), "curve: too gentle");
	path = curve_drive(&curve, path, &heading, 600.0, 0.1);
	check(s5_curve_recent(&curve), "curve: a long sweep counts");
	path = curve_drive(&curve, path, &heading, 2.0 * ((double)S5_CURVE_WINDOW_MM + S5_CURVE_HOLD_MM), 0.0);
	check(!s5_curve_recent(&curve), "curve: sweep forgotten");

	/* A pivot on the spot adds no path but is still a turn */
	heading += 40.0;
	s5_curve_note(&curve, path, heading);
	check(s5_curve_recent(&curve), "curve: pivot counts");
	path = curve_drive(&curve, path, &heading, 2.0 * ((double)S5_CURVE_WINDOW_MM + S5_CURVE_HOLD_MM), 0.0);

	/* Standing still does not age the curve or lose samples */
	path = curve_drive(&curve, path, &heading, 150.0, 0.6);
	for (i = 0U; i < 1000U; i++) {
		s5_curve_note(&curve, path, heading);
	}
	check(s5_curve_recent(&curve), "curve: still recent after standing still");
	check(curve.swing_deg >= S5_CURVE_MIN_DEG, "curve: standing still keeps the samples");

	/* Marked after a rejoin: however straight the odometry looks */
	s5_curve_reset(&curve);
	s5_curve_mark(&curve, 5000.0);
	check(s5_curve_recent(&curve), "curve: marked");
	path = curve_drive(&curve, 5000.0, &heading, S5_CURVE_HOLD_MM - 12.0, 0.0);
	check(s5_curve_recent(&curve), "curve: marked, within the hold");
	(void)curve_drive(&curve, path, &heading, 24.0, 0.0);
	check(!s5_curve_recent(&curve), "curve: marked, past the hold");
}

/* A scan taken on hardware (logs.txt, 5 Nov 2026) after a curve, the robot about
   32 deg left of square to the box.  The edge angles made it 0 mm wide; the robot
   then turned 51 deg right, which from there is still nearly at the face.
   Each row holds from that angle upwards. */
typedef struct {
	int	angle_deg;
	double	range_mm;
} replay_t;

static const replay_t replay_scan[] = {
	{0, 515.0}, {20, 893.0}, {30, 601.0}, {40, 579.0}, {50, 588.0}, {60, 575.0},
	{70, 515.0}, {80, 515.0}, {84, 563.0}, {85, 168.0}, {89, 163.0}, {90, 159.0},
	{100, 121.0}, {106, 125.0}, {107, 111.0}, {120, 111.0}, {140, 115.0},
	{150, 118.0}, {160, 117.0}
};

static bool replay_measure(void *p_context, int angle_deg, s5_estimate_t *p_estimate)
{
	unsigned	*p_calls = (unsigned *)p_context;
	double		range[5];
	bool		echo[5] = {true, true, true, true, true};
	unsigned	i, row = 0U;

	for (i = 0U; i < sizeof(replay_scan) / sizeof(replay_scan[0]); i++) {
		if (replay_scan[i].angle_deg <= angle_deg) {
			row = i;
		}
	}
	/* The reading straight ahead, taken first, was 172 mm */
	for (i = 0U; i < 5U; i++) {
		range[i] = ((*p_calls == 0U) && (angle_deg == 90)) ? 172.0 : replay_scan[row].range_mm;
	}
	(*p_calls)++;
	s5_estimate(range, echo, 5U, p_estimate);
	return true;
}

static void test_sloping_surface(void)
{
	static s5_scan_t	scan;
	s5_detour_plan_t	plan;
	unsigned		calls = 0U;
	const s5_point_t	*p_point;
	unsigned		i;

	check(s5_scan_run(&scan, replay_measure, &calls) && scan.b_valid, "slope: scanned");
	check(scan.edge[S5_LEFT].b_found, "slope: left end of the face found");
	check_near(scan.edge[S5_LEFT].outward_deg, 84.5, 1.0, "slope: left end at 84.5 deg");
	check(!scan.edge[S5_RIGHT].b_found, "slope: right end heard past the sweep");
	check(scan.edge[S5_RIGHT].last_object_deg == S5_ANGLE_RIGHT_DEG, "slope: followed to the sweep limit");
	/* every point on the way there is the obstacle */
	for (i = 0U; i < scan.fine_outward[S5_RIGHT]; i++) {
		p_point = &scan.fine[S5_RIGHT][i];
		check(p_point->b_object, "slope: right walk stays on the face");
	}
	check(scan.face.b_valid, "slope: face fitted");
	check_near(scan.face.tilt_deg, -32.0, 3.0, "slope: tilted about 32 deg");
	check_near(scan.face.width_mm, 181.0, 10.0, "slope: about its true 200 mm wide");
	check(scan.face.b_corner[S5_RIGHT], "slope: right corner placed though past the sweep");
	check(scan.placement.position == S5_POSITION_LEFT, "slope: it reaches left");
	check(scan.avoid.side == S5_AVOID_RIGHT, "slope: pass on the side it reaches less");
	check(s5_detour_plan(&scan, &plan), "slope: plan");
	/* From 32 deg left of square, this is much more than the 51 deg it turned */
	check(fabs(plan.turn_deg + plan.tilt_deg) >= 70.0, "slope: turns well clear of the face");
	printf("ok    %-34s tilt %+5.1f deg, width %3.0f mm, pass %s, turn %+.0f deg from where it stood\n",
	       "slope: replayed hardware scan", scan.face.tilt_deg, scan.face.width_mm,
	       (scan.avoid.side == S5_AVOID_LEFT) ? "left" : "right", plan.turn_deg + plan.tilt_deg);
}

/* com4_live.log, 5 Nov 2026 15:04, after the L bend and a 27 deg turn to square up.
   Each row holds from that angle upwards. */
static const replay_t bend_scan[] = {
	{0, 1230.0}, {30, 1195.0}, {40, 1163.0}, {48, 1151.0}, {49, 1131.0}, {50, 181.0}, {51, 168.0},
	{60, 162.0}, {70, 158.0}, {80, 145.0}, {100, 159.0}, {110, 165.0}, {120, 172.0}, {122, 168.0},
	{125, 181.0}, {126, 178.0}, {127, 386.0}, {128, 571.0}, {130, 2238.0}, {140, 585.0},
	{150, 641.0}, {160, 1423.0}
};
static double bend_128_mm;

static bool bend_measure(void *p_context, int angle_deg, s5_estimate_t *p_estimate)
{
	double		range[5];
	bool		echo[5] = {true, true, true, true, true};
	unsigned	i, row = 0U;

	(void)p_context;
	for (i = 0U; i < sizeof(bend_scan) / sizeof(bend_scan[0]); i++) {
		if (bend_scan[i].angle_deg <= angle_deg) {
			row = i;
		}
	}
	for (i = 0U; i < 5U; i++) {
		range[i] = (angle_deg == 128) ? bend_128_mm : bend_scan[row].range_mm;
	}
	s5_estimate(range, echo, 5U, p_estimate);
	return true;
}

static void test_bend_scan(void)
{
	static s5_scan_t scan;

	bend_128_mm = 571.0;
	check(s5_scan_run(&scan, bend_measure, NULL) && scan.face.b_valid, "bend: scanned");
	check(scan.placement.position == S5_POSITION_CENTRE, "bend: centred");
	check(scan.avoid.b_room[S5_RIGHT], "bend: the lone corner echo leaves room on the right");
	scan.squared_deg = -23.6;
	s5_avoidance(&scan, &scan.avoid);
	check(scan.avoid.side == S5_AVOID_RIGHT, "bend: passes on the side that was closer");

	/* The next reading agrees: something is really there */
	bend_128_mm = 390.0;
	check(s5_scan_run(&scan, bend_measure, NULL) && scan.face.b_valid, "bend: real object scanned");
	check(!scan.avoid.b_room[S5_RIGHT], "bend: a real object still blocks the right");
	scan.squared_deg = -23.6;
	s5_avoidance(&scan, &scan.avoid);
	check(scan.avoid.side == S5_AVOID_LEFT, "bend: so it goes the other way");
}

/* A box in front of a wall at about its own range still ends where the range jumps */
static void test_edge_still_ends(void)
{
	static s5_scan_t	scan;
	world_t			world = box(180.0, 100.0, -100.0, 400.0);

	check(s5_scan_run(&scan, world_measure, &world) && scan.b_valid, "edge: box before a wall, scanned");
	check(scan.edge[S5_LEFT].b_found && scan.edge[S5_RIGHT].b_found, "edge: both ends found");
	check_near(scan.width_mm, 200.0, 12.0, "edge: width unchanged by the surface rule");
}

/* ---- detour ---- */

static void check_plan_geometry(const s5_detour_plan_t *p_plan, const char *p_name)
{
	double front_reach = hypot(S5_FRONT_MM, S5_ROBOT_HALF_WIDTH_MM);
	double heading = -p_plan->tilt_deg * DEG_TO_RAD;	/* the robot's at the start */

	check(fabs(p_plan->turn_deg) >= S5_OUT_MIN_DEG - 1e-9 &&
	      fabs(p_plan->turn_deg) <= S5_OUT_MAX_DEG + 1e-9, p_name);
	check((p_plan->turn_deg > 0.0) == (p_plan->side == S5_AVOID_LEFT), p_name);
	check((p_plan->shift_mm > 0.0) == (p_plan->side == S5_AVOID_LEFT), p_name);
	/* the diagonal ends shift_mm out, at out_x_mm ... */
	check_near(p_plan->start_y_mm - p_plan->backup_mm * sin(heading) +
		   p_plan->diagonal_mm * sin(p_plan->turn_deg * DEG_TO_RAD), p_plan->shift_mm, 0.01, p_name);
	check_near(p_plan->diagonal_mm * cos(p_plan->turn_deg * DEG_TO_RAD) - p_plan->backup_mm * cos(heading),
		   p_plan->out_x_mm, 0.01, p_name);
	/* ... and the front corners never come within the margin of the face */
	check(p_plan->out_x_mm + front_reach + S5_TURN_MARGIN_MM <= p_plan->face_mm + 0.01, p_name);
}

static void test_detour_plan(void)
{
	static s5_scan_t	scan;
	s5_detour_plan_t	plan;
	world_t			world;
	double			margin = S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM;

	/* Obstacle reaching right, at the 150 mm stop: out to the left */
	world = box(180.0, 50.0, -150.0, 0.0);
	(void)s5_scan_run(&scan, world_measure, &world);
	check(s5_detour_plan(&scan, &plan) && plan.b_ok, "plan: pass left");
	check(plan.side == S5_AVOID_LEFT, "plan: pass left side");
	check_near(plan.shift_mm, 50.0 + margin, 10.0, "plan: left shift clears the edge");
	check_near(plan.face_mm, 180.0 + S5_SERVO_AHEAD_MM, 5.0, "plan: face from the axle");
	check_near(plan.backup_mm, 0.0, 1e-9, "plan: room to turn, no reversing");
	check_plan_geometry(&plan, "plan: left geometry");
	printf("ok    %-34s back %3.0f turn %+5.1f deg diagonal %3.0f mm -> x %4.0f y %+4.0f\n",
	       "plan: pass left", plan.backup_mm, plan.turn_deg, plan.diagonal_mm,
	       plan.out_x_mm, plan.shift_mm);

	world = box(180.0, 150.0, -50.0, 0.0);
	(void)s5_scan_run(&scan, world_measure, &world);
	check(s5_detour_plan(&scan, &plan) && (plan.side == S5_AVOID_RIGHT), "plan: pass right");
	check_near(plan.shift_mm, -(50.0 + margin), 10.0, "plan: right shift clears the edge");
	check_plan_geometry(&plan, "plan: right geometry");

	/* Face nearly touching: reverse first, steepest turn */
	scan.centre_forward_mm = 60.0;
	scan.face.b_valid = false;
	check(s5_detour_plan(&scan, &plan), "plan: close face");
	check(plan.backup_mm > 0.0, "plan: close face reverses");
	check_near(fabs(plan.turn_deg), S5_OUT_MAX_DEG, 1e-9, "plan: close face, steepest turn");
	check_plan_geometry(&plan, "plan: close face geometry");
	printf("ok    %-34s back %3.0f turn %+5.1f deg diagonal %3.0f mm -> x %4.0f y %+4.0f\n",
	       "plan: close face", plan.backup_mm, plan.turn_deg, plan.diagonal_mm,
	       plan.out_x_mm, plan.shift_mm);

	/* More reversing than allowed, no room, no side: stay stopped */
	scan.centre_forward_mm = 180.0;
	scan.avoid.shift_mm[S5_RIGHT] = 1000.0;
	check(!s5_detour_plan(&scan, &plan) && !plan.b_ok, "plan: too much reversing refused");
	scan.avoid.shift_mm[S5_RIGHT] = 165.0;
	scan.avoid.b_room[S5_RIGHT] = false;
	check(!s5_detour_plan(&scan, &plan), "plan: no room refused");
	scan.avoid.b_room[S5_RIGHT] = true;
	scan.avoid.side = S5_AVOID_NONE;
	check(!s5_detour_plan(&scan, &plan), "plan: no side refused");
}

/* A box face_mm..face_mm + depth_mm along x beside the robot, driving
   S5_PASS_STEP_MM between looks with an ideal side ray.  Every
   miss_every-th echo from it is lost; far_mm > 0 puts a wall that far
   beyond its side, heard whenever the box is not. */
static bool simulate_pass(const s5_detour_plan_t *p_plan, double depth_mm, unsigned miss_every,
			  double far_mm, s5_pass_t *p_pass)
{
	double		look = (double)S5_SIDE_LOOK_DEG * DEG_TO_RAD;
	double		x = p_plan->out_x_mm;
	double		y = p_plan->shift_mm;
	double		gap = fabs(y - p_plan->edge_mm);
	unsigned	count;

	s5_pass_init(p_pass, p_plan);
	for (count = 1U; x <= p_plan->out_x_mm + S5_PASS_MAX_MM; count++) {
		double	crossing = x + S5_SERVO_AHEAD_MM + gap / tan(look);
		bool	b_box = (crossing >= p_plan->face_mm) && (crossing <= p_plan->face_mm + depth_mm);
		bool	b_echo = b_box && ((miss_every == 0U) || (count % miss_every != 0U));
		/* Driver ranges start at the transducer, not the servo axis. */
		double	range = gap / sin(look) - S5_SENSOR_OFFSET_MM;

		if (!b_echo && (far_mm > 0.0)) {
			b_echo = true;
			range = (gap + far_mm) / sin(look) - S5_SENSOR_OFFSET_MM;
		}
		if (s5_pass_observe(p_pass, p_plan, x, y, b_echo, range)) {
			return true;
		}
		x += S5_PASS_STEP_MM;
	}
	return false;
}

static void check_pass(const s5_detour_plan_t *p_plan, double depth_mm, unsigned miss_every,
		       double far_mm, const char *p_name)
{
	s5_pass_t	pass;
	double		rear_reach = hypot(S5_REAR_MM, S5_ROBOT_HALF_WIDTH_MM);
	double		end = p_plan->face_mm + depth_mm;
	double		turn_in;
	double		past;
	double		gap = fabs(p_plan->shift_mm - p_plan->edge_mm);
	double		back = S5_RETURN_DEG * DEG_TO_RAD;

	check(simulate_pass(p_plan, depth_mm, miss_every, far_mm, &pass), p_name);
	/* the end is never estimated short of the true end ... */
	turn_in = s5_pass_clear_x(&pass) + S5_PASS_TRIM_MM - rear_reach - S5_PASS_MARGIN_MM;
	check(turn_in >= end - 1e-9, p_name);
	/* ... nor more than a step further than needed */
	check(turn_in <= end + S5_PASS_STEP_MM + 1e-9, p_name);
	/* Worst case, the diagonal got no further out than planned.  With the
	   forward trim, what keeps the obstacle's rear corner off the robot is
	   the sideways gap: the rear corners turning in place ... */
	past = s5_pass_clear_x(&pass) - end;
	check(hypot(past, gap) >= rear_reach, p_name);
	/* ... and the body driving back in along the return heading. */
	check(past * sin(back) + gap * cos(back) >= S5_ROBOT_HALF_WIDTH_MM, p_name);
	printf("ok    %-34s end %4.0f mm, estimate %4.0f mm, %u echoes, turn in at x %4.0f, return clears by %3.0f mm\n",
	       p_name, end, fmax(pass.rear_mm, pass.end_mm), pass.seen, s5_pass_clear_x(&pass),
	       past * sin(back) + gap * cos(back) - S5_ROBOT_HALF_WIDTH_MM);
}

static void test_detour_pass(void)
{
	static s5_scan_t	scan;
	s5_detour_plan_t	plan;
	s5_pass_t		pass;
	world_t			world;

	world = box(180.0, 50.0, -150.0, 0.0);
	(void)s5_scan_run(&scan, world_measure, &world);
	(void)s5_detour_plan(&scan, &plan);
	check_pass(&plan, 200.0, 0U, 0.0, "pass: 200 mm deep box");
	check_pass(&plan, 20.0, 0U, 0.0, "pass: thin board");
	check_pass(&plan, 300.0, 3U, 0.0, "pass: single missed echoes");
	check_pass(&plan, 200.0, 0U, 400.0, "pass: wall far beyond it");
	check(!simulate_pass(&plan, 5000.0, 0U, 0.0, &pass), "pass: never ends, gives up");

	/* Locate the actual side hit, including the horn's sensor offset. */
	s5_pass_init(&pass, &plan);
	(void)s5_pass_observe(&pass, &plan, 300.0, plan.shift_mm, true, 100.0);
	check_near(pass.rear_mm, 300.0 + S5_SERVO_AHEAD_MM +
		   (100.0 + S5_SENSOR_OFFSET_MM) * cos(S5_SIDE_LOOK_DEG * DEG_TO_RAD),
		   1e-9, "pass: side hit includes sensor offset");

	check_near(s5_return_length(165.0), 165.0 / sin(S5_RETURN_DEG * DEG_TO_RAD) + S5_RETURN_EXTRA_MM,
		   1e-9, "return: length");
	check_near(s5_return_length(-165.0), s5_return_length(165.0), 1e-9, "return: either side");
	check(s5_line_expected(0.0, 0.0), "line: on it");
	check(s5_line_expected(60.0, -S5_RETURN_DEG), "line: sensors reach it from the left");
	check(s5_line_expected(-60.0, S5_RETURN_DEG), "line: sensors reach it from the right");
	check(!s5_line_expected(300.0, -S5_RETURN_DEG), "line: too far out to be it");
}

int main(void)
{
	if (S5_BEAM_CORRECTION_DEG != 0.0) {
		printf("build with -DS5_BEAM_CORRECTION_DEG=0.0 (see the top of this file)\n");
		return 1;
	}
	test_estimate();
	test_geometry();
	test_scans();
	test_placement();
	test_backlash();
	test_avoidance();
	test_face();
	test_face_logged();
	test_sloping_surface();
	test_bend_scan();
	test_edge_still_ends();
	test_curve();
	test_detour_plan();
	test_detour_pass();
	printf("%u/%u checks passed\n", checks - failures, checks);
	return (failures == 0U) ? 0 : 1;
}
