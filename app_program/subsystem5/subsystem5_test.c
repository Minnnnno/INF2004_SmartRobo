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
	run_box(&world, 10.0, "noise and stray echoes");

	/* edges at ~38.7 deg: raw range there is +112 mm over the 90 deg reading */
	world = box(400.0, 320.0, -320.0, 0.0);
	run_box(&world, 12.0, "wide 640 mm flat face");

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
	check(scan.width_mm > 300.0 + 20.0 &&
	      scan.width_mm < 300.0 + 2.0 * 400.0 * tan(5.0 * DEG_TO_RAD) + 15.0,
	      "cone: width overestimated by about the beam");
	printf("ok    %-34s width %6.1f mm (true  300.0) uncorrected\n", "5 deg beam cone", scan.width_mm);
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

	/* left edge ~69 deg, right beyond the 70 deg limit: right is only a minimum */
	world = box(400.0, 1050.0, -2000.0, 0.0);
	run_placement(&world, S5_POSITION_UNKNOWN, true, "placement: near-equal, one beyond");

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
	run_box(&world, 10.0, "3 deg backlash, centred box");
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

/* ---- detour ---- */

static void check_plan_geometry(const s5_detour_plan_t *p_plan, const char *p_name)
{
	double front_reach = hypot(S5_FRONT_MM, S5_ROBOT_HALF_WIDTH_MM);

	check(fabs(p_plan->turn_deg) >= S5_OUT_MIN_DEG - 1e-9 &&
	      fabs(p_plan->turn_deg) <= S5_OUT_MAX_DEG + 1e-9, p_name);
	check((p_plan->turn_deg > 0.0) == (p_plan->side == S5_AVOID_LEFT), p_name);
	check((p_plan->shift_mm > 0.0) == (p_plan->side == S5_AVOID_LEFT), p_name);
	/* the diagonal ends shift_mm out, at out_x_mm ... */
	check_near(p_plan->diagonal_mm * sin(fabs(p_plan->turn_deg) * DEG_TO_RAD),
		   fabs(p_plan->shift_mm), 0.01, p_name);
	check_near(p_plan->diagonal_mm * cos(p_plan->turn_deg * DEG_TO_RAD) - p_plan->backup_mm,
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
	test_detour_plan();
	test_detour_pass();
	printf("%u/%u checks passed\n", checks - failures, checks);
	return (failures == 0U) ? 0 : 1;
}
