/*
 *	obstacle_scan.c
 *	Subsystem 5 -- obstacle width, position and which side to pass it on,
 *	from a servo-swept ultrasonic ranger.
 *	Pure computation; see obstacle_scan.h.
 */

#include "obstacle_scan.h"
#include <math.h>
#include <string.h>

#define S5_DEG_TO_RAD	(0.017453292519943295)

_Static_assert(S5_SAMPLES_PER_POINT + S5_RETRY_SAMPLES <= S5_MAX_SAMPLES,
	       "S5_MAX_SAMPLES too small for one point");
_Static_assert(S5_SAMPLES_PER_POINT >= 3U, "take at least three pings per angle");

static double s5_bearing_rad(double angle_deg)
{
	return ((double)S5_ANGLE_CENTRE_DEG - angle_deg) * S5_DEG_TO_RAD; /* + left */
}

double s5_forward_mm(double range_mm, double angle_deg)
{
	return (range_mm + S5_SENSOR_OFFSET_MM) * cos(s5_bearing_rad(angle_deg));
}

double s5_lateral_mm(double range_mm, double angle_deg)
{
	return (range_mm + S5_SENSOR_OFFSET_MM) * sin(s5_bearing_rad(angle_deg));
}

void s5_estimate(const double range_mm[], const bool echo[], unsigned count,
		 s5_estimate_t *p_out)
{
	unsigned	i, j;
	unsigned	best = count;
	unsigned	best_support = 0U;
	double		best_spread = 0.0;
	double		sum = 0.0;

	p_out->status = S5_ECHO_NONE;
	p_out->range_mm = 0.0;
	p_out->support = 0U;
	p_out->valid = 0U;
	p_out->total = count;

	for (i = 0U; i < count; i++) {
		unsigned	support = 0U;
		double		spread = 0.0;

		if (!echo[i]) {
			continue;
		}
		p_out->valid++;
		for (j = 0U; j < count; j++) {
			double gap = fabs(range_mm[j] - range_mm[i]);
			if (echo[j] && (gap <= S5_CLUSTER_TOL_MM)) {
				support++;
				spread += gap;
			}
		}
		/* Most agreeing pings wins, then the tightest group, then the
		   nearer echo (a multipath echo only ever travels further). */
		if ((best == count) || (support > best_support) ||
		    ((support == best_support) && ((spread < best_spread) ||
		     ((spread == best_spread) && (range_mm[i] < range_mm[best]))))) {
			best = i;
			best_support = support;
			best_spread = spread;
		}
	}
	if (best == count) {
		return;			/* no echo at all */
	}

	for (j = 0U; j < count; j++) {
		if (echo[j] && (fabs(range_mm[j] - range_mm[best]) <= S5_CLUSTER_TOL_MM)) {
			sum += range_mm[j];
		}
	}
	p_out->range_mm = sum / (double)best_support;
	p_out->support = best_support;

	if ((count - p_out->valid) * 2U > count) {
		p_out->status = S5_ECHO_NONE;		/* mostly silent */
	} else if ((best_support >= S5_MIN_AGREE) && (best_support * 2U >= count)) {
		p_out->status = S5_ECHO_OK;
	} else {
		p_out->status = S5_ECHO_UNSTABLE;
	}
}

bool s5_estimate_usable(const s5_estimate_t *p_estimate)
{
	return (p_estimate->status == S5_ECHO_OK) ||
	       ((p_estimate->status == S5_ECHO_UNSTABLE) &&
		(p_estimate->support > p_estimate->total - p_estimate->valid));
}

bool s5_point_is_object(const s5_estimate_t *p_estimate, int angle_deg,
			double centre_forward_mm)
{
	return s5_estimate_usable(p_estimate) &&
	       (fabs(s5_forward_mm(p_estimate->range_mm, (double)angle_deg) -
		     centre_forward_mm) <= S5_EDGE_THRESHOLD_FRACTION * centre_forward_mm);
}

void s5_sample_point(s5_ping_fn ping, void *p_context, s5_estimate_t *p_out)
{
	double		range_mm[S5_MAX_SAMPLES];
	bool		echo[S5_MAX_SAMPLES];
	unsigned	count = 0U;

	while (count < S5_SAMPLES_PER_POINT) {
		range_mm[count] = 0.0;
		echo[count] = ping(p_context, &range_mm[count]);
		count++;
	}
	s5_estimate(range_mm, echo, count, p_out);
	if (p_out->status != S5_ECHO_UNSTABLE) {
		return;
	}

	/* Disagreement (often half the beam on the edge): vote again with more pings */
	while (count < S5_SAMPLES_PER_POINT + S5_RETRY_SAMPLES) {
		range_mm[count] = 0.0;
		echo[count] = ping(p_context, &range_mm[count]);
		count++;
	}
	s5_estimate(range_mm, echo, count, p_out);
}

static bool s5_measure_point(s5_point_t *p_point, int angle_deg, double centre_forward_mm,
			     s5_measure_fn measure, void *p_context)
{
	p_point->angle_deg = angle_deg;
	if (!measure(p_context, angle_deg, &p_point->estimate)) {
		return false;
	}
	p_point->b_object = s5_point_is_object(&p_point->estimate, angle_deg,
					       centre_forward_mm);
	return true;
}

/* Walk back inward from last_deg (clear space, where the outward walk
   left the servo) until the obstacle is seen twice in a row.  The servo now
   reaches every angle moving the other way.  No further in than straight
   ahead or S5_INWARD_MAX_STEPS points: backlash can put the edge inside
   the coarse edge angle, so that is no limit. */
static bool s5_walk_inward(s5_scan_t *p_scan, unsigned side, int last_deg,
			   s5_measure_fn measure, void *p_context)
{
	s5_edge_t	*p_edge = &p_scan->edge[side];
	int		step = (side == S5_LEFT) ? S5_FINE_STEP_DEG : -S5_FINE_STEP_DEG;
	int		angle = last_deg;
	int		clear_deg = last_deg;
	int		first_object_deg = last_deg;
	unsigned	run = 0U;
	unsigned	taken;

	for (taken = 0U; taken < S5_INWARD_MAX_STEPS; taken++) {
		s5_point_t *p_point = &p_scan->fine[side][p_scan->fine_count[side]];

		angle += step;
		if ((step > 0) ? (angle > S5_ANGLE_CENTRE_DEG) : (angle < S5_ANGLE_CENTRE_DEG)) {
			break;
		}
		if (!s5_measure_point(p_point, angle, p_scan->centre_forward_mm,
				      measure, p_context)) {
			return false;
		}
		p_scan->fine_count[side]++;

		if (!p_point->b_object) {
			run = 0U;
			clear_deg = angle;
		} else if (++run == 1U) {
			first_object_deg = angle;
		} else {
			p_edge->b_inward = true;
			p_edge->inward_deg = 0.5 * (double)(clear_deg + first_object_deg);
			break;
		}
	}
	return true;
}

/* Walk outward 1 deg at a time from start_deg (the outermost coarse point
   still on the obstacle).  The edge is passed once two clear points in a
   row are seen, so one dropped ping cannot end the obstacle early.  Then
   walk back inward and average the two crossings. */
static bool s5_find_edge(s5_scan_t *p_scan, unsigned side, int start_deg,
			 double start_range_mm, s5_measure_fn measure, void *p_context)
{
	s5_edge_t	*p_edge = &p_scan->edge[side];
	int		step = (side == S5_LEFT) ? -S5_FINE_STEP_DEG : S5_FINE_STEP_DEG;
	int		limit = (side == S5_LEFT) ? S5_ANGLE_LEFT_DEG : S5_ANGLE_RIGHT_DEG;
	int		angle;
	int		last_deg = start_deg;
	int		clear_deg = start_deg;
	bool		b_pending = false;
	double		edge_deg;

	p_edge->b_found = false;
	p_edge->b_inward = false;
	p_edge->last_object_deg = start_deg;
	p_edge->range_mm = start_range_mm;

	for (angle = start_deg + step;
	     (step < 0) ? (angle >= limit) : (angle <= limit); angle += step) {
		s5_point_t *p_point = &p_scan->fine[side][p_scan->fine_count[side]];

		if (!s5_measure_point(p_point, angle, p_scan->centre_forward_mm,
				      measure, p_context)) {
			return false;
		}
		p_scan->fine_count[side]++;
		last_deg = angle;

		if (p_point->b_object) {
			p_edge->last_object_deg = angle;
			p_edge->range_mm = p_point->estimate.range_mm;
			b_pending = false;
		} else if (!b_pending) {
			b_pending = true;
			clear_deg = angle;
		} else {
			p_edge->b_found = true;
			break;
		}
	}
	if (b_pending) {
		p_edge->b_found = true;		/* also: one clear point at the scan limit */
	}

	/* Halfway between the last obstacle and the first clear angle */
	p_edge->first_clear_deg = p_edge->b_found ? clear_deg : p_edge->last_object_deg;
	p_edge->outward_deg = 0.5 * (double)(p_edge->last_object_deg + p_edge->first_clear_deg);
	p_edge->inward_deg = p_edge->outward_deg;
	p_scan->fine_outward[side] = p_scan->fine_count[side];
	if (p_edge->b_found &&
	    !s5_walk_inward(p_scan, side, last_deg, measure, p_context)) {
		return false;
	}

	/* Mean of both crossings, moved inward (towards 90 deg) by the beam
	   correction */
	edge_deg = 0.5 * (p_edge->outward_deg + p_edge->inward_deg);
	if (side == S5_LEFT) {
		edge_deg = fmin(edge_deg + S5_BEAM_CORRECTION_DEG, (double)S5_ANGLE_CENTRE_DEG);
	} else {
		edge_deg = fmax(edge_deg - S5_BEAM_CORRECTION_DEG, (double)S5_ANGLE_CENTRE_DEG);
	}
	p_edge->angle_deg = edge_deg;
	/* Project along the face, not the edge's own range: near an edge the
	   echo comes from the corner, off the corrected bearing, and reads long. */
	p_edge->lateral_mm = p_scan->centre_forward_mm * tan(s5_bearing_rad(edge_deg));
	return true;
}

/* Outermost coarse point, walking out from 90 deg, before the first clear one */
static void s5_coarse_edge(const s5_scan_t *p_scan, unsigned side,
			   int *p_angle_deg, double *p_range_mm)
{
	int step = (side == S5_LEFT) ? -S5_COARSE_STEP_DEG : S5_COARSE_STEP_DEG;
	int angle;

	*p_angle_deg = S5_ANGLE_CENTRE_DEG;
	*p_range_mm = p_scan->centre.estimate.range_mm;
	for (angle = S5_ANGLE_CENTRE_DEG + step;
	     (angle >= S5_ANGLE_LEFT_DEG) && (angle <= S5_ANGLE_RIGHT_DEG); angle += step) {
		const s5_point_t *p_point =
			&p_scan->coarse[(angle - S5_ANGLE_LEFT_DEG) / S5_COARSE_STEP_DEG];
		if (!p_point->b_object) {
			break;
		}
		*p_angle_deg = angle;
		*p_range_mm = p_point->estimate.range_mm;
	}
}

bool s5_scan_run(s5_scan_t *p_scan, s5_measure_fn measure, void *p_context)
{
	unsigned	side;
	int		angle;

	memset(p_scan, 0, sizeof(*p_scan));

	/* Reference: straight ahead */
	p_scan->centre.angle_deg = S5_ANGLE_CENTRE_DEG;
	if (!measure(p_context, S5_ANGLE_CENTRE_DEG, &p_scan->centre.estimate)) {
		return false;
	}
	if (!s5_estimate_usable(&p_scan->centre.estimate)) {
		return true;			/* nothing straight ahead to measure */
	}
	p_scan->centre.b_object = true;
	p_scan->centre_forward_mm = s5_forward_mm(p_scan->centre.estimate.range_mm,
						  (double)S5_ANGLE_CENTRE_DEG);

	/* Coarse sweep, left to right */
	for (angle = S5_ANGLE_LEFT_DEG; angle <= S5_ANGLE_RIGHT_DEG; angle += S5_COARSE_STEP_DEG) {
		if (!s5_measure_point(&p_scan->coarse[p_scan->coarse_count], angle,
				      p_scan->centre_forward_mm, measure, p_context)) {
			return false;
		}
		p_scan->coarse_count++;
	}

	/* Fine scan near each coarse edge */
	for (side = S5_LEFT; side <= S5_RIGHT; side++) {
		int	start_deg;
		double	start_range_mm;

		s5_coarse_edge(p_scan, side, &start_deg, &start_range_mm);
		if (!s5_find_edge(p_scan, side, start_deg, start_range_mm, measure, p_context)) {
			return false;
		}
	}

	p_scan->width_mm = p_scan->edge[S5_LEFT].lateral_mm - p_scan->edge[S5_RIGHT].lateral_mm;
	p_scan->offset_mm = 0.5 * (p_scan->edge[S5_LEFT].lateral_mm +
				   p_scan->edge[S5_RIGHT].lateral_mm);
	p_scan->b_valid = true;
	s5_placement(p_scan, &p_scan->placement);
	s5_avoidance(p_scan, &p_scan->avoid);
	return true;
}

void s5_placement(const s5_scan_t *p_scan, s5_placement_t *p_out)
{
	const s5_edge_t	*p_left = &p_scan->edge[S5_LEFT];
	const s5_edge_t	*p_right = &p_scan->edge[S5_RIGHT];

	memset(p_out, 0, sizeof(*p_out));
	p_out->position = S5_POSITION_UNKNOWN;
	if (!p_scan->b_valid) {
		return;
	}
	p_out->left_deg = (double)S5_ANGLE_CENTRE_DEG - p_left->angle_deg;
	p_out->right_deg = p_right->angle_deg - (double)S5_ANGLE_CENTRE_DEG;
	p_out->difference_deg = p_out->left_deg - p_out->right_deg;
	p_out->b_bounded = !p_left->b_found || !p_right->b_found;

	/* A side whose edge was not found is at least its angle, so it can only
	   make the obstacle lean further towards that side, never less. */
	if ((p_out->difference_deg > S5_CENTRE_TOLERANCE_DEG) && p_right->b_found) {
		p_out->position = S5_POSITION_LEFT;
	} else if ((p_out->difference_deg < -S5_CENTRE_TOLERANCE_DEG) && p_left->b_found) {
		p_out->position = S5_POSITION_RIGHT;
	} else if (!p_out->b_bounded) {
		p_out->position = S5_POSITION_CENTRE;
	}
}

/* Farthest range among the obstacle's own points */
static double s5_object_range(const s5_scan_t *p_scan)
{
	double		farthest = p_scan->centre.estimate.range_mm;
	unsigned	side, i;

	for (i = 0U; i < p_scan->coarse_count; i++) {
		if (p_scan->coarse[i].b_object) {
			farthest = fmax(farthest, p_scan->coarse[i].estimate.range_mm);
		}
	}
	for (side = S5_LEFT; side <= S5_RIGHT; side++) {
		for (i = 0U; i < p_scan->fine_count[side]; i++) {
			if (p_scan->fine[side][i].b_object) {
				farthest = fmax(farthest, p_scan->fine[side][i].estimate.range_mm);
			}
		}
	}
	return farthest;
}

/* Is this point something else beside the obstacle, beyond the given edge? */
static bool s5_beside(const s5_scan_t *p_scan, unsigned side, const s5_point_t *p_point,
		      double object_range_mm)
{
	const s5_edge_t	*p_edge = &p_scan->edge[side];
	bool		b_outside = (side == S5_LEFT) ?
				    (p_point->angle_deg <= p_edge->first_clear_deg) :
				    (p_point->angle_deg >= p_edge->first_clear_deg);

	return b_outside && !p_point->b_object && s5_estimate_usable(&p_point->estimate) &&
	       (p_point->estimate.range_mm > object_range_mm + S5_SPILL_RANGE_MM) &&
	       (s5_forward_mm(p_point->estimate.range_mm, (double)p_point->angle_deg) <=
		p_scan->centre_forward_mm + S5_PASS_LOOKAHEAD_MM);
}

/* Nearest echo beside the obstacle on one side, mm from the centre line */
static double s5_side_clear(const s5_scan_t *p_scan, unsigned side, double object_range_mm)
{
	double		nearest = INFINITY;
	double		sign = (side == S5_LEFT) ? 1.0 : -1.0;
	unsigned	i;

	for (i = 0U; i < p_scan->coarse_count; i++) {
		const s5_point_t *p_point = &p_scan->coarse[i];
		if (s5_beside(p_scan, side, p_point, object_range_mm)) {
			nearest = fmin(nearest, sign * s5_lateral_mm(p_point->estimate.range_mm,
								     (double)p_point->angle_deg));
		}
	}
	for (i = 0U; i < p_scan->fine_count[side]; i++) {
		const s5_point_t *p_point = &p_scan->fine[side][i];
		if (s5_beside(p_scan, side, p_point, object_range_mm)) {
			nearest = fmin(nearest, sign * s5_lateral_mm(p_point->estimate.range_mm,
								     (double)p_point->angle_deg));
		}
	}
	return nearest;
}

void s5_avoidance(const s5_scan_t *p_scan, s5_avoid_t *p_out)
{
	s5_avoid_side_t	order[2];
	double		object_range_mm;
	unsigned	side, i;

	memset(p_out, 0, sizeof(*p_out));
	p_out->side = S5_AVOID_NONE;
	p_out->clear_mm[S5_LEFT] = INFINITY;
	p_out->clear_mm[S5_RIGHT] = INFINITY;
	if (!p_scan->b_valid) {
		return;
	}

	object_range_mm = s5_object_range(p_scan);
	p_out->shift_mm[S5_LEFT] = fmax(0.0, p_scan->edge[S5_LEFT].lateral_mm +
					S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM);
	p_out->shift_mm[S5_RIGHT] = fmax(0.0, -p_scan->edge[S5_RIGHT].lateral_mm +
					 S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM);
	for (side = S5_LEFT; side <= S5_RIGHT; side++) {
		p_out->clear_mm[side] = s5_side_clear(p_scan, side, object_range_mm);
		/* After the shift, the robot's far side still keeps the margin */
		p_out->b_room[side] = p_scan->edge[side].b_found &&
			(p_out->clear_mm[side] >= p_out->shift_mm[side] +
			 S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM);
	}

	if (p_scan->placement.position == S5_POSITION_LEFT) {
		order[0] = S5_AVOID_RIGHT;
	} else if (p_scan->placement.position == S5_POSITION_RIGHT) {
		order[0] = S5_AVOID_LEFT;
	} else {
		order[0] = S5_DEFAULT_AVOID;
	}
	order[1] = (order[0] == S5_AVOID_LEFT) ? S5_AVOID_RIGHT : S5_AVOID_LEFT;
	for (i = 0U; i < 2U; i++) {
		if (p_out->b_room[(order[i] == S5_AVOID_LEFT) ? S5_LEFT : S5_RIGHT]) {
			p_out->side = order[i];
			return;
		}
	}
}
