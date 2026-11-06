/*
 *	detour_plan.c
 *	Subsystem 5 -- the geometry of driving round a scanned obstacle.
 *	See detour_plan.h for the frame and the steps.
 */

#include "detour_plan.h"
#include <math.h>

#define DEG_TO_RAD	(0.017453292519943295)

static double side_sign(s5_avoid_side_t side)
{
	return (side == S5_AVOID_LEFT) ? 1.0 : -1.0;
}

/* Every pose of the outward move keeps the robot's front corners (they
   sweep a circle of front_reach round the centre) S5_TURN_MARGIN_MM short
   of the obstacle's face, so turning out, the diagonal and turning back
   parallel all happen before the face: the robot is beside the obstacle
   only once it is clear of it sideways. */
bool s5_detour_plan(const s5_scan_t *p_scan, s5_detour_plan_t *p_plan)
{
	unsigned	index;
	double		sign;
	double		shift;
	double		travel;
	double		heading;
	double		front_reach;
	double		room;
	double		tan_max;
	double		angle_deg;
	bool		b_face = p_scan->face.b_valid;

	p_plan->b_ok = false;
	p_plan->side = S5_AVOID_NONE;
	p_plan->shift_mm = 0.0;
	p_plan->edge_mm = 0.0;
	p_plan->face_mm = 0.0;
	p_plan->backup_mm = 0.0;
	p_plan->turn_deg = 0.0;
	p_plan->diagonal_mm = 0.0;
	p_plan->out_x_mm = 0.0;
	p_plan->tilt_deg = 0.0;
	p_plan->start_y_mm = 0.0;
	if (!p_scan->b_valid || (p_scan->avoid.side == S5_AVOID_NONE)) {
		return false;
	}
	index = (p_scan->avoid.side == S5_AVOID_LEFT) ? (unsigned)S5_LEFT : (unsigned)S5_RIGHT;
	if (!p_scan->avoid.b_room[index] ||
	    !(b_face ? p_scan->face.b_corner[index] : p_scan->edge[index].b_found)) {
		return false;
	}
	sign = side_sign(p_scan->avoid.side);
	/* The line sensors were on the line, ahead of the axle, at the tilt it stopped
	   with.  Squaring up turns about the axle, so it does not change this. */
	p_plan->start_y_mm = S5_IR_AHEAD_MM * sin(s5_start_tilt_deg(p_scan) * DEG_TO_RAD);
	shift = p_scan->avoid.shift_mm[index];
	p_plan->side = p_scan->avoid.side;
	if (b_face) {
		p_plan->tilt_deg = p_scan->face.tilt_deg;
		p_plan->edge_mm = p_plan->start_y_mm +
			sign * (shift - S5_ROBOT_HALF_WIDTH_MM - S5_PASS_MARGIN_MM);
		p_plan->face_mm = p_scan->face.distance_mm +
			S5_SERVO_AHEAD_MM * cos(p_plan->tilt_deg * DEG_TO_RAD);
	} else {
		p_plan->edge_mm = p_plan->start_y_mm + p_scan->edge[index].lateral_mm;
		p_plan->face_mm = p_scan->centre_forward_mm + S5_SERVO_AHEAD_MM;
	}
	p_plan->shift_mm = p_plan->start_y_mm + sign * shift;
	heading = -p_plan->tilt_deg * DEG_TO_RAD;

	/* Forward travel the centre may make before the front corners reach
	   the margin; back up (along the robot's own heading, so that changes
	   the sideways move too) until the steepest turn fits in it. */
	front_reach = hypot(S5_FRONT_MM, S5_ROBOT_HALF_WIDTH_MM);
	room = p_plan->face_mm - front_reach - S5_TURN_MARGIN_MM;
	tan_max = tan(S5_OUT_MAX_DEG * DEG_TO_RAD);
	p_plan->backup_mm = fmax(0.0, (shift / tan_max - room) /
				 (cos(heading) - sign * sin(heading) / tan_max));
	if (p_plan->backup_mm > S5_BACKUP_MAX_MM) {
		return false;
	}
	room += p_plan->backup_mm * cos(heading);
	travel = shift + sign * p_plan->backup_mm * sin(heading);

	angle_deg = atan2(travel, room) / DEG_TO_RAD;
	angle_deg = fmax(S5_OUT_MIN_DEG, fmin(S5_OUT_MAX_DEG, angle_deg));
	p_plan->turn_deg = sign * angle_deg;
	p_plan->diagonal_mm = travel / sin(angle_deg * DEG_TO_RAD);
	p_plan->out_x_mm = travel / tan(angle_deg * DEG_TO_RAD) - p_plan->backup_mm * cos(heading);
	p_plan->b_ok = true;
	return true;
}

void s5_pass_init(s5_pass_t *p_pass, const s5_detour_plan_t *p_plan)
{
	p_pass->rear_mm = p_plan->face_mm;	/* depth unknown: at least its face */
	p_pass->end_mm = p_plan->face_mm;
	p_pass->gap_mm = side_sign(p_plan->side) * (p_plan->shift_mm - p_plan->edge_mm);
	p_pass->clear_run = 0U;
	p_pass->seen = 0U;
	p_pass->b_ended = false;
}

/* An echo near enough sideways is the obstacle (or something as close,
   which is no safer) and moves its known end to where the beam met it.
   A clear look counts only when the beam crosses the obstacle's side
   beyond that end; looking at a part already known to be there, silence
   is just a missed echo.  The end lies between the last echo and the first
   clear look after it, so the first clear look is taken as the end.  The
   beam's cone keeps hearing the obstacle a little after its axis has passed
   the end, which only moves that later: safe. */
bool s5_pass_observe(s5_pass_t *p_pass, const s5_detour_plan_t *p_plan,
		     double x_mm, double y_mm, bool b_echo, double range_mm)
{
	double look = (double)S5_SIDE_LOOK_DEG * DEG_TO_RAD;
	double servo_x = x_mm + S5_SERVO_AHEAD_MM;
	double axis_range_mm = range_mm + S5_SENSOR_OFFSET_MM;
	double crossing;

	p_pass->gap_mm = side_sign(p_plan->side) * (y_mm - p_plan->edge_mm);
	if (b_echo && (axis_range_mm * sin(look) <= p_pass->gap_mm + S5_SIDE_SEE_MM)) {
		p_pass->rear_mm = fmax(p_pass->rear_mm, servo_x + axis_range_mm * cos(look));
		p_pass->clear_run = 0U;
		p_pass->seen++;
	} else {
		crossing = servo_x + p_pass->gap_mm / tan(look);
		if (crossing > p_pass->rear_mm) {
			if (p_pass->clear_run == 0U) {
				p_pass->end_mm = crossing;
			}
			p_pass->clear_run++;
		}
	}
	if (p_pass->clear_run >= S5_PASS_CONFIRM) {
		p_pass->b_ended = true;
	}
	return p_pass->b_ended;
}

/* Turning in place, the rear corners sweep a circle of rear_reach. */
double s5_pass_clear_x(const s5_pass_t *p_pass)
{
	return fmax(p_pass->rear_mm, p_pass->end_mm) +
	       hypot(S5_REAR_MM, S5_ROBOT_HALF_WIDTH_MM) + S5_PASS_MARGIN_MM - S5_PASS_TRIM_MM;
}

double s5_return_length(double y_mm)
{
	return fabs(y_mm) / sin(S5_RETURN_DEG * DEG_TO_RAD) + S5_RETURN_EXTRA_MM;
}

bool s5_line_expected(double y_mm, double heading_deg)
{
	return fabs(y_mm + S5_IR_AHEAD_MM * sin(heading_deg * DEG_TO_RAD)) <= S5_LINE_WINDOW_MM;
}
