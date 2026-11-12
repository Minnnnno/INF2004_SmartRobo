/*
 *	telemetry_map.c
 *	Subsystem 1 -- robot_status_t snapshot -> status frame values.
 *
 *	Slot meanings follow docs/API.md "Snapshot for other buddies", with the
 *	Tribunal 1 packing decisions:
 *	  line     values[0] error -1/0/+1, values[1] trackable (> 0.5)
 *	  barcode  values[0] tf_barcode_t, values[1] valid (> 0.5)
 *	  terrain  values[0] heading rad, [1] pitch rad, [2] hump m,
 *	           [3] validity bits 0..2 (TF_TV_*) | terrain state << 3
 *	  obstacle values[0] distance m, [2] width m, [3] TF_OF_* flags
 */

#include "telemetry_map.h"
#include <math.h>

#define TM_PI	3.14159265358979323846

/* TRUE for a finite double.  NaN fails both comparisons.  (Avoids the
   isfinite macro, which warns under -Wconversion on some host libcs.) */
static bool tm_finite(double v)
{
	return (v > -1.0e300) && (v < 1.0e300);
}

/* Round half away from zero; non-finite -> 0.  The double is limited first
   so the conversion is always defined; tf_pack() does the real clamp. */
static int32_t to_int(double v)
{
	if (!tm_finite(v)) {
		return 0;
	}
	if (v > 1.0e6) {
		v = 1.0e6;
	} else if (v < -1.0e6) {
		v = -1.0e6;
	}
	return (int32_t)lround(v);
}

static int32_t saturate(uint32_t v, uint32_t max)
{
	return (int32_t)((v > max) ? max : v);
}

/* Degrees wrapped into [lo, lo + 360). */
static int32_t wrap_deg(double rad, int32_t lo)
{
	int32_t d;

	if (!tm_finite(rad)) {
		return lo;
	}
	d = to_int(fmod(rad * 180.0 / TM_PI, 360.0));
	while (d < lo) {
		d += 360;
	}
	while (d >= lo + 360) {
		d -= 360;
	}
	return d;
}

static uint32_t bits_of(const subsystem_status_t *s, unsigned slot)
{
	int32_t v = to_int(s->values[slot]);

	return (s->available && (v > 0)) ? (uint32_t)v : 0U;
}

void tm_fill(const robot_status_t *st, const tm_frame_info_t *info,
	     int32_t values[TF_FIELD_COUNT])
{
	const motion_status_t	*m = &st->motion;
	uint32_t		flags;

	values[TF_F_VERSION]   = (int32_t)TF_LAYOUT_VERSION;
	values[TF_F_SEQ]       = (int32_t)(info->seq & 0xFFU);
	values[TF_F_UPTIME_DS] = (int32_t)((uint32_t)to_int(info->uptime_s * 10.0) & 0xFFFFU);

	flags = (m->stationary ? TF_MF_STATIONARY : 0U) |
		(m->feedback_trusted ? TF_MF_TRUSTED : 0U) |
		(m->inhibited ? TF_MF_INHIBITED : 0U) |
		(m->hold ? TF_MF_HOLD : 0U);
	values[TF_F_M_STATE]   = (int32_t)m->state;
	values[TF_F_M_MODE]    = (int32_t)m->mode;
	values[TF_F_M_FAULT]   = (int32_t)m->fault;
	values[TF_F_M_FLAGS]   = (int32_t)flags;
	values[TF_F_OWNER]     = st->owner_valid ? (int32_t)st->owner : TF_OWNER_NONE;
	values[TF_F_CMD_ID]    = (int32_t)(m->id & 0xFFU);
	values[TF_F_SPEED_L]   = to_int(m->speed[0] * 1000.0);
	values[TF_F_SPEED_R]   = to_int(m->speed[1] * 1000.0);
	values[TF_F_CENTRE_MM] = to_int(m->centre * 1000.0);
	values[TF_F_HEADING]   = wrap_deg(m->heading, -180);

	values[TF_F_LINE_AVAIL] = st->line.available ? 1 : 0;
	values[TF_F_LINE_ERR]   = st->line.available ? to_int(st->line.values[0]) : 0;
	values[TF_F_LINE_TRACK] = (st->line.available && (st->line.values[1] > 0.5)) ? 1 : 0;
	values[TF_F_BC_VALID]   = (st->barcode.available && (st->barcode.values[1] > 0.5)) ? 1 : 0;
	values[TF_F_BC_CMD]     = st->barcode.available ? to_int(st->barcode.values[0]) : 0;

	flags = bits_of(&st->terrain, 3U);
	values[TF_F_T_VALID]   = (int32_t)(flags & 0x7U);
	values[TF_F_T_STATE]   = (int32_t)((flags >> 3) & 0x7U);
	values[TF_F_PITCH]     = ((flags & TF_TV_PITCH) != 0U) ?
				 to_int(st->terrain.values[1] * 180.0 / TM_PI) : 0;
	values[TF_F_T_HEADING] = ((flags & TF_TV_HEADING) != 0U) ?
				 wrap_deg(st->terrain.values[0], 0) : 0;
	values[TF_F_HUMP_MM]   = ((flags & TF_TV_HUMP) != 0U) ?
				 to_int(st->terrain.values[2] * 1000.0) : 0;

	values[TF_F_O_FLAGS]    = (int32_t)(bits_of(&st->obstacle, 3U) & 0x1FU);
	values[TF_F_O_AVAIL]    = st->obstacle.available ? 1 : 0;
	values[TF_F_O_DIST_CM]  = st->obstacle.available ? to_int(st->obstacle.values[0] * 100.0) : 0;
	values[TF_F_O_WIDTH_CM] = st->obstacle.available ? to_int(st->obstacle.values[2] * 100.0) : 0;

	values[TF_F_REJECTED] = saturate(info->rejected, 7U);
	values[TF_F_DROPPED]  = saturate(info->dropped, 7U);
}

_Static_assert((SHUTDOWN == TF_ENC_STATE_MAX) && (F_SETUP_TIMEOUT == TF_ENC_FAULT_MAX) &&
	       (FB_RESTART_LATCH == TF_EF_CAUSE_MAX),
	       "encoder frame ranges must match the motion enums");

void tm_fill_encoder(const robot_status_t *st, uint32_t seq, double now_s,
		     tf_encoder_t *e)
{
	const motion_status_t	*m = &st->motion;
	double			t = m->encoder_sample_time;
	double			age;

	e->seq = (uint8_t)(seq & 0xFFU);
	e->reliable = m->encoder.reliable;
	e->trusted = m->feedback_trusted;
	e->cause = (uint8_t)m->feedback_cause;
	e->state = (uint8_t)m->state;
	e->fault = (uint8_t)m->fault;
	e->count[0] = m->encoder.count[0];
	e->count[1] = m->encoder.count[1];
	e->invalid[0] = m->encoder.invalid[0];
	e->invalid[1] = m->encoder.invalid[1];

	e->sample_ms = 0U;
	e->age_ms = (uint16_t)TF_ENC_AGE_UNKNOWN;
	e->fresh = false;
	if (!tm_finite(t) || (t < 0.0) || !tm_finite(now_s)) {
		return;
	}
	e->sample_ms = (uint32_t)fmod(t * 1000.0, 4294967296.0);
	age = (now_s - t) * 1000.0;
	if ((age < 0.0) || (age >= (double)TF_ENC_AGE_UNKNOWN)) {
		return;		/* sample from the future, or very old */
	}
	e->age_ms = (uint16_t)to_int(age);	/* nearest ms; 65535 still means old */
	e->fresh = (e->age_ms <= TF_ENC_FRESH_MS);
}

static bool line_ok(const robot_status_t *s)
{
	return s->line.available && (s->line.values[1] > 0.5);
}

static bool barcode_ok(const robot_status_t *s)
{
	return s->barcode.available && (s->barcode.values[1] > 0.5);
}

uint8_t tm_edge_events(const robot_status_t *prev, const robot_status_t *cur)
{
	uint32_t ev = 0U;

	if ((cur->motion.fault != F_NONE) && (prev->motion.fault == F_NONE)) {
		ev |= TF_EV_MOTION_FAULT;
	}
	if (line_ok(prev) && !line_ok(cur)) {
		ev |= TF_EV_LINE_LOST;
	}
	/* A new decode: valid now, and either not valid before or a different
	   instruction.  B3 clears validity between barcodes, so the same
	   instruction twice in a row is still reported. */
	if (barcode_ok(cur) && (!barcode_ok(prev) ||
	    (to_int(cur->barcode.values[0]) != to_int(prev->barcode.values[0])))) {
		ev |= TF_EV_BARCODE;
	}
	return (uint8_t)ev;
}
