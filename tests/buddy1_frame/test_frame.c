/*
 *	test_frame.c
 *	Host tests for Buddy 1's binary formats (include/telemetry_frame.h)
 *	and the snapshot mapping (app_program/subsystem1/telemetry_map.c).
 *
 *	Tribunal 1 test plan, "Packed telemetry and dashboard decoding":
 *	every decoded value matches the value written, negative values are
 *	restored, the frame is 21 bytes, out-of-range values are clamped
 *	rather than wrapped.  Also prints the JSON vs packed size comparison.
 *
 *	Build: see tests/buddy1_frame/README.md, or ROBO_HOST_TESTS=ON.
 */

#include "telemetry_frame.h"
#include "telemetry_map.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned checks, failures;

#define CHECK(cond, ...) do {						\
	checks++;							\
	if (!(cond)) {							\
		failures++;						\
		printf("FAIL %s:%d: ", __FILE__, __LINE__);		\
		printf(__VA_ARGS__);					\
		printf("\n");						\
	}								\
} while (0)

#define PI 3.14159265358979323846

/* Golden frame, shared with tools/dashboard.py --selftest.  If the layout
   changes on purpose, bump TF_LAYOUT_VERSION and regenerate both. */
static const char GOLDEN_HEX[] =
	"10730394808dcbe69475b02a7e69c002a77201ec00";

static void golden_values(int32_t v[TF_FIELD_COUNT])
{
	memset(v, 0, sizeof(int32_t) * TF_FIELD_COUNT);
	v[TF_F_VERSION] = (int32_t)TF_LAYOUT_VERSION;
	v[TF_F_SEQ] = 7;
	v[TF_F_UPTIME_DS] = 12345;
	v[TF_F_M_STATE] = 2;		/* DRIVING */
	v[TF_F_M_MODE] = 2;		/* WHEEL_SPEED_TRACKING */
	v[TF_F_M_FAULT] = 0;
	v[TF_F_M_FLAGS] = TF_MF_TRUSTED;
	v[TF_F_OWNER] = 1;		/* SOURCE_LINE */
	v[TF_F_CMD_ID] = 0xB9;
	v[TF_F_SPEED_L] = -1;
	v[TF_F_SPEED_R] = 80;
	v[TF_F_CENTRE_MM] = -1234;
	v[TF_F_HEADING] = -170;
	v[TF_F_LINE_AVAIL] = 1;
	v[TF_F_LINE_ERR] = -1;
	v[TF_F_LINE_TRACK] = 1;
	v[TF_F_BC_VALID] = 1;
	v[TF_F_BC_CMD] = TF_BC_UTURN;
	v[TF_F_T_VALID] = TF_TV_PITCH | TF_TV_HUMP;
	v[TF_F_T_STATE] = 3;
	v[TF_F_PITCH] = -12;
	v[TF_F_T_HEADING] = 0;
	v[TF_F_HUMP_MM] = 42;
	v[TF_F_O_FLAGS] = TF_OF_FIRST_EDGE | TF_OF_SECOND_EDGE | TF_OF_WIDTH_VALID;
	v[TF_F_O_AVAIL] = 1;
	v[TF_F_O_DIST_CM] = 400;
	v[TF_F_O_WIDTH_CM] = 15;
	v[TF_F_REJECTED] = 3;
	v[TF_F_DROPPED] = 0;
}

static void to_hex(const uint8_t *b, size_t n, char *out)
{
	size_t i;

	for (i = 0; i < n; i++) {
		sprintf(&out[2 * i], "%02x", b[i]);
	}
}

static void mid_values(int32_t v[TF_FIELD_COUNT])
{
	int i;

	for (i = 0; i < TF_FIELD_COUNT; i++) {
		v[i] = tf_fields[i].min + (tf_fields[i].max - tf_fields[i].min) / 2;
	}
	v[TF_F_VERSION] = (int32_t)TF_LAYOUT_VERSION;
}

static void test_layout(void)
{
	int i, bits = 0;

	for (i = 0; i < TF_FIELD_COUNT; i++) {
		int64_t range = (int64_t)tf_fields[i].max - tf_fields[i].min;

		bits += tf_fields[i].bits;
		CHECK(range < ((int64_t)1 << tf_fields[i].bits) &&
		      range >= ((int64_t)1 << (tf_fields[i].bits - 1)),
		      "%s is not minimum-bit", tf_fields[i].name);
	}
	CHECK(bits == 161 && TF_FRAME_BITS == 161, "161 data bits, got %d", bits);
	CHECK(TF_FRAME_BYTES == 21 && TF_SPARE_BITS == 7, "21 bytes, 7 spare");
}

/* Every legal value of every field survives pack/unpack. */
static void test_exhaustive(void)
{
	int32_t		in[TF_FIELD_COUNT], out[TF_FIELD_COUNT];
	uint8_t		frame[TF_FRAME_BYTES];
	int		i, j;
	int32_t		v;
	unsigned	bad = 0;

	for (i = 0; i < TF_FIELD_COUNT; i++) {
		if (i == TF_F_VERSION) {
			continue;
		}
		mid_values(in);
		for (v = tf_fields[i].min; v <= tf_fields[i].max; v++) {
			in[i] = v;
			if ((tf_pack(in, frame) != 0U) || !tf_unpack(frame, sizeof(frame), out)) {
				bad++;
				continue;
			}
			for (j = 0; j < TF_FIELD_COUNT; j++) {
				if (in[j] != out[j]) {
					bad++;
				}
			}
		}
		CHECK(bad == 0U, "%s round trip: %u mismatches", tf_fields[i].name, bad);
		bad = 0;
	}
}

static void test_clamp(void)
{
	int32_t		in[TF_FIELD_COUNT], out[TF_FIELD_COUNT];
	uint8_t		frame[TF_FRAME_BYTES];
	int		i;
	const int32_t	extremes[2] = { INT32_MIN, INT32_MAX };

	for (i = 1; i < TF_FIELD_COUNT; i++) {
		int k;
		for (k = 0; k < 4; k++) {
			int32_t want;
			mid_values(in);
			switch (k) {
			case 0: in[i] = tf_fields[i].min - 1; want = tf_fields[i].min; break;
			case 1: in[i] = tf_fields[i].max + 1; want = tf_fields[i].max; break;
			default: in[i] = extremes[k - 2];
				want = (k == 2) ? tf_fields[i].min : tf_fields[i].max; break;
			}
			CHECK(tf_pack(in, frame) == (1UL << i), "%s clamp mask", tf_fields[i].name);
			CHECK(tf_unpack(frame, sizeof(frame), out) && out[i] == want,
			      "%s clamps to %d, got %d", tf_fields[i].name, want, out[i]);
		}
	}

	/* The Tribunal example: -1 must not become a large positive number. */
	mid_values(in);
	in[TF_F_SPEED_L] = -1;
	(void)tf_pack(in, frame);
	CHECK(tf_unpack(frame, sizeof(frame), out) && out[TF_F_SPEED_L] == -1,
	      "-1 mm/s decodes as -1");
}

static void test_malformed(void)
{
	int32_t	in[TF_FIELD_COUNT], out[TF_FIELD_COUNT];
	uint8_t	frame[TF_FRAME_BYTES], bad[TF_FRAME_BYTES];

	mid_values(in);
	(void)tf_pack(in, frame);
	CHECK(!tf_unpack(frame, TF_FRAME_BYTES - 1, out), "short frame rejected");
	CHECK(!tf_unpack(frame, TF_FRAME_BYTES + 1, out), "long frame rejected");
	CHECK(!tf_unpack(NULL, TF_FRAME_BYTES, out), "NULL rejected");

	memcpy(bad, frame, sizeof(bad));
	bad[TF_FRAME_BYTES - 1] |= 0x01;		/* spare bit */
	CHECK(!tf_unpack(bad, sizeof(bad), out), "spare bit set rejected");

	memcpy(bad, frame, sizeof(bad));
	bad[0] = (uint8_t)((bad[0] & 0x0F) | 0xF0);	/* version 15 */
	CHECK(!tf_unpack(bad, sizeof(bad), out), "wrong version rejected");

	/* M_STATE is bits 28..30: code 7 is outside 0..6. */
	memcpy(bad, frame, sizeof(bad));
	bad[3] = (uint8_t)(bad[3] | 0x0E);
	CHECK(!tf_unpack(bad, sizeof(bad), out), "M_STATE code 7 rejected");
}

static void test_golden(void)
{
	int32_t	in[TF_FIELD_COUNT], out[TF_FIELD_COUNT];
	uint8_t	frame[TF_FRAME_BYTES];
	char	hex[2 * TF_FRAME_BYTES + 1];

	golden_values(in);
	CHECK(tf_pack(in, frame) == 0U, "golden values in range");
	to_hex(frame, sizeof(frame), hex);
	printf("golden frame: %s\n", hex);
	CHECK(strcmp(hex, GOLDEN_HEX) == 0, "golden frame changed: %s", hex);
	CHECK(tf_unpack(frame, sizeof(frame), out) && memcmp(in, out, sizeof(in)) == 0,
	      "golden round trip");
}

static void test_commands(void)
{
	unsigned	word, valid = 0, bad = 0;
	tf_command_t	cmd, back;
	uint8_t		raw[2], again[2];
	tf_ack_t	res, r;

	for (word = 0; word < 65536U; word++) {
		unsigned op = word >> 13;
		raw[0] = (uint8_t)(word >> 8);
		raw[1] = (uint8_t)word;
		if (tf_cmd_decode(raw, 2, &cmd)) {
			valid++;
			tf_cmd_encode(&cmd, again);
			if (memcmp(raw, again, 2) != 0 || op < 1 || op > 3) {
				bad++;
			}
		} else if (op >= 1 && op <= 3) {
			bad++;
		}
	}
	CHECK(bad == 0U && valid == 3U * 8192U, "command words: %u valid, %u bad", valid, bad);
	CHECK(!tf_cmd_decode(raw, 1, &cmd) && !tf_cmd_decode(raw, 3, &cmd), "command length checked");

	bad = 0;
	for (word = 0; word < 4U * 8U * 256U; word++) {
		res = (tf_ack_t)(word >> 11);
		cmd.opcode = (uint8_t)((word >> 8) & 7U);
		cmd.seq = (uint8_t)word;
		cmd.session = 31;
		tf_ack_encode(res, &cmd, raw);
		if (!tf_ack_decode(raw, 2, &r, &back) || r != res ||
		    back.opcode != cmd.opcode || back.seq != cmd.seq) {
			bad++;
		}
	}
	CHECK(bad == 0U, "ack round trip: %u bad", bad);
}

static void test_mapping(void)
{
	robot_status_t	st;
	tm_frame_info_t	info = { 300, 12.34, 9, 2 };
	int32_t		v[TF_FIELD_COUNT], out[TF_FIELD_COUNT];
	uint8_t		frame[TF_FRAME_BYTES];

	memset(&st, 0, sizeof(st));
	st.motion.state = DRIVING;
	st.motion.mode = DISTANCE_MOVE;
	st.motion.fault = F_NONE;
	st.motion.id = 0x1234;
	st.motion.speed[0] = -0.1234;		/* -123.4 mm/s */
	st.motion.speed[1] = 0.0995;		/* 99.5 -> 100 */
	st.motion.centre = -6.0;		/* beyond -5000 mm: clamps */
	st.motion.heading = 190.0 * PI / 180.0;	/* wraps to -170 */
	st.motion.feedback_trusted = true;
	st.motion.hold = true;
	st.owner_valid = false;
	st.line.available = false;		/* slots ignored */
	st.line.values[0] = 1.0;
	st.line.values[1] = 1.0;
	st.barcode.available = true;
	st.barcode.values[0] = TF_BC_RIGHT;
	st.barcode.values[1] = 1.0;
	st.terrain.available = true;
	st.terrain.values[0] = -10.0 * PI / 180.0;	/* heading invalid: not sent */
	st.terrain.values[1] = 15.4 * PI / 180.0;
	st.terrain.values[2] = 0.0505;
	st.terrain.values[3] = (double)(TF_TV_PITCH | TF_TV_HUMP | (5U << 3));
	st.obstacle.available = true;
	st.obstacle.values[0] = NAN;			/* non-finite -> 0 */
	st.obstacle.values[2] = 0.2;
	st.obstacle.values[3] = (double)(TF_OF_SCANNING | TF_OF_NO_ECHO);

	tm_fill(&st, &info, v);
	CHECK(v[TF_F_SEQ] == 300 % 256, "seq wraps");
	CHECK(v[TF_F_UPTIME_DS] == 123, "uptime 0.1 s");
	CHECK(v[TF_F_M_STATE] == DRIVING && v[TF_F_M_FAULT] == F_NONE, "motion state");
	CHECK(v[TF_F_M_FLAGS] == (int32_t)(TF_MF_TRUSTED | TF_MF_HOLD), "motion flags");
	CHECK(v[TF_F_OWNER] == TF_OWNER_NONE, "no owner");
	CHECK(v[TF_F_CMD_ID] == 0x34, "cmd id low byte");
	CHECK(v[TF_F_SPEED_L] == -123 && v[TF_F_SPEED_R] == 100, "speeds %d %d",
	      v[TF_F_SPEED_L], v[TF_F_SPEED_R]);
	CHECK(v[TF_F_HEADING] == -170, "heading wraps, got %d", v[TF_F_HEADING]);
	CHECK(v[TF_F_LINE_AVAIL] == 0 && v[TF_F_LINE_ERR] == 0 && v[TF_F_LINE_TRACK] == 0,
	      "unavailable line sends zeros");
	CHECK(v[TF_F_BC_VALID] == 1 && v[TF_F_BC_CMD] == TF_BC_RIGHT, "barcode");
	CHECK(v[TF_F_T_VALID] == (int32_t)(TF_TV_PITCH | TF_TV_HUMP) && v[TF_F_T_STATE] == 5,
	      "terrain validity and state");
	CHECK(v[TF_F_PITCH] == 15 && v[TF_F_HUMP_MM] == 51 && v[TF_F_T_HEADING] == 0,
	      "terrain values %d %d %d", v[TF_F_PITCH], v[TF_F_HUMP_MM], v[TF_F_T_HEADING]);
	CHECK(v[TF_F_O_FLAGS] == (int32_t)(TF_OF_SCANNING | TF_OF_NO_ECHO) &&
	      v[TF_F_O_DIST_CM] == 0 && v[TF_F_O_WIDTH_CM] == 20, "obstacle");
	CHECK(v[TF_F_REJECTED] == 7 && v[TF_F_DROPPED] == 2, "counters saturate");

	CHECK(tf_pack(v, frame) == (1UL << TF_F_CENTRE_MM), "only centre clamps");
	CHECK(tf_unpack(frame, sizeof(frame), out) && out[TF_F_CENTRE_MM] == -5000,
	      "centre clamps to -5000");
}

static void test_edges(void)
{
	robot_status_t a, b;

	memset(&a, 0, sizeof(a));
	a.line.available = true;
	a.line.values[1] = 1.0;
	b = a;
	CHECK(tm_edge_events(&a, &b) == 0U, "no change, no event");
	b.line.values[1] = 0.0;
	b.motion.fault = F_TIMING;
	b.barcode.available = true;
	b.barcode.values[1] = 1.0;
	CHECK(tm_edge_events(&a, &b) == (TF_EV_LINE_LOST | TF_EV_MOTION_FAULT | TF_EV_BARCODE),
	      "edges latch events");
	a = b;
	b.barcode.timestamp = 5.0;		/* same decode, republished */
	CHECK(tm_edge_events(&a, &b) == 0U, "steady fault/barcode not repeated");
	b.barcode.values[0] = TF_BC_STRAIGHT;
	CHECK(tm_edge_events(&a, &b) == TF_EV_BARCODE, "new instruction reported");
}

static void test_distinct_motion_faults(void)
{
	const motion_fault_t faults[] = { F_SETUP, F_SYSTEM, F_ESTOP, F_SETUP_TIMEOUT };
	for (unsigned i = 0U; i < sizeof(faults) / sizeof(faults[0]); ++i) {
		robot_status_t st = {0};
		tm_frame_info_t info = {0};
		int32_t values[TF_FIELD_COUNT], decoded[TF_FIELD_COUNT];
		uint8_t frame[TF_FRAME_BYTES];
		st.motion.state = FAULT;
		st.motion.fault = faults[i];
		tm_fill(&st, &info, values);
		CHECK(tf_pack(values, frame) == 0U, "fault %u must not clamp", (unsigned)faults[i]);
		CHECK(tf_unpack(frame, sizeof(frame), decoded), "fault frame decodes");
		CHECK(decoded[TF_F_M_FAULT] == (int32_t)faults[i], "fault reason survives telemetry");
	}
}

/* Golden encoder frame, shared with tools/dashboard.py --selftest. */
static const char GOLDEN_ENC_HEX[] =
	"010707200012d687000cfffffffe0001e2400000000000000003";

static void golden_encoder(tf_encoder_t *e)
{
	memset(e, 0, sizeof(*e));
	e->seq = 7;
	e->reliable = true;
	e->trusted = true;
	e->fresh = true;
	e->cause = FB_NONE;
	e->state = DRIVING;
	e->fault = F_NONE;
	e->sample_ms = 1234567U;
	e->age_ms = 12U;
	e->count[0] = 0xFFFFFFFEU;		/* -2: reversed past zero */
	e->count[1] = 123456U;
	e->invalid[0] = 0U;
	e->invalid[1] = 3U;
}

static bool enc_equal(const tf_encoder_t *a, const tf_encoder_t *b)
{
	return a->seq == b->seq && a->reliable == b->reliable && a->trusted == b->trusted &&
	       a->fresh == b->fresh && a->cause == b->cause && a->state == b->state &&
	       a->fault == b->fault && a->sample_ms == b->sample_ms && a->age_ms == b->age_ms &&
	       a->count[0] == b->count[0] && a->count[1] == b->count[1] &&
	       a->invalid[0] == b->invalid[0] && a->invalid[1] == b->invalid[1];
}

static void test_encoder_frame(void)
{
	tf_encoder_t	in, out;
	uint8_t		frame[TF_ENC_BYTES], bad[TF_ENC_BYTES];
	char		hex[2 * TF_ENC_BYTES + 1];
	unsigned	s, f, c;

	golden_encoder(&in);
	CHECK(!tf_enc_pack(&in, frame), "golden encoder in range");
	to_hex(frame, sizeof(frame), hex);
	printf("golden encoder frame: %s\n", hex);
	CHECK(strcmp(hex, GOLDEN_ENC_HEX) == 0, "golden encoder frame changed: %s", hex);
	CHECK(tf_enc_unpack(frame, sizeof(frame), &out) && enc_equal(&in, &out),
	      "golden encoder round trip");

	/* Full 32-bit counts survive, including extremes. */
	in.count[0] = 0U;
	in.count[1] = 0xFFFFFFFFU;
	in.invalid[0] = 0x80000000U;
	in.invalid[1] = 0xFFFFFFFFU;
	in.sample_ms = 0xFFFFFFFFU;
	in.age_ms = (uint16_t)TF_ENC_AGE_UNKNOWN;
	(void)tf_enc_pack(&in, frame);
	CHECK(tf_enc_unpack(frame, sizeof(frame), &out) && enc_equal(&in, &out),
	      "32-bit extremes round trip");

	/* Every legal state/fault/cause and flag combination. */
	for (s = 0U; s <= TF_ENC_STATE_MAX; s++) {
		for (f = 0U; f <= TF_ENC_FAULT_MAX; f++) {
			for (c = 0U; c <= TF_EF_CAUSE_MAX; c++) {
				in.state = (uint8_t)s;
				in.fault = (uint8_t)f;
				in.cause = (uint8_t)c;
				in.reliable = (c & 1U) != 0U;
				in.trusted = (f & 1U) != 0U;
				in.fresh = (s & 1U) != 0U;
				CHECK(!tf_enc_pack(&in, frame) &&
				      tf_enc_unpack(frame, sizeof(frame), &out) && enc_equal(&in, &out),
				      "codes %u/%u/%u round trip", s, f, c);
			}
		}
	}

	/* Out-of-range codes clamp, never wrap. */
	in.state = 9;
	in.fault = 200;
	in.cause = 7;
	CHECK(tf_enc_pack(&in, frame), "encoder clamp reported");
	CHECK(tf_enc_unpack(frame, sizeof(frame), &out) && out.state == TF_ENC_STATE_MAX &&
	      out.fault == TF_ENC_FAULT_MAX && out.cause == TF_EF_CAUSE_MAX, "encoder clamps");

	golden_encoder(&in);
	(void)tf_enc_pack(&in, frame);
	CHECK(!tf_enc_unpack(frame, TF_ENC_BYTES - 1, &out), "short encoder frame rejected");
	CHECK(!tf_enc_unpack(frame, TF_ENC_BYTES + 1, &out), "long encoder frame rejected");
	CHECK(!tf_enc_unpack(NULL, TF_ENC_BYTES, &out), "NULL encoder frame rejected");
	CHECK(!tf_enc_unpack(frame, TF_FRAME_BYTES, &out), "status frame is not an encoder frame");
	memcpy(bad, frame, sizeof(bad));
	bad[0] = 2;
	CHECK(!tf_enc_unpack(bad, sizeof(bad), &out), "encoder version rejected");
	memcpy(bad, frame, sizeof(bad));
	bad[2] |= 0x40;
	CHECK(!tf_enc_unpack(bad, sizeof(bad), &out), "encoder reserved bit rejected");
	memcpy(bad, frame, sizeof(bad));
	bad[3] = 0x70;
	CHECK(!tf_enc_unpack(bad, sizeof(bad), &out), "encoder state 7 rejected");
	memcpy(bad, frame, sizeof(bad));
	bad[3] = 0x2D;
	CHECK(!tf_enc_unpack(bad, sizeof(bad), &out), "encoder fault 13 rejected");
	memcpy(bad, frame, sizeof(bad));
	bad[2] = (uint8_t)(bad[2] | (6U << TF_EF_CAUSE_SHIFT));
	CHECK(!tf_enc_unpack(bad, sizeof(bad), &out), "encoder cause 6 rejected");
}

static void test_encoder_mapping(void)
{
	robot_status_t	st;
	tf_encoder_t	e;

	memset(&st, 0, sizeof(st));
	st.motion.state = FAULT;
	st.motion.fault = F_FEEDBACK;
	st.motion.feedback_cause = FB_NO_PROGRESS;
	st.motion.feedback_trusted = false;
	st.motion.encoder.reliable = true;
	st.motion.encoder.count[0] = 0xFFFFFFF0U;
	st.motion.encoder.count[1] = 42U;
	st.motion.encoder.invalid[0] = 0xFFFFFFFFU;	/* never clamped */
	st.motion.encoder.invalid[1] = 1U;
	st.motion.encoder_sample_time = 12.345;

	tm_fill_encoder(&st, 300U, 12.395, &e);
	CHECK(e.seq == 300U % 256U, "encoder seq low byte");
	CHECK(e.state == FAULT && e.fault == F_FEEDBACK && e.cause == FB_NO_PROGRESS,
	      "encoder state/fault/cause");
	CHECK(e.reliable && !e.trusted, "reliable but not trusted");
	CHECK(e.count[0] == 0xFFFFFFF0U && e.count[1] == 42U &&
	      e.invalid[0] == 0xFFFFFFFFU && e.invalid[1] == 1U, "counts copied unchanged");
	CHECK(e.sample_ms == 12345U, "sample ms %u", (unsigned)e.sample_ms);
	CHECK(e.age_ms == 50U && e.fresh, "50 ms old is fresh, age %u", (unsigned)e.age_ms);

	tm_fill_encoder(&st, 0U, 12.345 + 0.150, &e);
	CHECK(e.age_ms == 150U && !e.fresh, "150 ms old is stale");

	tm_fill_encoder(&st, 0U, 12.345 + 100.0, &e);
	CHECK(e.age_ms == TF_ENC_AGE_UNKNOWN && !e.fresh, "very old saturates");

	tm_fill_encoder(&st, 0U, 12.0, &e);
	CHECK(e.age_ms == TF_ENC_AGE_UNKNOWN && !e.fresh, "sample from the future is not fresh");

	st.motion.encoder_sample_time = NAN;
	tm_fill_encoder(&st, 0U, 12.0, &e);
	CHECK(e.sample_ms == 0U && e.age_ms == TF_ENC_AGE_UNKNOWN && !e.fresh,
	      "non-finite sample time is not fresh");

	st.motion.encoder_sample_time = 5.0;
	tm_fill_encoder(&st, 0U, NAN, &e);
	CHECK(e.age_ms == TF_ENC_AGE_UNKNOWN && !e.fresh, "non-finite clock is not fresh");

	/* Robot clock past 2^32 ms (49.7 days) wraps the ms field. */
	st.motion.encoder_sample_time = 4294967.296 + 1.0;
	tm_fill_encoder(&st, 0U, 4294967.296 + 1.01, &e);
	CHECK(e.sample_ms >= 999U && e.sample_ms <= 1001U && e.fresh,
	      "sample ms wraps, got %u", (unsigned)e.sample_ms);
}

/* Tribunal follow-up: "measure JSON vs packed size". */
static void report_json_size(void)
{
	int32_t	v[TF_FIELD_COUNT];
	char	json[1024];
	int	i, n = 0;

	golden_values(v);
	n += snprintf(&json[n], sizeof(json) - (size_t)n, "{");
	for (i = 0; i < TF_FIELD_COUNT; i++) {
		n += snprintf(&json[n], sizeof(json) - (size_t)n, "%s\"%s\":%d",
			      (i == 0) ? "" : ",", tf_fields[i].name, (int)v[i]);
	}
	n += snprintf(&json[n], sizeof(json) - (size_t)n, "}");
	printf("size: JSON %d bytes, packed %d bytes (%d bits used, %d spare), %.1f%% of JSON\n",
	       n, TF_FRAME_BYTES, TF_FRAME_BITS, TF_SPARE_BITS,
               100.0 * (double)TF_FRAME_BYTES / n);
	CHECK(TF_FRAME_BYTES < n, "packed frame is smaller than JSON");
}

int main(void)
{
	test_layout();
	test_exhaustive();
	test_clamp();
	test_malformed();
	test_golden();
	test_commands();
	test_mapping();
	test_edges();
	test_distinct_motion_faults();
	test_encoder_frame();
	test_encoder_mapping();
	report_json_size();
	printf("%s: %u checks, %u failed\n", failures ? "FAIL" : "PASS", checks, failures);
	return failures ? 1 : 0;
}
