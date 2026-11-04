/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 *	The same frame/command checks also run on the PC: tests/buddy1_frame.
 *
 *	Run these before the Vehicle Controller task starts: S1-U04/U09 put
 *	commands on mbf_comms_cmd, and a running controller would act on them.
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

LOCAL void send_cmd(UB opcode, UB session, UB seq)
{
	tf_command_t	cmd = { opcode, session, seq };
	UB		raw[TF_CMD_BYTES];

	tf_cmd_encode(&cmd, raw);
	comms_hook_command(raw, TF_CMD_BYTES);
}

LOCAL INT take_cmd(COMMS_CMD *cmd)
{
	return tk_rcv_mbf(mbf_comms_cmd, cmd, TMO_POL);
}

LOCAL void drain(void)
{
	char		topic[COMMS_TOPIC_MAX];
	unsigned char	payload[COMMS_PAYLOAD_MAX];
	int		qos;
	COMMS_CMD	cmd;

	while (comms_hook_next(topic, payload, &qos) >= 0) {
	}
	while (take_cmd(&cmd) >= E_OK) {
	}
}

EXPORT void subsystem1_test(void)
{
	ER	er;

	TEST_INIT();
	drain();

	/* S1-U01: a diagnostic message is accepted while the queue has room */
	er = comms_send_telemetry("test", "hello");
	TEST_CHECK("S1-U01", er == E_OK, "telemetry is queued");

	/* S1-U02: a full queue drops the message at once instead of waiting */
	{
		UW	before = comms_dropped();
		INT	i;

		er = E_OK;
		for (i = 0; i < 20 && er == E_OK; i++) {
			er = comms_send_telemetry("test", "fill");
		}
		TEST_CHECK("S1-U02", er == E_TMOUT && comms_dropped() > before,
			   "full queue drops and counts");
	}

	/* S1-U03: an oversized payload is refused */
	{
		char	big[COMMS_PAYLOAD_MAX + 1];
		INT	i;

		for (i = 0; i < COMMS_PAYLOAD_MAX; i++) {
			big[i] = 'x';
		}
		big[COMMS_PAYLOAD_MAX] = '\0';
		TEST_CHECK("S1-U03", comms_send_telemetry("test", big) == E_PAR,
			   "oversized payload is refused");
	}
	drain();

	/* S1-U04: a valid binary command reaches the controller's queue;
	   wrong length, truncated and reserved opcodes do not */
	{
		COMMS_CMD	cmd = 0;
		UB		junk[3] = { 0x20, 0x01, 0x00 };
		UB		reserved[2] = { 0xE0, 0x01 };	/* opcode 7 */
		BOOL		none;

		comms_hook_command(junk, 3);
		comms_hook_command(junk, -1);
		comms_hook_command(reserved, 2);
		none = (take_cmd(&cmd) == E_TMOUT);
		send_cmd(TF_OP_STOP, 9, 1);
		TEST_CHECK("S1-U04", none &&
			   take_cmd(&cmd) == (INT)sizeof(cmd) && cmd == COMMS_CMD_STOP,
			   "valid command forwarded, malformed rejected");
	}

	/* S1-U09: a repeated or stale START is not forwarded; STOP always is */
	{
		COMMS_CMD	cmd = 0;
		BOOL		first, again, stale, stop, fresh;

		send_cmd(TF_OP_START, 9, 2);
		first = (take_cmd(&cmd) >= E_OK) && (cmd == COMMS_CMD_START);
		send_cmd(TF_OP_START, 9, 2);		/* QoS 1 redelivery */
		again = (take_cmd(&cmd) == E_TMOUT);
		send_cmd(TF_OP_START, 9, 1);		/* older */
		stale = (take_cmd(&cmd) == E_TMOUT);
		send_cmd(TF_OP_STOP, 9, 2);		/* repeated stop */
		stop = (take_cmd(&cmd) >= E_OK) && (cmd == COMMS_CMD_STOP);
		send_cmd(TF_OP_START, 10, 0);		/* new dashboard session */
		fresh = (take_cmd(&cmd) >= E_OK) && (cmd == COMMS_CMD_START);
		TEST_CHECK("S1-U09", first && again && stale && stop && fresh,
			   "duplicate START suppressed, STOP always forwarded");
	}

	/* S1-U10: frame round trip keeps negatives and clamps out-of-range */
	{
		int32_t	in[TF_FIELD_COUNT] = {0}, out[TF_FIELD_COUNT];
		UB	frame[TF_FRAME_BYTES];
		INT	i;
		BOOL	same;
		UW	clamped;

		in[TF_F_VERSION] = (int32_t)TF_LAYOUT_VERSION;
		in[TF_F_SPEED_L] = -1;
		in[TF_F_SPEED_R] = 999;			/* clamps to 250 */
		in[TF_F_CENTRE_MM] = -5000;
		in[TF_F_HEADING] = -180;
		in[TF_F_LINE_ERR] = -1;
		in[TF_F_PITCH] = -90;
		clamped = tf_pack(in, frame);
		in[TF_F_SPEED_R] = 250;
		same = tf_unpack(frame, TF_FRAME_BYTES, out);
		for (i = 0; same && i < TF_FIELD_COUNT; i++) {
			same = (in[i] == out[i]);
		}
		TEST_CHECK("S1-U10", same && clamped == (1UL << TF_F_SPEED_R) &&
			   TF_FRAME_BYTES == 21,
			   "21-byte frame round trip, negatives kept, clamp not wrap");
	}

	drain();
	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
