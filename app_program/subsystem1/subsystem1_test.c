/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
	ER	er;

	TEST_INIT();

	/* S1-U01: a telemetry message is accepted while the queue has room */
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

	/* S1-U04: only valid commands reach the Vehicle Controller's queue */
	{
		COMMS_CMD	cmd = 0;
		INT		size;

		comms_hook_command("garbage", 7);
		size = tk_rcv_mbf(mbf_comms_cmd, &cmd, TMO_POL);
		comms_hook_command("stop", 4);
		TEST_CHECK("S1-U04", size == E_TMOUT &&
			   tk_rcv_mbf(mbf_comms_cmd, &cmd, TMO_POL) == (INT)sizeof(cmd) &&
			   cmd == COMMS_CMD_STOP,
			   "valid command forwarded, invalid rejected");
	}

	/* Empty the queues again, so the test messages do not sit in them
	   on a build that has no MQTT client to take them out. */
	{
		char	topic[COMMS_TOPIC_MAX];
		char	payload[COMMS_PAYLOAD_MAX];
		int	qos;

		while (comms_hook_next(topic, payload, &qos) >= 0) {
		}
	}

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
