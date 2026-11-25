/*
 *  subsystem1_test.c
 *  On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *  Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *  console output to testing/subsystem1/logs/ and record the result
 *  in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 *  The same frame/command checks also run on the PC: tests/subsystem1.
 *
 *  Run these before the Vehicle Controller task starts: S1-U04/U09 put
 *  commands on mbf_comms_cmd, and a running controller would act on them.
 */

#include <stdint.h>
#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

LOCAL void
send_cmd (UB opcode, UB session, UB seq)
{
    tf_command_t cmd = {opcode, session, seq};
    UB           raw[TF_CMD_BYTES];

    tf_cmd_encode(&cmd, raw);
    comms_hook_command(raw, TF_CMD_BYTES);
}

LOCAL INT
take_cmd (COMMS_CMD * p_cmd)
{
    return tk_rcv_mbf(g_mbf_comms_cmd, p_cmd, TMO_POL);
}

LOCAL void
drain (void)
{
    char      topic[COMMS_TOPIC_MAX];
    uint8_t   payload[COMMS_PAYLOAD_MAX];
    int       qos;
    COMMS_CMD cmd;

    while (comms_hook_next(topic, payload, &qos) >= 0)
    {
    }

    while (take_cmd(&cmd) >= E_OK)
    {
    }
}

EXPORT void
subsystem1_test (void)
{
    ER result;

    TEST_INIT();
    drain();

    /* S1-U01: a diagnostic message is accepted while the queue has room */
    result = comms_send_telemetry("test", "hello");
    TEST_CHECK("S1-U01", result == E_OK, "telemetry is queued");

    /* S1-U02: a full queue drops the message at once instead of waiting */
    {
        UW  before = comms_dropped();
        INT index;

        result = E_OK;

        for (index = 0; index < 20 && E_OK == result; index++)
        {
            result = comms_send_telemetry("test", "fill");
        }

        TEST_CHECK("S1-U02", result == E_TMOUT && comms_dropped() > before,
                   "full queue drops and counts");
    }

    /* S1-U03: an oversized payload is refused */
    {
        char big[COMMS_PAYLOAD_MAX + 1];
        INT  index;

        for (index = 0; index < COMMS_PAYLOAD_MAX; index++)
        {
            big[index] = 'x';
        }

        big[COMMS_PAYLOAD_MAX] = '\0';
        TEST_CHECK("S1-U03", comms_send_telemetry("test", big) == E_PAR,
                   "oversized payload is refused");
    }

    drain();

    /* S1-U04: a valid binary command reaches the controller's queue;
       wrong length, truncated and reserved opcodes do not */
    {
        COMMS_CMD cmd         = 0;
        UB        junk[3]     = {0x20, 0x01, 0x00};
        UB        reserved[2] = {0xE0, 0x01}; /* opcode 7 */
        BOOL      none;

        comms_hook_command(junk, 3);
        comms_hook_command(junk, -1);
        comms_hook_command(reserved, 2);
        none = (E_TMOUT == take_cmd(&cmd));
        send_cmd(TF_OP_STOP, 9, 1);
        TEST_CHECK("S1-U04",
                   none && take_cmd(&cmd) == (INT)sizeof(cmd)
                       && COMMS_CMD_STOP == cmd,
                   "valid command forwarded, malformed rejected");
    }

    /* S1-U09: a repeated or stale START is not forwarded; STOP always is */
    {
        COMMS_CMD cmd = 0;

        BOOL first, again, stale, stop, fresh;

        send_cmd(TF_OP_START, 9, 2);
        first = (take_cmd(&cmd) >= E_OK) && (COMMS_CMD_START == cmd);
        send_cmd(TF_OP_START, 9, 2); /* QoS 1 redelivery */
        again = (E_TMOUT == take_cmd(&cmd));
        send_cmd(TF_OP_START, 9, 1); /* older */
        stale = (E_TMOUT == take_cmd(&cmd));
        send_cmd(TF_OP_STOP, 9, 2); /* repeated stop */
        stop = (take_cmd(&cmd) >= E_OK) && (COMMS_CMD_STOP == cmd);
        send_cmd(TF_OP_START, 10, 0); /* new dashboard session */
        fresh = (take_cmd(&cmd) >= E_OK) && (COMMS_CMD_START == cmd);
        TEST_CHECK("S1-U09", first && again && stale && stop && fresh,
                   "duplicate START suppressed, STOP always forwarded");
    }

    /* S1-U10: frame round trip keeps negatives and clamps out-of-range */
    {
        int32_t inward[TF_FIELD_COUNT] = {0}, out[TF_FIELD_COUNT];
        UB      frame[TF_FRAME_BYTES];
        INT     index;
        BOOL    same;
        UW      clamped;

        inward[TF_F_VERSION]   = (int32_t)TF_LAYOUT_VERSION;
        inward[TF_F_SPEED_L]   = -1;
        inward[TF_F_SPEED_R]   = 999; /* clamps to 250 */
        inward[TF_F_CENTRE_MM] = -5000;
        inward[TF_F_HEADING]   = -180;
        inward[TF_F_LINE_ERR]  = -1;
        inward[TF_F_PITCH]     = -90;
        clamped                = tf_pack(inward, frame);
        inward[TF_F_SPEED_R]   = 250;
        same                   = tf_unpack(frame, TF_FRAME_BYTES, out);

        for (index = 0; same && index < TF_FIELD_COUNT; index++)
        {
            same = (inward[index] == out[index]);
        }

        TEST_CHECK("S1-U10",
                   same && clamped == (1UL << TF_F_SPEED_R)
                       && TF_FRAME_BYTES == 21,
                   "21-byte frame round trip, negatives kept, clamp not wrap");
    }

    drain();
    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
