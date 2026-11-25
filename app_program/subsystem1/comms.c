/*
 *  comms.c
 *  Subsystem 1 -- status frame, events, commands and their queues.
 *
 *  comms_task (lowest application priority) copies the shared robot status
 *  every 100 ms with motion_get_status(), which holds the priority-
 *  inheritance bus mutex only for the copy and never waits for it.  The
 *  frame is then built and packed with no lock held, and queued.  The MQTT
 *  client (mqtt_utk.c, in the kernel's WiFi service task) empties the
 *  queues through the hooks in comms_hooks.h.  Every queue access uses
 *  TMO_POL, so neither side can hold the other up, and nothing here can
 *  delay the 10 ms motion task beyond one snapshot copy.
 */

#include "../../include/float_types.h"
#include <stdint.h>
#include "subsystem1.h" /* first: sets up size_t for the kernel */
#include "robot.h"
#include "motion_hal.h"
#include "telemetry_map.h"
#include "../subsystem3/route.h"

#define COMMS_TLM_DEPTH     8   /* queued frames / diagnostics */
#define COMMS_EVT_DEPTH     4   /* queued events and acks */
#define COMMS_CMD_DEPTH     4   /* commands waiting for the controller */
#define COMMS_MBF_OVERHEAD  8   /* kernel bookkeeping per message, bytes */
#define COMMS_STATUS_MS     100 /* status frame period: 10 Hz */
#define COMMS_REPORT_FRAMES 100 /* console statistics every 10 s */

_Static_assert(((int)COMMS_CMD_START == (int)TF_OP_START)
                   && ((int)COMMS_CMD_STOP == (int)TF_OP_STOP)
                   && ((int)COMMS_CMD_ESTOP == (int)TF_OP_ESTOP),
               "comms_cmd_t must equal the binary opcodes");
_Static_assert(TF_FRAME_BYTES <= COMMS_PAYLOAD_MAX,
               "frame does not fit a message");
_Static_assert(TF_ENC_BYTES <= COMMS_PAYLOAD_MAX,
               "encoder frame does not fit a message");

typedef struct
{
    UB   qos;
    UB   len;
    char topic[COMMS_TOPIC_MAX];
    UB   payload[COMMS_PAYLOAD_MAX];
} comms_msg_t;

EXPORT ID g_mbf_comms_cmd; /* IF-07, see comms.h */

LOCAL ID            g_mbf_comms_tlm;
LOCAL ID            g_mbf_comms_evt;
LOCAL volatile BOOL g_comms_online;
LOCAL volatile UW   g_comms_drops;  /* diagnostic only */
LOCAL volatile UB   g_comms_events; /* latched TF_EV_* bits; DI/EI only */
LOCAL comms_stats_t g_stats;

/* Command de-duplication; touched only by comms_hook_command(). */
LOCAL BOOL g_cmd_seen;
LOCAL UB   g_cmd_session;
LOCAL UB   g_cmd_seq;

/* comms_task's snapshots: static to keep them off its 2 KB stack. */
LOCAL robot_status_t g_snap;
LOCAL robot_status_t g_prev;

/* Copy a string into dst[max].  Returns its length, or -1 if it does not fit.
 */
LOCAL INT
copy_text (char * p_dst, const char * p_src, INT max)
{
    INT index;

    for (index = 0; index < max; index++)
    {
        p_dst[index] = p_src[index];

        if ('\0' == p_src[index])
        {
            return index;
        }
    }

    return -1;
}

LOCAL ER
queue_message (ID mbf, UB qos, const char * p_topic, const char * p_name,
               const UB * p_data, INT len)
{
    comms_msg_t msg;

    INT tlen;
    INT index;
    ER  result;

    if ((NULL == p_data) || (len < 0) || (len > COMMS_PAYLOAD_MAX))
    {
        return E_PAR;
    }

    tlen = copy_text(msg.topic, p_topic, COMMS_TOPIC_MAX);

    if (NULL != p_name)
    {
        if (('\0' == p_name[0])
            || (copy_text(&msg.topic[tlen], p_name, COMMS_NAME_MAX) < 0))
        {
            return E_PAR;
        }
    }

    for (index = 0; index < len; index++)
    {
        msg.payload[index] = p_data[index];
    }

    msg.len = (UB)len;
    msg.qos = qos;

    result = tk_snd_mbf(mbf, &msg, sizeof(msg), TMO_POL);

    if (result < E_OK)
    {
        g_comms_drops++;
    }

    return result;
}

EXPORT ER
comms_send_telemetry (const char * p_name, const char * p_payload)
{
    char text[COMMS_PAYLOAD_MAX];
    INT  len;
    ER   result = 0;

    if ((NULL == p_name) || (NULL == p_payload))
    {
        result = E_PAR;
    }
    else
    {
        len = copy_text(text, p_payload, COMMS_PAYLOAD_MAX);

        if (len < 0)
        {
            result = E_PAR;
        }
        else
        {
            result = queue_message(g_mbf_comms_tlm, 0, COMMS_TOPIC_TELEMETRY,
                                   p_name, (const UB *)text, len);
        }
    }

    return result;
}

EXPORT void
comms_latch_event (UB events)
{
    UINT intsts;

    DI(intsts);
    g_comms_events |= events;
    EI(intsts);
}

LOCAL UB
take_events (void)
{
    UINT intsts;
    UB   events;

    DI(intsts);
    events         = g_comms_events;
    g_comms_events = 0;
    EI(intsts);
    return events;
}

EXPORT BOOL
comms_is_online (void)
{
    return g_comms_online;
}

EXPORT UW
comms_dropped (void)
{
    return g_comms_drops;
}

EXPORT void
comms_stats (comms_stats_t * p_out)
{
    *p_out = g_stats;
}

/* ---- hooks, called from the WiFi service task (comms_hooks.h) ---- */

int
comms_hook_next (char * p_topic, uint8_t * p_payload, int * p_qos)
{
    comms_msg_t msg;
    INT         index;
    int         result = 0;

    if ((tk_rcv_mbf(g_mbf_comms_evt, &msg, TMO_POL) < E_OK)
        && (tk_rcv_mbf(g_mbf_comms_tlm, &msg, TMO_POL) < E_OK))
    {
        result = -1;
    }
    else
    {
        *p_qos = msg.qos;
        (void)copy_text(p_topic, msg.topic, COMMS_TOPIC_MAX);

        for (index = 0; index < (INT)msg.len; index++)
        {
            p_payload[index] = msg.payload[index];
        }

        result = (int)msg.len;
    }

    return result;
}

/* TRUE if seq is after cmd_seq in 8-bit serial-number order. */
LOCAL BOOL
seq_newer (UB seq)
{
    UB diff = (UB)(seq - g_cmd_seq);

    return (0U != diff) && (diff < 128U);
}

void
comms_hook_command (const uint8_t * p_payload, int len)
{
    tf_command_t cmd;
    tf_ack_t     result;
    UB           ack[TF_ACK_BYTES];
    comms_cmd_t  forward;
    BOOL         same_session;

    if ((len < 0) || (!tf_cmd_decode(p_payload, (size_t)len, &cmd)))
    {
        /* Wrong length, truncated or reserved opcode. */
        result = TF_ACK_REJECTED;
        g_stats.cmd_rejected++;
        comms_latch_event(TF_EV_CMD_REJECTED);
    }
    else
    {
        same_session = g_cmd_seen && (cmd.session == g_cmd_session);

        if ((TF_OP_START == cmd.opcode) && same_session
            && (!seq_newer(cmd.seq)))
        {
            /* A redelivered or stale START must never restart the
               robot.  STOP/ESTOP are always forwarded. */
            result = TF_ACK_DUPLICATE;
            g_stats.cmd_duplicate++;
        }
        else
        {
            forward = (comms_cmd_t)cmd.opcode;

            if (tk_snd_mbf(g_mbf_comms_cmd, &forward, sizeof(forward), TMO_POL)
                < E_OK)
            {
                result = TF_ACK_BUSY; /* controller not reading */
                g_stats.cmd_busy++;
            }
            else
            {
                result = TF_ACK_OK;
                g_stats.cmd_ok++;

                if ((!same_session) || (seq_newer(cmd.seq)))
                {
                    g_cmd_seen    = TRUE;
                    g_cmd_session = cmd.session;
                    g_cmd_seq     = cmd.seq;
                }
            }
        }
    }

    tf_ack_encode(result, &cmd, ack);
    (void)queue_message(g_mbf_comms_evt, 1, COMMS_TOPIC_ACK, NULL, ack,
                        TF_ACK_BYTES);
}

void
comms_hook_route (const uint8_t * p_payload, int len)
{
    UB           reply[COMMS_PAYLOAD_MAX + 1];
    const char * p_head
        = ((len >= 0) && (route_set((const char *)p_payload, len)))
              ? "ok "
              : "rejected ";
    INT count = 0;
    INT index;

    while ('\0' != *p_head)
    {
        reply[count++] = (UB)*p_head++;
    }

    for (index = 0; (index < len) && (count < COMMS_PAYLOAD_MAX); index++)
    {
        reply[count++] = p_payload[index]; /* echo what was received */
    }

    reply[count] = '\0';
    tm_printf((UB *)"route: %s\n", reply);
    (void)queue_message(g_mbf_comms_evt, 1, COMMS_TOPIC_ROUTE_ACK, NULL, reply,
                        count);
}

void
comms_hook_online (int online)
{
    g_comms_online = (0 != online);
}

/* ---- task ---- */

LOCAL UW
elapsed_us (float64_t from, float64_t until)
{
    float64_t microseconds = (until - from) * 1.0e6;

    return (microseconds <= 0.0)
               ? 0U
               : ((microseconds >= 4.0e9) ? 0xFFFFFFFFU : (UW)microseconds);
}

LOCAL void
publish_events (void)
{
    static UB seq;
    UB        frame[TF_EVENT_BYTES];
    UB        events = take_events();

    if (0U == events)
    {
        return;
    }

    frame[0] = events;
    frame[1] = seq;

    if (E_OK
        == queue_message(g_mbf_comms_evt, 1, COMMS_TOPIC_EVENT, NULL, frame,
                         TF_EVENT_BYTES))
    {
        seq++;
        g_stats.events++;
    }
    else
    {
        comms_latch_event(events); /* keep them for the next period */
    }
}

LOCAL void
comms_task (INT stacd, void * p_exinf)
{
    tm_frame_info_t info = {0};
    int32_t         values[TF_FIELD_COUNT];
    UB              frame[TF_FRAME_BYTES];
    tf_encoder_t    enc;
    UB              enc_frame[TF_ENC_BYTES];
    BOOL            have_prev     = FALSE;
    UW              base_rejected = 0;
    UW              base_drops    = 0;

    float64_t time_start;
    float64_t time_read;
    float64_t time_queued;
    UW        elapsed;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        tk_dly_tsk(COMMS_STATUS_MS);

        /* Copy under the mutex (TMO_POL inside); it is released again
           before anything below runs. */
        time_start = motion_hal_time();

        if (!motion_get_status(&g_snap))
        {
            g_stats.skipped++;
            continue;
        }

        time_read = motion_hal_time();
        elapsed   = elapsed_us(time_start, time_read);

        if (elapsed > g_stats.copy_us_max)
        {
            g_stats.copy_us_max = elapsed;
        }

        if (have_prev)
        {
            comms_latch_event(tm_edge_events(&g_prev, &g_snap));
        }
        else
        {
            base_rejected = g_snap.rejected_requests;
            have_prev     = TRUE;
        }

        g_prev = g_snap;

        if (!g_comms_online)
        {
            continue; /* events stay latched until online */
        }

        info.uptime_s = time_read;
        info.rejected = g_snap.rejected_requests - base_rejected;
        info.dropped  = g_comms_drops - base_drops;
        tm_fill(&g_snap, &info, values);

        if (0U != tf_pack(values, frame))
        {
            g_stats.clamped++;
        }

        tm_fill_encoder(&g_snap, info.seq, time_read, &enc);

        if (tf_enc_pack(&enc, enc_frame))
        {
            g_stats.clamped++;
        }

        time_queued = motion_hal_time();
        elapsed     = elapsed_us(time_read, time_queued);

        if (elapsed > g_stats.pack_us_max)
        {
            g_stats.pack_us_max = elapsed;
        }

        if (E_OK
            == queue_message(g_mbf_comms_tlm, 0, COMMS_TOPIC_FRAME, NULL, frame,
                             TF_FRAME_BYTES))
        {
            info.seq++;
            g_stats.frames++;
            base_rejected = g_snap.rejected_requests;
            base_drops    = g_comms_drops;

            /* Sent only after its status frame, so a SEQ always
               pairs the two.  A failure counts in comms_drops. */
            if (E_OK
                == queue_message(g_mbf_comms_tlm, 0, COMMS_TOPIC_ENCODER, NULL,
                                 enc_frame, TF_ENC_BYTES))
            {
                g_stats.enc_frames++;
            }

            if (0U == (g_stats.frames % COMMS_REPORT_FRAMES))
            {
                tm_printf((UB *)"comms: frames=%u enc=%u skipped=%u clamped=%u "
                                "dropped=%u copy_us_max=%u pack_us_max=%u\n",
                          g_stats.frames, g_stats.enc_frames, g_stats.skipped,
                          g_stats.clamped, g_comms_drops, g_stats.copy_us_max,
                          g_stats.pack_us_max);
            }
        }

        publish_events();
    }
}

LOCAL ID
create_queue (INT depth, INT msgsz)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = depth * (msgsz + COMMS_MBF_OVERHEAD);
    cmbf.maxmsz = msgsz;
    return tk_cre_mbf(&cmbf);
}

EXPORT ER
comms_init (void)
{
    ER result = 0;

    g_mbf_comms_tlm = create_queue(COMMS_TLM_DEPTH, sizeof(comms_msg_t));

    if (g_mbf_comms_tlm < E_OK)
    {
        result = g_mbf_comms_tlm;
    }
    else
    {
        g_mbf_comms_evt = create_queue(COMMS_EVT_DEPTH, sizeof(comms_msg_t));

        if (g_mbf_comms_evt < E_OK)
        {
            result = g_mbf_comms_evt;
        }
        else
        {
            g_mbf_comms_cmd
                = create_queue(COMMS_CMD_DEPTH, sizeof(comms_cmd_t));
            result = (g_mbf_comms_cmd < E_OK) ? g_mbf_comms_cmd : E_OK;
        }
    }

    return result;
}

EXPORT ER
comms_start (void)
{
    ID tid = app_start_task(comms_task, S1_PRI_COMMS, 0, "comms_task");

    return (tid < E_OK) ? tid : E_OK;
}

/*** end of file ***/
