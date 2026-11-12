/*
 *	comms.c
 *	Subsystem 1 -- status frame, events, commands and their queues.
 *
 *	comms_task (lowest application priority) copies the shared robot status
 *	every 100 ms with motion_get_status(), which holds the priority-
 *	inheritance bus mutex only for the copy and never waits for it.  The
 *	frame is then built and packed with no lock held, and queued.  The MQTT
 *	client (mqtt_utk.c, in the kernel's WiFi service task) empties the
 *	queues through the hooks in comms_hooks.h.  Every queue access uses
 *	TMO_POL, so neither side can hold the other up, and nothing here can
 *	delay the 10 ms motion task beyond one snapshot copy.
 */

#include "subsystem1.h"		/* first: sets up size_t for the kernel */
#include "robot.h"
#include "motion_hal.h"
#include "telemetry_map.h"

#define COMMS_TLM_DEPTH		8	/* queued frames / diagnostics */
#define COMMS_EVT_DEPTH		4	/* queued events and acks */
#define COMMS_CMD_DEPTH		4	/* commands waiting for the controller */
#define COMMS_MBF_OVERHEAD	8	/* kernel bookkeeping per message, bytes */
#define COMMS_STATUS_MS		100	/* status frame period: 10 Hz */
#define COMMS_REPORT_FRAMES	100	/* console statistics every 10 s */

_Static_assert((int)COMMS_CMD_START == (int)TF_OP_START &&
	       (int)COMMS_CMD_STOP == (int)TF_OP_STOP &&
	       (int)COMMS_CMD_ESTOP == (int)TF_OP_ESTOP,
	       "COMMS_CMD must equal the binary opcodes");
_Static_assert(TF_FRAME_BYTES <= COMMS_PAYLOAD_MAX, "frame does not fit a message");
_Static_assert(TF_ENC_BYTES <= COMMS_PAYLOAD_MAX, "encoder frame does not fit a message");

typedef struct {
	UB	qos;
	UB	len;
	char	topic[COMMS_TOPIC_MAX];
	UB	payload[COMMS_PAYLOAD_MAX];
} COMMS_MSG;

EXPORT ID	mbf_comms_cmd;		/* IF-07, see comms.h */

LOCAL ID		mbf_comms_tlm;
LOCAL ID		mbf_comms_evt;
LOCAL volatile BOOL	comms_online;
LOCAL volatile UW	comms_drops;	/* diagnostic only */
LOCAL volatile UB	comms_events;	/* latched TF_EV_* bits; DI/EI only */
LOCAL COMMS_STATS	stats;

/* Command de-duplication; touched only by comms_hook_command(). */
LOCAL BOOL		cmd_seen;
LOCAL UB		cmd_session;
LOCAL UB		cmd_seq;

/* comms_task's snapshots: static to keep them off its 2 KB stack. */
LOCAL robot_status_t	snap;
LOCAL robot_status_t	prev;

/* Copy a string into dst[max].  Returns its length, or -1 if it does not fit. */
LOCAL INT copy_text(char *dst, const char *src, INT max)
{
	INT	i;

	for (i = 0; i < max; i++) {
		dst[i] = src[i];
		if (src[i] == '\0') {
			return i;
		}
	}
	return -1;
}

LOCAL ER queue_message(ID mbf, UB qos, const char *topic, const char *name,
		       const UB *data, INT len)
{
	COMMS_MSG	msg;
	INT		tlen, i;
	ER		er;

	if ((data == NULL) || (len < 0) || (len > COMMS_PAYLOAD_MAX)) {
		return E_PAR;
	}
	tlen = copy_text(msg.topic, topic, COMMS_TOPIC_MAX);
	if (name != NULL) {
		if (name[0] == '\0' ||
		    copy_text(&msg.topic[tlen], name, COMMS_NAME_MAX) < 0) {
			return E_PAR;
		}
	}
	for (i = 0; i < len; i++) {
		msg.payload[i] = data[i];
	}
	msg.len = (UB)len;
	msg.qos = qos;

	er = tk_snd_mbf(mbf, &msg, sizeof(msg), TMO_POL);
	if (er < E_OK) {
		comms_drops++;
	}
	return er;
}

EXPORT ER comms_send_telemetry(const char *name, const char *payload)
{
	char	text[COMMS_PAYLOAD_MAX];
	INT	len;

	if ((name == NULL) || (payload == NULL)) {
		return E_PAR;
	}
	len = copy_text(text, payload, COMMS_PAYLOAD_MAX);
	if (len < 0) {
		return E_PAR;
	}
	return queue_message(mbf_comms_tlm, 0, COMMS_TOPIC_TELEMETRY, name,
			     (const UB*)text, len);
}

EXPORT void comms_latch_event(UB events)
{
	UINT	intsts;

	DI(intsts);
	comms_events |= events;
	EI(intsts);
}

LOCAL UB take_events(void)
{
	UINT	intsts;
	UB	events;

	DI(intsts);
	events = comms_events;
	comms_events = 0;
	EI(intsts);
	return events;
}

EXPORT BOOL comms_is_online(void)
{
	return comms_online;
}

EXPORT UW comms_dropped(void)
{
	return comms_drops;
}

EXPORT void comms_stats(COMMS_STATS *out)
{
	*out = stats;
}

/* ---- hooks, called from the WiFi service task (comms_hooks.h) ---- */

int comms_hook_next(char *topic, unsigned char *payload, int *qos)
{
	COMMS_MSG	msg;
	INT		i;

	if (tk_rcv_mbf(mbf_comms_evt, &msg, TMO_POL) < E_OK &&
	    tk_rcv_mbf(mbf_comms_tlm, &msg, TMO_POL) < E_OK) {
		return -1;
	}

	*qos = msg.qos;
	(void)copy_text(topic, msg.topic, COMMS_TOPIC_MAX);
	for (i = 0; i < (INT)msg.len; i++) {
		payload[i] = msg.payload[i];
	}
	return (int)msg.len;
}

/* TRUE if seq is after cmd_seq in 8-bit serial-number order. */
LOCAL BOOL seq_newer(UB seq)
{
	UB diff = (UB)(seq - cmd_seq);

	return (diff != 0U) && (diff < 128U);
}

void comms_hook_command(const unsigned char *payload, int len)
{
	tf_command_t	cmd;
	tf_ack_t	result;
	UB		ack[TF_ACK_BYTES];
	COMMS_CMD	forward;
	BOOL		same_session;

	if ((len < 0) || !tf_cmd_decode(payload, (size_t)len, &cmd)) {
		/* Wrong length, truncated or reserved opcode. */
		result = TF_ACK_REJECTED;
		stats.cmd_rejected++;
		comms_latch_event(TF_EV_CMD_REJECTED);
	} else {
		same_session = cmd_seen && (cmd.session == cmd_session);
		if ((cmd.opcode == TF_OP_START) && same_session && !seq_newer(cmd.seq)) {
			/* A redelivered or stale START must never restart the
			   robot.  STOP/ESTOP are always forwarded. */
			result = TF_ACK_DUPLICATE;
			stats.cmd_duplicate++;
		} else {
			forward = (COMMS_CMD)cmd.opcode;
			if (tk_snd_mbf(mbf_comms_cmd, &forward, sizeof(forward), TMO_POL) < E_OK) {
				result = TF_ACK_BUSY;	/* controller not reading */
				stats.cmd_busy++;
			} else {
				result = TF_ACK_OK;
				stats.cmd_ok++;
				if (!same_session || seq_newer(cmd.seq)) {
					cmd_seen = TRUE;
					cmd_session = cmd.session;
					cmd_seq = cmd.seq;
				}
			}
		}
	}

	tf_ack_encode(result, &cmd, ack);
	(void)queue_message(mbf_comms_evt, 1, COMMS_TOPIC_ACK, NULL, ack, TF_ACK_BYTES);
}

void comms_hook_online(int online)
{
	comms_online = (online != 0);
}

/* ---- task ---- */

LOCAL UW elapsed_us(double from, double to)
{
	double us = (to - from) * 1.0e6;

	return (us <= 0.0) ? 0U : ((us >= 4.0e9) ? 0xFFFFFFFFU : (UW)us);
}

LOCAL void publish_events(void)
{
	static UB	seq;
	UB		frame[TF_EVENT_BYTES];
	UB		events = take_events();

	if (events == 0U) {
		return;
	}
	frame[0] = events;
	frame[1] = seq;
	if (queue_message(mbf_comms_evt, 1, COMMS_TOPIC_EVENT, NULL,
			  frame, TF_EVENT_BYTES) == E_OK) {
		seq++;
		stats.events++;
	} else {
		comms_latch_event(events);	/* keep them for the next period */
	}
}

LOCAL void comms_task(INT stacd, void *exinf)
{
	tm_frame_info_t	info = {0};
	int32_t		values[TF_FIELD_COUNT];
	UB		frame[TF_FRAME_BYTES];
	tf_encoder_t	enc;
	UB		enc_frame[TF_ENC_BYTES];
	BOOL		have_prev = FALSE;
	UW		base_rejected = 0, base_drops = 0;
	double		t0, t1, t2;
	UW		us;

	(void)stacd; (void)exinf;

	while (1) {
		tk_dly_tsk(COMMS_STATUS_MS);

		/* Copy under the mutex (TMO_POL inside); it is released again
		   before anything below runs. */
		t0 = motion_hal_time();
		if (!motion_get_status(&snap)) {
			stats.skipped++;
			continue;
		}
		t1 = motion_hal_time();
		us = elapsed_us(t0, t1);
		if (us > stats.copy_us_max) {
			stats.copy_us_max = us;
		}

		if (have_prev) {
			comms_latch_event(tm_edge_events(&prev, &snap));
		} else {
			base_rejected = snap.rejected_requests;
			have_prev = TRUE;
		}
		prev = snap;

		if (!comms_online) {
			continue;	/* events stay latched until online */
		}

		info.uptime_s = t1;
		info.rejected = snap.rejected_requests - base_rejected;
		info.dropped = comms_drops - base_drops;
		tm_fill(&snap, &info, values);
		if (tf_pack(values, frame) != 0U) {
			stats.clamped++;
		}
		tm_fill_encoder(&snap, info.seq, t1, &enc);
		if (tf_enc_pack(&enc, enc_frame)) {
			stats.clamped++;
		}
		t2 = motion_hal_time();
		us = elapsed_us(t1, t2);
		if (us > stats.pack_us_max) {
			stats.pack_us_max = us;
		}

		if (queue_message(mbf_comms_tlm, 0, COMMS_TOPIC_FRAME, NULL,
				  frame, TF_FRAME_BYTES) == E_OK) {
			info.seq++;
			stats.frames++;
			base_rejected = snap.rejected_requests;
			base_drops = comms_drops;
			/* Sent only after its status frame, so a SEQ always
			   pairs the two.  A failure counts in comms_drops. */
			if (queue_message(mbf_comms_tlm, 0, COMMS_TOPIC_ENCODER, NULL,
					  enc_frame, TF_ENC_BYTES) == E_OK) {
				stats.enc_frames++;
			}
			if ((stats.frames % COMMS_REPORT_FRAMES) == 0U) {
				tm_printf((UB*)"comms: frames=%u enc=%u skipped=%u clamped=%u "
					  "dropped=%u copy_us_max=%u pack_us_max=%u\n",
					  stats.frames, stats.enc_frames, stats.skipped, stats.clamped,
					  comms_drops, stats.copy_us_max, stats.pack_us_max);
			}
		}
		publish_events();
	}
}

LOCAL ID create_queue(INT depth, INT msgsz)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = depth * (msgsz + COMMS_MBF_OVERHEAD);
	cmbf.maxmsz = msgsz;
	return tk_cre_mbf(&cmbf);
}

EXPORT ER comms_init(void)
{
	mbf_comms_tlm = create_queue(COMMS_TLM_DEPTH, sizeof(COMMS_MSG));
	if (mbf_comms_tlm < E_OK) {
		return mbf_comms_tlm;
	}
	mbf_comms_evt = create_queue(COMMS_EVT_DEPTH, sizeof(COMMS_MSG));
	if (mbf_comms_evt < E_OK) {
		return mbf_comms_evt;
	}
	mbf_comms_cmd = create_queue(COMMS_CMD_DEPTH, sizeof(COMMS_CMD));
	return (mbf_comms_cmd < E_OK) ? mbf_comms_cmd : E_OK;
}

EXPORT ER comms_start(void)
{
	ID tid = app_start_task(comms_task, S1_PRI_COMMS, 0, "comms_task");
	return (tid < E_OK) ? tid : E_OK;
}
