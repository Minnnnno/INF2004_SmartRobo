/*
 *	comms.c
 *	Subsystem 1 -- telemetry, event and command queues.
 *
 *	Other subsystems fill the queues through comms.h.  The MQTT client
 *	(mqtt_utk.c, running in the kernel's WiFi service task) empties them
 *	through the hooks in comms_hooks.h.  Every queue access uses TMO_POL,
 *	so neither side can hold the other up.
 */

#include "subsystem1.h"

#define COMMS_TLM_DEPTH		8	/* queued telemetry messages */
#define COMMS_EVT_DEPTH		4	/* queued events and acks */
#define COMMS_CMD_DEPTH		4	/* commands waiting for the controller */
#define COMMS_MBF_OVERHEAD	8	/* kernel bookkeeping per message, bytes */
#define COMMS_STATUS_MS		1000	/* comms_task publishing period */

typedef struct {
	UB	qos;
	char	topic[COMMS_TOPIC_MAX];
	char	payload[COMMS_PAYLOAD_MAX];
} COMMS_MSG;

EXPORT ID	mbf_comms_cmd;		/* IF-07, see comms.h */

LOCAL ID		mbf_comms_tlm;
LOCAL ID		mbf_comms_evt;
LOCAL volatile BOOL	comms_online;
LOCAL volatile UW	comms_drops;	/* diagnostic only, not exact on two cores */

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

LOCAL BOOL text_is(const char *text, INT len, const char *word)
{
	INT	i;

	for (i = 0; i < len; i++) {
		if (word[i] == '\0' || text[i] != word[i]) {
			return FALSE;
		}
	}
	return (word[len] == '\0');
}

LOCAL ER queue_message(ID mbf, UB qos, const char *topic, const char *name,
		       const char *payload)
{
	COMMS_MSG	msg;
	INT		len;
	ER		er;

	if (payload == NULL) {
		return E_PAR;
	}

	len = copy_text(msg.topic, topic, COMMS_TOPIC_MAX);
	if (name != NULL) {
		if (name[0] == '\0' ||
		    copy_text(&msg.topic[len], name, COMMS_NAME_MAX) < 0) {
			return E_PAR;
		}
	}
	if (copy_text(msg.payload, payload, COMMS_PAYLOAD_MAX) < 0) {
		return E_PAR;
	}
	msg.qos = qos;

	er = tk_snd_mbf(mbf, &msg, sizeof(msg), TMO_POL);
	if (er < E_OK) {
		comms_drops++;
	}
	return er;
}

EXPORT ER comms_send_telemetry(const char *name, const char *payload)
{
	if (name == NULL) {
		return E_PAR;
	}
	return queue_message(mbf_comms_tlm, 0, COMMS_TOPIC_TELEMETRY, name, payload);
}

EXPORT ER comms_send_event(const char *payload)
{
	return queue_message(mbf_comms_evt, 1, COMMS_TOPIC_EVENT, NULL, payload);
}

EXPORT BOOL comms_is_online(void)
{
	return comms_online;
}

EXPORT UW comms_dropped(void)
{
	return comms_drops;
}

/* ---- hooks, called from the WiFi service task (comms_hooks.h) ---- */

int comms_hook_next(char *topic, char *payload, int *qos)
{
	COMMS_MSG	msg;

	if (tk_rcv_mbf(mbf_comms_evt, &msg, TMO_POL) < E_OK &&
	    tk_rcv_mbf(mbf_comms_tlm, &msg, TMO_POL) < E_OK) {
		return -1;
	}

	*qos = msg.qos;
	(void)copy_text(topic, msg.topic, COMMS_TOPIC_MAX);
	return copy_text(payload, msg.payload, COMMS_PAYLOAD_MAX);
}

void comms_hook_command(const char *payload, int len)
{
	COMMS_CMD	cmd;
	const char	*ack;

	if (text_is(payload, len, "start")) {
		cmd = COMMS_CMD_START;
		ack = "ok start";
	} else if (text_is(payload, len, "stop")) {
		cmd = COMMS_CMD_STOP;
		ack = "ok stop";
	} else if (text_is(payload, len, "estop")) {
		cmd = COMMS_CMD_ESTOP;
		ack = "ok estop";
	} else {
		(void)queue_message(mbf_comms_evt, 1, COMMS_TOPIC_ACK, NULL, "rejected");
		return;
	}

	if (tk_snd_mbf(mbf_comms_cmd, &cmd, sizeof(cmd), TMO_POL) < E_OK) {
		ack = "busy";	/* the controller is not reading its commands */
	}
	(void)queue_message(mbf_comms_evt, 1, COMMS_TOPIC_ACK, NULL, ack);
}

void comms_hook_online(int online)
{
	comms_online = (online != 0);
}

/* ---- task ---- */

/* Publishes this subsystem's own status once a second.  The team's shared
   robot status will be read and published from here too. */
LOCAL void comms_task(INT stacd, void *exinf)
{
	UB	text[COMMS_PAYLOAD_MAX];

	(void)stacd; (void)exinf;

	while (1) {
		tk_dly_tsk(COMMS_STATUS_MS);

		tm_sprintf(text, (const UB*)"online=%d dropped=%u",
			   comms_is_online(), comms_dropped());
		(void)comms_send_telemetry("comms", (const char*)text);
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
