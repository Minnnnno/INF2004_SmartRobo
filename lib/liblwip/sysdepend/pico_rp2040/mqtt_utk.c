/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

/* Topics, size limits and the queue hooks of the comms subsystem (plain C
 * types only, so the note above about tk/tkernel.h still holds). */
#include "../../../../app_program/subsystem1/comms_hooks.h"

#define MQTT_UTK_TICK_MS        100     /* queue drain period */
#define MQTT_UTK_HEARTBEAT_TICKS 10     /* heartbeat every 1 s */
#define MQTT_UTK_TX_PER_TICK    4       /* queued messages published per tick */
#define MQTT_UTK_KEEPALIVE_S    10      /* broker reports "offline" ~15 s after loss */
#define MQTT_UTK_RETRY_MIN_MS   1000    /* reconnect backoff: 1, 2, 4, 8, 10 s */
#define MQTT_UTK_RETRY_MAX_MS   10000

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * heartbeat this demo publishes; anything longer is truncated and flagged
 * rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;
static u32_t		retry_ms = MQTT_UTK_RETRY_MIN_MS;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	(void)tot_len;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len; i++) {
		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
			/* The console is a terminal -- keep raw bytes from a
			 * payload this demo did not publish from scrambling it. */
			UB c = data[i];
			rx_data[rx_len++] = (c >= 0x20 && c < 0x7f)? c: '.';
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		rx_data[rx_len] = '\0';
		tm_printf((UB*)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
			  rx_truncated? " (truncated)": "");

		/* The command topic is the only subscription. A truncated
		 * message is handed over empty, so it is rejected. */
		comms_hook_command((const char*)rx_data, rx_truncated? 0: rx_len);
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " COMMS_TOPIC_CMD "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

/* Publishes what the comms subsystem has queued, and the heartbeat. While
 * the broker is unreachable nothing is taken out, so the queues hold the
 * messages (and the comms API drops new ones once they are full). */
static void tick_timeout(void *arg)
{
	static unsigned int	count = 0;
	static u8_t		ticks = 0;
	char			topic[COMMS_TOPIC_MAX];
	char			payload[COMMS_PAYLOAD_MAX];
	int			len, qos, i;

	(void)arg;

	if(mqtt_client_is_connected(client)) {
		for(i = 0; i < MQTT_UTK_TX_PER_TICK; i++) {
			len = comms_hook_next(topic, payload, &qos);
			if(len < 0) {
				break;
			}
			if(mqtt_publish(client, topic, payload, (u16_t)len,
					(u8_t)qos, 0 /* retain */,
					publish_done_cb, NULL) != ERR_OK) {
				/* Output buffer full: this message is lost,
				 * leave the rest for the next tick. */
				tm_printf((UB*)"mqtt: publish to %s dropped\n", topic);
				break;
			}
		}

		if(++ticks >= MQTT_UTK_HEARTBEAT_TICKS) {
			ticks = 0;
			len = tm_sprintf((UB*)payload, (const UB*)"hb %u", count++);
			(void)mqtt_publish(client, COMMS_TOPIC_HEARTBEAT, payload,
					   (u16_t)len, 0 /* qos */, 0 /* retain */,
					   publish_done_cb, NULL);
		}
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one tick chain alive at a time. */
	sys_timeout(MQTT_UTK_TICK_MS, tick_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

/* Waits retry_ms before the next connection attempt, then doubles the wait
 * up to the cap. connecting stays set meanwhile, so the poll loop's own
 * mqtt_utk_start() calls cannot cut the wait short. */
static void schedule_retry(void)
{
	connecting = 1;
	comms_hook_online(0);

	tm_printf((UB*)"mqtt: retrying in %d ms\n", (int)retry_ms);
	sys_untimeout(retry_timeout, NULL);
	sys_timeout(retry_ms, retry_timeout, NULL);

	retry_ms *= 2;
	if(retry_ms > MQTT_UTK_RETRY_MAX_MS) {
		retry_ms = MQTT_UTK_RETRY_MAX_MS;
	}
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		retry_ms = MQTT_UTK_RETRY_MIN_MS;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Callbacks must be registered before subscribing, or the
		 * first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, COMMS_TOPIC_CMD, 1 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* Retained, so a dashboard that connects later still sees it.
		 * The Last Will (mqtt_utk_start) replaces it with "offline". */
		(void)mqtt_publish(client, COMMS_TOPIC_STATUS, "online", 6,
				   1 /* qos */, 1 /* retain */,
				   publish_done_cb, NULL);
		comms_hook_online(1);

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating tick, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(tick_timeout, NULL);
		sys_timeout(MQTT_UTK_TICK_MS, tick_timeout, NULL);
	} else {
		/* Rejected, never established, or dropped. Retry on a delay,
		 * so a broker that keeps rejecting (e.g. bad client ID) can't
		 * turn this into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d\n", (int)status);
		schedule_retry();
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	/* Last Will: the broker publishes this for us if the connection is
	 * lost without a clean disconnect (power off, out of range). */
	client_info.will_topic = COMMS_TOPIC_STATUS;
	client_info.will_msg = "offline";
	client_info.will_qos = 1;
	client_info.will_retain = 1;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		schedule_retry();
	}
}
