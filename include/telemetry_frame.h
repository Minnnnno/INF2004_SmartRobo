/*
 *	telemetry_frame.h
 *	Shared binary MQTT formats: status frame, command, acknowledgement and
 *	event frame.  Owned by Buddy 1; changed only by team agreement
 *	(Week 8 Tribunal 1 decision log: "Pack all fields into one bit-level
 *	status frame").
 *
 *	This is the ONE definition of every field's range, bit width and sign.
 *	The firmware codec (app_program/subsystem1/telemetry_frame.c), the host
 *	round-trip test (tests/buddy1_frame) and the dashboard decoder
 *	(tools/dashboard.py, which parses TF_FIELDS below) all read this table.
 *	Plain C only: no kernel headers, so mqtt_utk.c and host tests can use it.
 *
 *	Encoding rules
 *	  - A field stores the unsigned offset (value - min) in `bits` bits.
 *	    Signed values therefore never wrap: -1 with min -250 is sent as 249,
 *	    and the decoder adds min back.
 *	  - Out-of-range values are CLAMPED to [min, max], never wrapped.
 *	    Counters (SEQ, UPTIME_DS, CMD_ID) wrap by design before packing.
 *	  - Fields are written most-significant bit first, in table order,
 *	    starting at bit 7 of byte 0.  Spare bits at the end are zero.
 *	  - Each field uses the minimum number of bits for its range; this is
 *	    checked at compile time in telemetry_frame.c.
 *	  - Bump TF_LAYOUT_VERSION whenever a field is added, removed or resized.
 *	    New fields go into the spare bits first, so the size stays 21 bytes.
 */

#ifndef TELEMETRY_FRAME_H
#define TELEMETRY_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TF_LAYOUT_VERSION	1U

/* ---------------------------------------------------------------------
 *  Status frame, published at QoS 0 on COMMS_TOPIC_FRAME every 100 ms.
 *
 *  X(name, min, max, bits, owner, "units and meaning")
 *  Keep each entry on ONE line: tools/dashboard.py parses these lines.
 * --------------------------------------------------------------------- */
#define TF_FIELDS(X) \
	X(VERSION,     0,     15,     4,  B1, "frame layout version, TF_LAYOUT_VERSION") \
	X(SEQ,         0,     255,    8,  B1, "frame counter, wraps; gaps = lost frames") \
	X(UPTIME_DS,   0,     65535,  16, B1, "uptime in 0.1 s, wraps every 109 min") \
	X(M_STATE,     0,     6,      3,  B2, "motion_state_t") \
	X(M_MODE,      0,     4,      3,  B2, "motion_mode_t") \
	X(M_FAULT,     0,     12,     4,  B2, "motion_fault_t (primary cause)") \
	X(M_FLAGS,     0,     15,     4,  B2, "bit0 stationary, bit1 feedback trusted, bit2 inhibited, bit3 hold") \
	X(OWNER,       0,     7,      3,  B2, "request_source_t owning motion; 7 = no owner") \
	X(CMD_ID,      0,     255,    8,  B2, "active command id, low 8 bits") \
	X(SPEED_L,     -250,  250,    9,  B2, "left wheel speed, mm/s") \
	X(SPEED_R,     -250,  250,    9,  B2, "right wheel speed, mm/s") \
	X(CENTRE_MM,   -5000, 5000,   14, B2, "signed centre travel, mm") \
	X(HEADING,     -180,  179,    9,  B2, "odometry heading, degrees, wrapped") \
	X(LINE_AVAIL,  0,     1,      1,  B3, "line status available") \
	X(LINE_ERR,    -1,    1,      2,  B3, "line error: -1 left, 0 centred, +1 right") \
	X(LINE_TRACK,  0,     1,      1,  B3, "line trackable") \
	X(BC_VALID,    0,     1,      1,  B3, "barcode instruction valid") \
	X(BC_CMD,      0,     3,      2,  B3, "tf_barcode_t: left, right, straight, U-turn") \
	X(T_VALID,     0,     7,      3,  B4, "bit0 heading, bit1 pitch, bit2 hump height valid") \
	X(T_STATE,     0,     6,      3,  B4, "terrain state (7 states, B4 statechart)") \
	X(PITCH,       -90,   90,     8,  B4, "pitch, degrees, + nose up") \
	X(T_HEADING,   0,     359,    9,  B4, "magnetic heading, degrees") \
	X(HUMP_MM,     0,     255,    8,  B4, "peak hump height, mm") \
	X(O_FLAGS,     0,     31,     5,  B5, "bit0 scanning, bit1 first boundary, bit2 second boundary, bit3 width valid, bit4 no echo") \
	X(O_AVAIL,     0,     1,      1,  B5, "obstacle status available") \
	X(O_DIST_CM,   0,     400,    9,  B5, "nearest obstacle distance, cm") \
	X(O_WIDTH_CM,  0,     255,    8,  B5, "obstacle width, cm; use only if width valid") \
	X(REJECTED,    0,     7,      3,  B1, "controller rejections since last frame, saturates") \
	X(DROPPED,     0,     7,      3,  B1, "comms messages dropped since last frame, saturates")

#define TF_ENUM_(name, lo, hi, bits, owner, text)	TF_F_##name,
enum { TF_FIELDS(TF_ENUM_) TF_FIELD_COUNT };
#undef TF_ENUM_

#define TF_SUM_(name, lo, hi, bits, owner, text)	+ (bits)
enum {
	TF_FRAME_BITS  = 0 TF_FIELDS(TF_SUM_),
	TF_FRAME_BYTES = (TF_FRAME_BITS + 7) / 8,
	TF_SPARE_BITS  = TF_FRAME_BYTES * 8 - TF_FRAME_BITS
};
#undef TF_SUM_

typedef struct {
	const char	*name;
	int32_t		min;
	int32_t		max;
	uint8_t		bits;
} tf_field_t;

extern const tf_field_t tf_fields[TF_FIELD_COUNT];

/* M_FLAGS, T_VALID and O_FLAGS bits */
#define TF_MF_STATIONARY	0x1U
#define TF_MF_TRUSTED		0x2U
#define TF_MF_INHIBITED		0x4U
#define TF_MF_HOLD		0x8U

#define TF_TV_HEADING		0x1U
#define TF_TV_PITCH		0x2U	/* = S4_VALID_PITCH in subsystem4.c */
#define TF_TV_HUMP		0x4U

#define TF_OF_SCANNING		0x01U
#define TF_OF_FIRST_EDGE	0x02U
#define TF_OF_SECOND_EDGE	0x04U
#define TF_OF_WIDTH_VALID	0x08U
#define TF_OF_NO_ECHO		0x10U

#define TF_OWNER_NONE		7

/* BC_CMD values.  Mapping still to be confirmed with the controller owner. */
typedef enum { TF_BC_LEFT, TF_BC_RIGHT, TF_BC_STRAIGHT, TF_BC_UTURN } tf_barcode_t;

/* Pack values[TF_FIELD_COUNT] into out[TF_FRAME_BYTES].  Out-of-range values
   are clamped.  Returns a mask with bit i set if field i was clamped. */
uint32_t tf_pack(const int32_t values[TF_FIELD_COUNT], uint8_t out[TF_FRAME_BYTES]);

/* Unpack a received frame.  Returns false (values untouched) if len is
   wrong, the version differs, a code is above its field's range or a
   spare bit is set. */
bool tf_unpack(const uint8_t *frame, size_t len, int32_t values[TF_FIELD_COUNT]);

/* ---------------------------------------------------------------------
 *  Command, received on COMMS_TOPIC_CMD at QoS 1.  2 bytes, big-endian:
 *    bits 15..13  opcode   (tf_opcode_t; 0 and 4..7 are rejected)
 *    bits 12..8   session  (picked at random by each dashboard start)
 *    bits  7..0   seq      (incremented per command by the dashboard)
 *  Within one session, a START whose seq is not newer than the last
 *  accepted command (8-bit serial order) is acknowledged as duplicate and
 *  not forwarded, so a QoS 1 redelivery or a stale START cannot restart
 *  the robot.  STOP and ESTOP are always forwarded: repeating them is
 *  harmless.  A new session number is accepted and becomes current.
 * --------------------------------------------------------------------- */
#define TF_CMD_BYTES		2
typedef enum { TF_OP_NONE, TF_OP_START, TF_OP_STOP, TF_OP_ESTOP } tf_opcode_t;
typedef struct { uint8_t opcode, session, seq; } tf_command_t;

bool tf_cmd_decode(const uint8_t *data, size_t len, tf_command_t *cmd);
void tf_cmd_encode(const tf_command_t *cmd, uint8_t out[TF_CMD_BYTES]);

/* ---------------------------------------------------------------------
 *  Acknowledgement, published on COMMS_TOPIC_ACK at QoS 1.  2 bytes:
 *    bits 15..14  result  (tf_ack_t)
 *    bits 13..11  opcode  (as received; 0 if the length was wrong)
 *    bits 10..3   seq     (as received)
 *    bits  2..0   spare, zero
 * --------------------------------------------------------------------- */
#define TF_ACK_BYTES		2
typedef enum { TF_ACK_OK, TF_ACK_DUPLICATE, TF_ACK_BUSY, TF_ACK_REJECTED } tf_ack_t;

void tf_ack_encode(tf_ack_t result, const tf_command_t *cmd, uint8_t out[TF_ACK_BYTES]);
bool tf_ack_decode(const uint8_t *data, size_t len, tf_ack_t *result, tf_command_t *cmd);

/* ---------------------------------------------------------------------
 *  Event frame, published on COMMS_TOPIC_EVENT at QoS 1.  2 bytes:
 *    byte 0  event mask (TF_EV_*), every event latched since the last one
 *    byte 1  event frame counter, wraps
 *  Events are latched bits, not a queue: a burst of the same event is
 *  reported once, and nothing is lost while the broker is unreachable.
 * --------------------------------------------------------------------- */
#define TF_EVENT_BYTES		2
#define TF_EV_MOTION_FAULT	0x01U	/* B1 detects from the snapshot */
#define TF_EV_LINE_LOST		0x02U	/* B1 detects from the snapshot */
#define TF_EV_BARCODE		0x04U	/* B1 detects from the snapshot */
#define TF_EV_HUMP_PEAK		0x08U	/* B4 latches when a peak completes */
#define TF_EV_OBSTACLE		0x10U	/* B5 latches on detection */
#define TF_EV_SCAN_DONE		0x20U	/* B5 latches when a scan result is ready */
#define TF_EV_HANDOVER		0x40U	/* B5 latches on B5 -> B3 recovery handover */
#define TF_EV_CMD_REJECTED	0x80U	/* B1 latches on a malformed command */

#endif /* TELEMETRY_FRAME_H */
