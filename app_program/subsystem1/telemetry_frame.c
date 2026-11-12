/*
 *	telemetry_frame.c
 *	Subsystem 1 -- bit-level codec for the formats in telemetry_frame.h.
 *	Plain C, no kernel calls: also built by the host test.
 */

#include "telemetry_frame.h"

/* Minimum-bit rule, checked for every field: the range fits in `bits`
   bits but would not fit in one bit fewer. */
#define TF_CHECK_(name, lo, hi, bits, owner, text)				\
	_Static_assert((hi) > (lo), #name ": empty range");			\
	_Static_assert(((int64_t)(hi) - (lo)) < ((int64_t)1 << (bits)),	\
		       #name ": too few bits for its range");			\
	_Static_assert(((int64_t)(hi) - (lo)) >= ((int64_t)1 << ((bits) - 1)),	\
		       #name ": more bits than its range needs");
TF_FIELDS(TF_CHECK_)
#undef TF_CHECK_

/* Agreed in the Tribunal: 161 bits padded to 21 bytes, 7 spare bits. */
_Static_assert(TF_FRAME_BYTES == 21, "status frame must stay 21 bytes");
_Static_assert(TF_SPARE_BITS == 7, "status frame spare bits changed");
_Static_assert(TF_FIELD_COUNT <= 32, "clamp mask holds 32 fields");

#define TF_ROW_(name, lo, hi, bits, owner, text)	{ #name, (lo), (hi), (bits) },
const tf_field_t tf_fields[TF_FIELD_COUNT] = { TF_FIELDS(TF_ROW_) };
#undef TF_ROW_

/* Write the low `bits` bits of code, most significant first, at bit *pos. */
static void put_bits(uint8_t *buf, unsigned *pos, uint32_t code, unsigned bits)
{
	while (bits > 0U) {
		bits--;
		if (((code >> bits) & 1U) != 0U) {
			buf[*pos >> 3] |= (uint8_t)(0x80U >> (*pos & 7U));
		}
		(*pos)++;
	}
}

static uint32_t get_bits(const uint8_t *buf, unsigned *pos, unsigned bits)
{
	uint32_t code = 0U;

	while (bits > 0U) {
		bits--;
		code = (code << 1) | (((uint32_t)buf[*pos >> 3] >> (7U - (*pos & 7U))) & 1U);
		(*pos)++;
	}
	return code;
}

uint32_t tf_pack(const int32_t values[TF_FIELD_COUNT], uint8_t out[TF_FRAME_BYTES])
{
	uint32_t	clamped = 0U;
	unsigned	pos = 0U;
	unsigned	i;

	for (i = 0U; i < (unsigned)TF_FRAME_BYTES; i++) {
		out[i] = 0U;
	}
	for (i = 0U; i < (unsigned)TF_FIELD_COUNT; i++) {
		const tf_field_t	*f = &tf_fields[i];
		int32_t			v = values[i];

		if (v < f->min) {
			v = f->min;
			clamped |= 1UL << i;
		} else if (v > f->max) {
			v = f->max;
			clamped |= 1UL << i;
		}
		put_bits(out, &pos, (uint32_t)(v - f->min), f->bits);
	}
	return clamped;
}

bool tf_unpack(const uint8_t *frame, size_t len, int32_t values[TF_FIELD_COUNT])
{
	int32_t		tmp[TF_FIELD_COUNT];
	unsigned	pos = 0U;
	unsigned	i;

	if ((frame == NULL) || (len != (size_t)TF_FRAME_BYTES)) {
		return false;
	}
	for (i = 0U; i < (unsigned)TF_FIELD_COUNT; i++) {
		const tf_field_t	*f = &tf_fields[i];
		uint32_t		code = get_bits(frame, &pos, f->bits);

		if (code > (uint32_t)(f->max - f->min)) {
			return false;	/* not a value the sender can produce */
		}
		tmp[i] = (int32_t)code + f->min;
	}
	if (get_bits(frame, &pos, (unsigned)TF_SPARE_BITS) != 0U) {
		return false;
	}
	if (tmp[TF_F_VERSION] != (int32_t)TF_LAYOUT_VERSION) {
		return false;
	}
	for (i = 0U; i < (unsigned)TF_FIELD_COUNT; i++) {
		values[i] = tmp[i];
	}
	return true;
}

bool tf_cmd_decode(const uint8_t *data, size_t len, tf_command_t *cmd)
{
	uint32_t word;

	cmd->opcode = 0U;
	cmd->session = 0U;
	cmd->seq = 0U;
	if ((data == NULL) || (len != (size_t)TF_CMD_BYTES)) {
		return false;
	}
	word = ((uint32_t)data[0] << 8) | data[1];
	cmd->opcode = (uint8_t)((word >> 13) & 0x7U);
	cmd->session = (uint8_t)((word >> 8) & 0x1FU);
	cmd->seq = (uint8_t)(word & 0xFFU);
	return (cmd->opcode >= (uint8_t)TF_OP_START) && (cmd->opcode <= (uint8_t)TF_OP_ESTOP);
}

void tf_cmd_encode(const tf_command_t *cmd, uint8_t out[TF_CMD_BYTES])
{
	uint32_t word = (((uint32_t)cmd->opcode & 0x7U) << 13) |
			(((uint32_t)cmd->session & 0x1FU) << 8) | cmd->seq;

	out[0] = (uint8_t)(word >> 8);
	out[1] = (uint8_t)(word & 0xFFU);
}

void tf_ack_encode(tf_ack_t result, const tf_command_t *cmd, uint8_t out[TF_ACK_BYTES])
{
	uint32_t word = (((uint32_t)result & 0x3U) << 14) |
			(((uint32_t)cmd->opcode & 0x7U) << 11) |
			((uint32_t)cmd->seq << 3);

	out[0] = (uint8_t)(word >> 8);
	out[1] = (uint8_t)(word & 0xFFU);
}

static void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24);
	p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8);
	p[3] = (uint8_t)v;
}

static uint32_t get_u32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8) | p[3];
}

static uint8_t clamp_u8(uint8_t v, unsigned max, bool *clamped)
{
	if (v > max) {
		*clamped = true;
		return (uint8_t)max;
	}
	return v;
}

bool tf_enc_pack(const tf_encoder_t *e, uint8_t out[TF_ENC_BYTES])
{
	bool	clamped = false;
	uint8_t	cause = clamp_u8(e->cause, TF_EF_CAUSE_MAX, &clamped);
	uint8_t	state = clamp_u8(e->state, TF_ENC_STATE_MAX, &clamped);
	uint8_t	fault = clamp_u8(e->fault, TF_ENC_FAULT_MAX, &clamped);

	out[0] = (uint8_t)TF_ENC_VERSION;
	out[1] = e->seq;
	out[2] = (uint8_t)((e->reliable ? TF_EF_RELIABLE : 0U) |
			   (e->trusted ? TF_EF_TRUSTED : 0U) |
			   (e->fresh ? TF_EF_FRESH : 0U) |
			   ((unsigned)cause << TF_EF_CAUSE_SHIFT));
	out[3] = (uint8_t)(((unsigned)state << 4) | fault);
	put_u32(&out[4], e->sample_ms);
	out[8] = (uint8_t)(e->age_ms >> 8);
	out[9] = (uint8_t)e->age_ms;
	put_u32(&out[10], e->count[0]);
	put_u32(&out[14], e->count[1]);
	put_u32(&out[18], e->invalid[0]);
	put_u32(&out[22], e->invalid[1]);
	return clamped;
}

bool tf_enc_unpack(const uint8_t *frame, size_t len, tf_encoder_t *e)
{
	unsigned cause, state, fault;

	if ((frame == NULL) || (len != (size_t)TF_ENC_BYTES) ||
	    (frame[0] != (uint8_t)TF_ENC_VERSION) || ((frame[2] & 0xC0U) != 0U)) {
		return false;
	}
	cause = ((unsigned)frame[2] >> TF_EF_CAUSE_SHIFT) & 0x7U;
	state = (unsigned)frame[3] >> 4;
	fault = (unsigned)frame[3] & 0xFU;
	if ((cause > TF_EF_CAUSE_MAX) || (state > TF_ENC_STATE_MAX) ||
	    (fault > TF_ENC_FAULT_MAX)) {
		return false;
	}
	e->seq = frame[1];
	e->reliable = (frame[2] & TF_EF_RELIABLE) != 0U;
	e->trusted = (frame[2] & TF_EF_TRUSTED) != 0U;
	e->fresh = (frame[2] & TF_EF_FRESH) != 0U;
	e->cause = (uint8_t)cause;
	e->state = (uint8_t)state;
	e->fault = (uint8_t)fault;
	e->sample_ms = get_u32(&frame[4]);
	e->age_ms = (uint16_t)(((unsigned)frame[8] << 8) | frame[9]);
	e->count[0] = get_u32(&frame[10]);
	e->count[1] = get_u32(&frame[14]);
	e->invalid[0] = get_u32(&frame[18]);
	e->invalid[1] = get_u32(&frame[22]);
	return true;
}

bool tf_ack_decode(const uint8_t *data, size_t len, tf_ack_t *result, tf_command_t *cmd)
{
	uint32_t word;

	if ((data == NULL) || (len != (size_t)TF_ACK_BYTES)) {
		return false;
	}
	word = ((uint32_t)data[0] << 8) | data[1];
	if ((word & 0x7U) != 0U) {
		return false;
	}
	*result = (tf_ack_t)((word >> 14) & 0x3U);
	cmd->opcode = (uint8_t)((word >> 11) & 0x7U);
	cmd->session = 0U;
	cmd->seq = (uint8_t)((word >> 3) & 0xFFU);
	return true;
}
