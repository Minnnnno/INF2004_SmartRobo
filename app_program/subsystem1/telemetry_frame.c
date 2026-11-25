/*
 *  telemetry_frame.c
 *  Subsystem 1 -- bit-level codec for the formats in telemetry_frame.h.
 *  Plain C, no kernel calls: also built by the host test.
 */

#include "telemetry_frame.h"

/* Minimum-bit rule, checked for every field: the range fits in `bits`
   bits but would not fit in one bit fewer. */
#define TF_CHECK_(name, lo, hi, bits, owner, text)                         \
    _Static_assert((hi) > (lo), #name ": empty range");                    \
    _Static_assert(((int64_t)(hi) - (lo)) < ((int64_t)1 << (bits)),        \
                   #name ": too few bits for its range");                  \
    _Static_assert(((int64_t)(hi) - (lo)) >= ((int64_t)1 << ((bits) - 1)), \
                   #name ": more bits than its range needs");
TF_FIELDS(TF_CHECK_)
#undef TF_CHECK_

/* Agreed in the Tribunal: 21 bytes.  Layout v3 (Buddy 4 fields, Buddy 3
   barcode character): 167 bits, 1 spare bit. */
_Static_assert(TF_FRAME_BYTES == 21, "status frame must stay 21 bytes");
_Static_assert(TF_SPARE_BITS == 1, "status frame spare bits changed");
_Static_assert(44U == (sizeof(TF_BC_CHARS) - 1U), "BC_CHAR range is 0..44");
_Static_assert(TF_FIELD_COUNT <= 32, "clamp mask holds 32 fields");

#define TF_ROW_(name, lo, hi, bits, owner, text) {#name, (lo), (hi), (bits)},
const tf_field_t g_tf_fields[TF_FIELD_COUNT] = {TF_FIELDS(TF_ROW_)};
#undef TF_ROW_

/* Write the low `bits` bits of code, most significant first, at bit *pos. */
static void
put_bits (uint8_t * p_buf, unsigned * p_pos, uint32_t code, unsigned bits)
{
    while (bits > 0U)
    {
        bits--;

        if (0U != ((code >> bits) & 1U))
        {
            p_buf[*p_pos >> 3] |= (uint8_t)(0x80U >> (*p_pos & 7U));
        }

        (*p_pos)++;
    }
}

static uint32_t
get_bits (const uint8_t * p_buf, unsigned * p_pos, unsigned bits)
{
    uint32_t code = 0U;

    while (bits > 0U)
    {
        bits--;
        code = (code << 1)
               | (((uint32_t)p_buf[*p_pos >> 3] >> (7U - (*p_pos & 7U))) & 1U);
        (*p_pos)++;
    }

    return code;
}

uint32_t
tf_pack (const int32_t values[TF_FIELD_COUNT], uint8_t out[TF_FRAME_BYTES])
{
    uint32_t clamped = 0U;
    unsigned pos     = 0U;
    unsigned index;

    for (index = 0U; index < (unsigned)TF_FRAME_BYTES; index++)
    {
        out[index] = 0U;
    }

    for (index = 0U; index < (unsigned)TF_FIELD_COUNT; index++)
    {
        const tf_field_t * p_field = &g_tf_fields[index];
        int32_t            value   = values[index];

        if (value < p_field->min)
        {
            value = p_field->min;
            clamped |= 1UL << index;
        }
        else if (value > p_field->max)
        {
            value = p_field->max;
            clamped |= 1UL << index;
        }
        else
        {
            /* Nothing to do in any other case. */
        }

        put_bits(out, &pos, (uint32_t)(value - p_field->min), p_field->bits);
    }

    return clamped;
}

bool
tf_unpack (const uint8_t * p_frame, size_t len, int32_t values[TF_FIELD_COUNT])
{
    int32_t  tmp[TF_FIELD_COUNT];
    unsigned pos = 0U;
    unsigned index;

    if ((NULL == p_frame) || ((size_t)TF_FRAME_BYTES != len))
    {
        return false;
    }

    for (index = 0U; index < (unsigned)TF_FIELD_COUNT; index++)
    {
        const tf_field_t * p_field = &g_tf_fields[index];
        uint32_t           code    = get_bits(p_frame, &pos, p_field->bits);

        if (code > (uint32_t)(p_field->max - p_field->min))
        {
            return false; /* not a value the sender can produce */
        }

        tmp[index] = (int32_t)code + p_field->min;
    }

    if (0U != get_bits(p_frame, &pos, (unsigned)TF_SPARE_BITS))
    {
        return false;
    }

    if ((int32_t)TF_LAYOUT_VERSION != tmp[TF_F_VERSION])
    {
        return false;
    }

    for (index = 0U; index < (unsigned)TF_FIELD_COUNT; index++)
    {
        values[index] = tmp[index];
    }

    return true;
}

bool
tf_cmd_decode (const uint8_t * p_data, size_t len, tf_command_t * p_cmd)
{
    uint32_t word;

    p_cmd->opcode  = 0U;
    p_cmd->session = 0U;
    p_cmd->seq     = 0U;

    if ((NULL == p_data) || ((size_t)TF_CMD_BYTES != len))
    {
        return false;
    }

    word           = ((uint32_t)p_data[0] << 8) | p_data[1];
    p_cmd->opcode  = (uint8_t)((word >> 13) & 0x7U);
    p_cmd->session = (uint8_t)((word >> 8) & 0x1FU);
    p_cmd->seq     = (uint8_t)(word & 0xFFU);
    return (p_cmd->opcode >= (uint8_t)TF_OP_START)
           && (p_cmd->opcode <= (uint8_t)TF_OP_ESTOP);
}

void
tf_cmd_encode (const tf_command_t * p_cmd, uint8_t out[TF_CMD_BYTES])
{
    uint32_t word = (((uint32_t)p_cmd->opcode & 0x7U) << 13)
                    | (((uint32_t)p_cmd->session & 0x1FU) << 8) | p_cmd->seq;

    out[0] = (uint8_t)(word >> 8);
    out[1] = (uint8_t)(word & 0xFFU);
}

void
tf_ack_encode (tf_ack_t result, const tf_command_t * p_cmd,
               uint8_t out[TF_ACK_BYTES])
{
    uint32_t word = (((uint32_t)result & 0x3U) << 14)
                    | (((uint32_t)p_cmd->opcode & 0x7U) << 11)
                    | ((uint32_t)p_cmd->seq << 3);

    out[0] = (uint8_t)(word >> 8);
    out[1] = (uint8_t)(word & 0xFFU);
}

static void
put_u32 (uint8_t * p_bytes, uint32_t value)
{
    p_bytes[0] = (uint8_t)(value >> 24);
    p_bytes[1] = (uint8_t)(value >> 16);
    p_bytes[2] = (uint8_t)(value >> 8);
    p_bytes[3] = (uint8_t)value;
}

static uint32_t
get_u32 (const uint8_t * p_bytes)
{
    return ((uint32_t)p_bytes[0] << 24) | ((uint32_t)p_bytes[1] << 16)
           | ((uint32_t)p_bytes[2] << 8) | p_bytes[3];
}

static uint8_t
clamp_u8 (uint8_t value, unsigned max, bool * p_clamped)
{
    if (value > max)
    {
        *p_clamped = true;
        return (uint8_t)max;
    }

    return value;
}

bool
tf_enc_pack (const tf_encoder_t * p_encoder, uint8_t out[TF_ENC_BYTES])
{
    bool    b_clamped = false;
    uint8_t cause     = clamp_u8(p_encoder->cause, TF_EF_CAUSE_MAX, &b_clamped);
    uint8_t state = clamp_u8(p_encoder->state, TF_ENC_STATE_MAX, &b_clamped);
    uint8_t fault = clamp_u8(p_encoder->fault, TF_ENC_FAULT_MAX, &b_clamped);

    out[0] = (uint8_t)TF_ENC_VERSION;
    out[1] = p_encoder->seq;
    out[2] = (uint8_t)((p_encoder->reliable ? TF_EF_RELIABLE : 0U)
                       | (p_encoder->trusted ? TF_EF_TRUSTED : 0U)
                       | (p_encoder->fresh ? TF_EF_FRESH : 0U)
                       | ((unsigned)cause << TF_EF_CAUSE_SHIFT));
    out[3] = (uint8_t)(((unsigned)state << 4) | fault);
    put_u32(&out[4], p_encoder->sample_ms);
    out[8] = (uint8_t)(p_encoder->age_ms >> 8);
    out[9] = (uint8_t)p_encoder->age_ms;
    put_u32(&out[10], p_encoder->count[0]);
    put_u32(&out[14], p_encoder->count[1]);
    put_u32(&out[18], p_encoder->invalid[0]);
    put_u32(&out[22], p_encoder->invalid[1]);
    return b_clamped;
}

bool
tf_enc_unpack (const uint8_t * p_frame, size_t len, tf_encoder_t * p_encoder)
{
    unsigned cause;
    unsigned state;
    unsigned fault;

    if ((NULL == p_frame) || ((size_t)TF_ENC_BYTES != len)
        || ((uint8_t)TF_ENC_VERSION != p_frame[0])
        || (0U != (p_frame[2] & 0xC0U)))
    {
        return false;
    }

    cause = ((unsigned)p_frame[2] >> TF_EF_CAUSE_SHIFT) & 0x7U;
    state = (unsigned)p_frame[3] >> 4;
    fault = (unsigned)p_frame[3] & 0xFU;

    if ((cause > TF_EF_CAUSE_MAX) || (state > TF_ENC_STATE_MAX)
        || (fault > TF_ENC_FAULT_MAX))
    {
        return false;
    }

    p_encoder->seq       = p_frame[1];
    p_encoder->reliable  = 0U != (p_frame[2] & TF_EF_RELIABLE);
    p_encoder->trusted   = 0U != (p_frame[2] & TF_EF_TRUSTED);
    p_encoder->fresh     = 0U != (p_frame[2] & TF_EF_FRESH);
    p_encoder->cause     = (uint8_t)cause;
    p_encoder->state     = (uint8_t)state;
    p_encoder->fault     = (uint8_t)fault;
    p_encoder->sample_ms = get_u32(&p_frame[4]);
    p_encoder->age_ms    = (uint16_t)(((unsigned)p_frame[8] << 8) | p_frame[9]);
    p_encoder->count[0]  = get_u32(&p_frame[10]);
    p_encoder->count[1]  = get_u32(&p_frame[14]);
    p_encoder->invalid[0] = get_u32(&p_frame[18]);
    p_encoder->invalid[1] = get_u32(&p_frame[22]);
    return true;
}

bool
tf_ack_decode (const uint8_t * p_data, size_t len, tf_ack_t * p_result,
               tf_command_t * p_cmd)
{
    uint32_t word;

    if ((NULL == p_data) || ((size_t)TF_ACK_BYTES != len))
    {
        return false;
    }

    word = ((uint32_t)p_data[0] << 8) | p_data[1];

    if (0U != (word & 0x7U))
    {
        return false;
    }

    *p_result      = (tf_ack_t)((word >> 14) & 0x3U);
    p_cmd->opcode  = (uint8_t)((word >> 11) & 0x7U);
    p_cmd->session = 0U;
    p_cmd->seq     = (uint8_t)((word >> 3) & 0xFFU);
    return true;
}

/*** end of file ***/
