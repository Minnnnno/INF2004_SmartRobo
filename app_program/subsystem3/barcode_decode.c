/* Barcode decoding, see barcode_decode.h. */
#include "../../include/float_types.h"
#include "barcode_decode.h"

#ifndef BARCODE_SMOOTH
/* Average over 1 step either side (1.2 mm on the 0.4 mm grid). */
#define BARCODE_SMOOTH 1U
#endif
#ifndef BARCODE_NOISE_GAIN
/* A bar must rise 3x the average white noise ... */
#define BARCODE_NOISE_GAIN 3U
#endif
#ifndef BARCODE_MIN_RISE
#define BARCODE_MIN_RISE 9U /* ... plus 9 counts. */
#endif
#ifndef BARCODE_BAR_GAIN
/* Bar detection inside a stretch: noise multiple ... */
#define BARCODE_BAR_GAIN 2U
#endif
#ifndef BARCODE_BAR_RISE
/* ... plus counts (the stretch start/end keep NOISE_GAIN/MIN_RISE). */
#define BARCODE_BAR_RISE 6U
#endif
#ifndef BARCODE_FALL_NUM
/* A bar ends once it has fallen NUM/DEN of the way back to the low */
#define BARCODE_FALL_NUM 1U
#endif /* point before it: a third, as the small code's narrow spaces stay \
          well above white. */
#ifndef BARCODE_FALL_DEN
#define BARCODE_FALL_DEN 3U
#endif
#ifndef BARCODE_REF_SPAN
/* Width reference: tallest bar within this many either side (0 = whole pass).
 */
#define BARCODE_REF_SPAN 3U
#endif
#ifndef BARCODE_EDGE_REPAIR
/* Rebuild a missing outermost bar from the other '*' (0 = off). */
#define BARCODE_EDGE_REPAIR 1
#endif
/* A driven stretch ends only after 25 mm of flat floor ... */
#define BARCODE_FLAT_PATH 250U
/* ... unless it moved under 2 mm in the last 0.5 s (standing still). */
#define BARCODE_STILL_PATH 20U
#ifndef BARCODE_MAX_STRAY
/* A read leaves at most this many elements of the pass unused. */
#define BARCODE_MAX_STRAY 8U
#endif
#ifndef BARCODE_MIN_MOVE
/* A stretch that drove under 1 cm is decoded by time instead. */
#define BARCODE_MIN_MOVE 100U
#endif

/* Code 39: 9 elements (bar, space, ... bar), bit 8 = first element, 1 = wide.
 * code39_pattern[i] encodes code39_symbol[i]. */
static const char g_code39_symbol[]
    = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%";
static const uint16_t g_code39_pattern[] = {
    /* 0-9 */
    0x034,
    0x121,
    0x061,
    0x160,
    0x031,
    0x130,
    0x070,
    0x025,
    0x124,
    0x064,
    /* A-J */
    0x109,
    0x049,
    0x148,
    0x019,
    0x118,
    0x058,
    0x00D,
    0x10C,
    0x04C,
    0x01C,
    /* K-T */
    0x103,
    0x043,
    0x142,
    0x013,
    0x112,
    0x052,
    0x007,
    0x106,
    0x046,
    0x016,
    /* U-Z */
    0x181,
    0x0C1,
    0x1C0,
    0x091,
    0x190,
    0x0D0,
    /* - . space * $ / + % */
    0x085,
    0x184,
    0x0C4,
    0x094,
    0x0A8,
    0x0A2,
    0x08A,
    0x02A,
};
_Static_assert((sizeof g_code39_symbol - 1U)
                   == (sizeof g_code39_pattern / sizeof g_code39_pattern[0]),
               "one Code 39 pattern per symbol");

/* Marks the count longest elements among bars (first = 0) or spaces (first =
 * 1). */
static uint16_t
barcode_widest (const uint32_t * p_widths, unsigned first, unsigned count)
{
    uint16_t pattern = 0U;

    for (unsigned wheel = 0U; wheel < count; ++wheel)
    {
        unsigned widest = 9U;

        for (unsigned index = first; index < 9U; index += 2U)
        {
            bool b_taken = 0U != (pattern & (1U << (8U - index)));

            if ((!b_taken)
                && ((9U == widest) || (p_widths[index] > p_widths[widest])))
            {
                widest = index;
            }
        }

        pattern = (uint16_t)(pattern | (1U << (8U - widest)));
    }

    return pattern;
}

static char
barcode_lookup (uint16_t pattern)
{
    for (unsigned index = 0U;
         index < (sizeof g_code39_pattern / sizeof g_code39_pattern[0]);
         ++index)
    {
        if (g_code39_pattern[index] == pattern)
        {
            return g_code39_symbol[index];
        }
    }

    return '\0';
}

/* The nth longest (1 = longest) among bars (first = 0) or spaces (first = 1).
 */
static uint32_t
barcode_nth (const uint32_t * p_widths, unsigned first, unsigned nth)
{
    uint16_t taken   = barcode_widest(p_widths, first, nth - 1U);
    uint32_t longest = 0U;

    for (unsigned index = first; index < 9U; index += 2U)
    {
        if ((0U == (taken & (1U << (8U - index))))
            && (p_widths[index] > longest))
        {
            longest = p_widths[index];
        }
    }

    return longest;
}

/* Bars and spaces are compared only among themselves, because the sensor reads
 * black wider than white. Most characters have 2 wide bars and 1 wide space; $
 * / + % have 3 wide spaces and no wide bar. Each layout is scored by how well
 * it fits: 2 wide bars + 1 wide space: (bar 2nd / bar 3rd) * (space 1st / space
 * 2nd) 3 wide spaces:              (space 3rd / space 4th) * (bar 5th / bar
 * 1st, all bars even) No speed reference is needed. */
static char
barcode_char (const uint32_t * p_widths)
{
    float64_t normal        = ((float64_t)barcode_nth(p_widths, 0U, 2U)
                               / (float64_t)barcode_nth(p_widths, 0U, 3U))
                              * ((float64_t)barcode_nth(p_widths, 1U, 1U)
                                 / (float64_t)barcode_nth(p_widths, 1U, 2U));
    float64_t spaces        = ((float64_t)barcode_nth(p_widths, 1U, 3U)
                               / (float64_t)barcode_nth(p_widths, 1U, 4U))
                              * ((float64_t)barcode_nth(p_widths, 0U, 5U)
                                 / (float64_t)barcode_nth(p_widths, 0U, 1U));
    bool      b_spaces_only = spaces > normal;

    return barcode_lookup(b_spaces_only
                              ? barcode_widest(p_widths, 1U, 3U)
                              : (uint16_t)(barcode_widest(p_widths, 0U, 2U)
                                           | barcode_widest(p_widths, 1U, 1U)));
}

/* Elements alternate bar/space starting with a bar; characters are separated by
 * a one-element gap. Returns true only for a complete *...* read. */
static bool
barcode_decode (const uint32_t * p_widths, unsigned count, char * p_out)
{
    unsigned chars = (count + 1U) / 10U;

    if ((chars < 3U) || (chars > BARCODE_MAX_CHARS)
        || (count != ((chars * 10U) - 1U)))
    {
        return false;
    }

    for (unsigned character = 0U; character < chars; ++character)
    {
        p_out[character] = barcode_char(&p_widths[character * 10U]);

        if ('\0' == p_out[character])
        {
            return false;
        }
    }

    p_out[chars] = '\0';
    return ('*' == p_out[0]) && ('*' == p_out[chars - 1U]);
}

/* Finds a complete *...* anywhere in the elements, so stray marks before or
 * after the barcode are skipped. first is the index of the first bar. Returns
 * the character count. */
static unsigned
barcode_search (const uint32_t * p_widths, unsigned count, unsigned first,
                char * p_out)
{
    for (unsigned start = first; (start + 29U) <= count; start += 2U)
    {
        for (unsigned chars = BARCODE_MAX_CHARS; chars >= 3U; --chars)
        {
            unsigned length = (chars * 10U) - 1U;

            if (((start + length) <= count)
                && (barcode_decode(&p_widths[start], length, p_out)))
            {
                return chars;
            }
        }
    }

    return 0U;
}

static void
barcode_reverse (uint32_t * p_widths, unsigned count)
{
    for (unsigned index = 0U; index < (count / 2U); ++index)
    {
        uint32_t swap = p_widths[index];

        p_widths[index]              = p_widths[count - 1U - index];
        p_widths[count - 1U - index] = swap;
    }
}

/* Forward first, then reversed (in place, restored before returning).
 * Elements alternate bar/space from a bar at index 0. */
static unsigned
barcode_read_strict (uint32_t * p_widths, unsigned count, char * p_out)
{
    unsigned chars = barcode_search(p_widths, count, 0U, p_out);

    if ((0U == chars) && (0U != count))
    {
        barcode_reverse(p_widths, count);
        chars = barcode_search(p_widths, count, (count - 1U) % 2U, p_out);
        barcode_reverse(p_widths, count);
    }

    /* One pass crosses one code: a *...* found among many more elements is a
     * chance match in noise. */
    if ((0U != chars) && (count > ((chars * 10U) - 1U + BARCODE_MAX_STRAY)))
    {
        chars = 0U;
    }

    return chars;
}

#if BARCODE_EDGE_REPAIR
/* The outermost bar of the code, and the wide space inside it, can go missing
 * (that bar sits nearest the sheet's edge): the elements then start (or end)
 * two short. Start and stop are both
 * '*', read the same way round, so the missing pair is copied from the '*' at
 * the other end and the whole code must still decode, both '*' included. Tried
 * only when the strict read fails. */
static unsigned
barcode_read (uint32_t * p_widths, unsigned count, char * p_out)
{
    static uint32_t elements[BARCODE_MAX_ELEMENTS];
    unsigned        chars  = barcode_read_strict(p_widths, count, p_out);
    unsigned int    result = 0U;

    if ((0U != chars) || (count < 27U) || ((count + 2U) > BARCODE_MAX_ELEMENTS))
    {
        result = chars;
    }
    else
    {
        for (unsigned side = 0U; (side < 4U) && (0U == chars); ++side)
        {
            bool b_reversed = side >= 2U;
            bool b_at_start = 0U == (side % 2U);

            for (unsigned index = 0U; index < count; ++index)
            {
                elements[index] = b_reversed ? p_widths[count - 1U - index]
                                             : p_widths[index];
            }

            if (b_at_start)
            /* e = [e[n-9], e[n-8]] + e: the stop's first bar and wide space. */
            {
                for (unsigned index = count; index > 0U; --index)
                {
                    elements[index + 1U] = elements[index - 1U];
                }

                elements[0] = elements[count - 7U];
                elements[1] = elements[count - 6U];
            }
            else
            { /* e = e + [e[7], e[8]]: the start's last space and bar. */
                elements[count]      = elements[7];
                elements[count + 1U] = elements[8];
            }

            chars = barcode_read_strict(elements, count + 2U, p_out);
        }

        result = chars;
    }

    return result;
}
#else
#define barcode_read barcode_read_strict
#endif

/* Black reads higher than white. At the robot's ride height the sensor blurs
 * the bars: narrow ones are faint bumps and wide ones tall, so one threshold
 * misses bars.
 *   1. smooth over 5 samples, in place;
 *   2. the first 0.5 s is white: its level and average spread (noise);
 *   3. a bar rises a prominence above the lowest point since the last bar and
 * ends once it falls a third of the way back to that low point (and half a
 * prominence below its peak), so a faint bar ends before the next one and a dip
 * inside a solid bar does not split it;
 *   4. blur spreads a bar but keeps the area under it, so a bar's width is its
 * area above the white either side, over the bump only (where it stays half a
 *      prominence above that white), divided by the scan's black-white range,
 *      centred on the area's centre; spaces are the gaps between bars.
 * White before the first bar and after the last is not stored. */
/* Step 3: finds the bars that rise at least prominence. Returns their count. */
static unsigned
barcode_bars (const uint16_t * p_samples, unsigned count, uint32_t prominence,
              unsigned * p_peak, unsigned * p_trough)
{
    unsigned bars     = 0U;
    unsigned low_at   = 0U;
    unsigned high_at  = 0U;
    bool     b_in_bar = false;

    for (unsigned index = 0U; index < count; ++index)
    {
        if (!b_in_bar)
        {
            if (p_samples[index] < p_samples[low_at])
            {
                low_at = index;
            }

            if (p_samples[index] >= (p_samples[low_at] + prominence))
            {
                b_in_bar       = true;
                high_at        = index;
                p_trough[bars] = low_at;
            }
        }
        else
        {
            if (p_samples[index] > p_samples[high_at])
            {
                high_at = index;
            }

            if (((p_samples[index] + (prominence / 2U)) <= p_samples[high_at])
                && ((BARCODE_FALL_DEN
                     * ((uint32_t)p_samples[high_at] - p_samples[index]))
                    >= (BARCODE_FALL_NUM
                        * ((uint32_t)p_samples[high_at]
                           - p_samples[p_trough[bars]]))))
            {
                p_peak[bars++] = high_at;
                b_in_bar       = false;
                low_at         = index;

                if (BARCODE_MAX_BARS == bars)
                {
                    break;
                }
            }
        }
    }

    if (b_in_bar && (bars < BARCODE_MAX_BARS))
    {
        p_peak[bars++] = high_at;
        low_at         = count - 1U;
    }

    p_trough[bars] = low_at;
    return bars;
}

/* Median of the bars' peak heights above white (sorts a copy). */
static uint32_t
barcode_median_height (const uint16_t * p_samples, const unsigned * p_peak,
                       unsigned bars, uint32_t white)
{
    uint32_t heights[BARCODE_MAX_BARS];

    for (unsigned position = 0U; position < bars; ++position)
    {
        heights[position] = (p_samples[p_peak[position]] > white)
                                ? p_samples[p_peak[position]] - white
                                : 0U;
    }

    for (unsigned position = 1U; position < bars; ++position)
    { /* Insertion sort: at most 50 values. */
        uint32_t height = heights[position];
        unsigned other  = position;

        while ((other > 0U) && (heights[other - 1U] > height))
        {
            heights[other] = heights[other - 1U];
            --other;
        }

        heights[other] = height;
    }

    return (0U == bars) ? 0U : heights[bars / 2U];
}

static unsigned
barcode_elements (uint16_t * p_samples, unsigned count, uint32_t * p_widths,
                  unsigned contrast_div)
{
    /* Unsmoothed values of the last BARCODE_SMOOTH samples. */
    uint16_t raw[BARCODE_SMOOTH] = {0};

    /* Trough: lowest point before each bar, and after the last. */
    unsigned  peak[BARCODE_MAX_BARS];
    unsigned  trough[BARCODE_MAX_BARS + 1U];
    unsigned  bars;
    unsigned  sample_count = 0U;
    uint32_t  white        = 0U;
    uint32_t  noise        = 0U;
    uint32_t  prominence;
    uint32_t  highest = 0U;
    uint32_t  contrast;
    float64_t previous_end = 0.0;

    if ((count < BARCODE_QUIET) || (count > BARCODE_SAMPLES))
    {
        return 0U;
    }

    for (unsigned index = 0U; index < count; ++index)
    {
        unsigned from = (index < BARCODE_SMOOTH) ? 0U : index - BARCODE_SMOOTH;
        unsigned last_sample = ((index + BARCODE_SMOOTH) >= count)
                                   ? count - 1U
                                   : index + BARCODE_SMOOTH;
        uint32_t sum         = 0U;

        for (unsigned other = from; other <= last_sample; ++other)
        {
            sum += (other < index) ? raw[other % BARCODE_SMOOTH]
                                   : p_samples[other];
        }

        raw[index % BARCODE_SMOOTH] = p_samples[index];
        p_samples[index] = (uint16_t)(sum / (last_sample - from + 1U));
    }

    for (unsigned index = 0U; index < BARCODE_QUIET; ++index)
    {
        white += p_samples[index];
    }

    white /= BARCODE_QUIET;

    for (unsigned index = 0U; index < BARCODE_QUIET; ++index)
    {
        noise += (p_samples[index] > white) ? p_samples[index] - white
                                            : white - p_samples[index];
    }

    noise /= BARCODE_QUIET;
    prominence = (BARCODE_BAR_GAIN * noise) + BARCODE_BAR_RISE;

    for (unsigned index = 0U; index < count; ++index)
    {
        if (p_samples[index] > highest)
        {
            highest = p_samples[index];
        }
    }

    if (highest <= white)
    {
        return 0U;
    }

    if (contrast_div > 0U)
    {
        /* The noise alone sets the first pass; the bars' median height then
         * scales the prominence, so a faint code keeps its faint bars and a dip
         * inside a solid bar of a strong code does not split it. A bigger
         * divisor is more sensitive (calibrated on a known 'A', see barcode.c).
         */
        contrast = barcode_median_height(
            p_samples, peak,
            barcode_bars(p_samples, count, prominence, peak, trough), white);

        if ((contrast / contrast_div) > prominence)
        {
            prominence = contrast / contrast_div;
        }
    }

    bars = barcode_bars(p_samples, count, prominence, peak, trough);
#ifdef BARCODE_DEBUG
    BARCODE_DEBUG(m, count, peak, trough, bars, white, prominence);
#endif
    contrast = highest - white;

    for (unsigned position = 0U; position < bars; ++position)
    {
#if BARCODE_REF_SPAN > 0
        /* Black level near this bar: the tallest bar within BARCODE_REF_SPAN
         * either side (a character spans 5 bars and nearly always has a wide
         * one), so a bar the sensor met off-centre does not shrink the widths
         * of the whole pass. */
        unsigned low
            = (position < BARCODE_REF_SPAN) ? 0U : position - BARCODE_REF_SPAN;
        unsigned high = ((position + BARCODE_REF_SPAN) >= bars)
                            ? bars - 1U
                            : position + BARCODE_REF_SPAN;

        contrast = 1U;

        for (unsigned other = low; other <= high; ++other)
        {
            if ((p_samples[peak[other]] > white)
                && ((p_samples[peak[other]] - white) > contrast))
            {
                contrast = p_samples[peak[other]] - white;
            }
        }
#endif
        float64_t base   = ((float64_t)p_samples[trough[position]]
                            + (float64_t)p_samples[trough[position + 1U]])
                           / 2.0;
        float64_t edge   = base + ((float64_t)prominence / 2.0);
        float64_t area   = 0.0;
        float64_t moment = 0.0;
        float64_t width;
        float64_t centre;
        unsigned  start = peak[position];
        unsigned  end   = peak[position];

        while ((start > trough[position])
               && ((float64_t)p_samples[start - 1U] > edge))
        {
            --start;
        }

        while ((end < trough[position + 1U])
               && ((float64_t)p_samples[end + 1U] > edge))
        {
            ++end;
        }

        for (unsigned index = start; index <= end; ++index)
        {
            float64_t above = (float64_t)p_samples[index] - base;

            if (above > 0.0)
            {
                area += above;
                moment += above * (float64_t)index;
            }
        }

        if (area <= 0.0)
        {
            continue;
        }

        width  = area / (float64_t)contrast; /* In samples. */
        centre = moment / area;

        if (sample_count > 0U)
        {
            float64_t gap = (centre - (width / 2.0)) - previous_end;

            p_widths[sample_count++]
                = (uint32_t)(((gap > 1.0) ? gap : 1.0) * BARCODE_SAMPLE_US);
        }

        p_widths[sample_count++] = (uint32_t)(width * BARCODE_SAMPLE_US);
        previous_end             = centre + (width / 2.0);
    }

    return sample_count;
}

uint32_t
barcode_level (const uint16_t * p_samples, unsigned count, uint32_t * p_noise)
{
    uint32_t mean   = 0U;
    uint32_t spread = 0U;

    for (unsigned index = 0U; index < count; ++index)
    {
        mean += p_samples[index];
    }

    mean /= count;

    for (unsigned index = 0U; index < count; ++index)
    {
        spread += (p_samples[index] > mean) ? p_samples[index] - mean
                                            : mean - p_samples[index];
    }

    *p_noise = spread / count;
    return mean;
}

/* How far a reading must leave the quiet level to count as a bar. */
static uint32_t
barcode_prominence (uint32_t noise)
{
    return (BARCODE_NOISE_GAIN * noise) + BARCODE_MIN_RISE;
}

void
barcode_range (const uint16_t * p_samples, unsigned count, uint16_t * p_low,
               uint16_t * p_high)
{
    *p_low  = 0xFFFFU;
    *p_high = 0U;

    for (unsigned index = 0U; index < count; ++index)
    {
        if (p_samples[index] < *p_low)
        {
            *p_low = p_samples[index];
        }

        if (p_samples[index] > *p_high)
        {
            *p_high = p_samples[index];
        }
    }
}

/* True when the end of the stretch lies within two prominences (the range of 50
 * noisy samples is wider than the noise-based prominence; a bar rise is far
 * larger). The flat end is the last BARCODE_FLAT samples, and while the car
 * drives it must also span BARCODE_FLAT_PATH of floor, so a wide space crossed
 * slowly does not end the pass. Standing still (or a hand scan): time only. */
static bool
barcode_flat (const barcode_seg_t * p_segment)
{
    unsigned from = p_segment->count - BARCODE_FLAT;
    uint32_t end  = p_segment->path[p_segment->count - 1U];

    uint16_t low;
    uint16_t high;

    if ((end - p_segment->path[from]) >= BARCODE_STILL_PATH)
    {
        while ((from > BARCODE_QUIET)
               && ((end - p_segment->path[from]) < BARCODE_FLAT_PATH))
        {
            --from;
        }

        if ((end - p_segment->path[from]) < BARCODE_FLAT_PATH)
        {
            return false;
        } /* Not yet 25 mm since the start. */
    }

    barcode_range(&p_segment->samples[from], p_segment->count - from, &low,
                  &high);
    return (uint32_t)(high - low) < (2U * p_segment->prominence);
}

/* Copies a stretch into out, resampled to equal steps of driven distance so
 * that a bar's width does not depend on the car's speed (it speeds up and slows
 * down while wiggling). The quiet lead-in is kept as it is: it is only the
 * noise reference. A stretch the car hardly moved through (hand scan, no path)
 * is copied unchanged. Returns the new length. */
static unsigned
barcode_resample (const uint16_t * p_samples, const uint32_t * p_path,
                  unsigned count, uint16_t * p_out)
{
    unsigned     sample_count = BARCODE_QUIET;
    unsigned     other        = BARCODE_QUIET;
    uint32_t     sample_time  = p_path[BARCODE_QUIET - 1U] + BARCODE_PATH_STEP;
    unsigned int result       = 0U;

    for (unsigned index = 0U; index < BARCODE_QUIET; ++index)
    {
        p_out[index] = p_samples[index];
    }

    if (p_path[count - 1U] < (p_path[BARCODE_QUIET] + BARCODE_MIN_MOVE))
    {
        for (unsigned index = BARCODE_QUIET; index < count; ++index)
        {
            p_out[index] = p_samples[index];
        }

        result = count;
    }
    else
    {
        while (sample_count < BARCODE_SAMPLES)
        {
            while ((other < count) && (p_path[other] < sample_time))
            {
                ++other;
            }

            if (other >= count)
            {
                break;
            }

            /* path[j - 1] < at <= path[j]: interpolate the reading at the
             * distance 'at'. */
            p_out[sample_count++]
                = (uint16_t)((int32_t)p_samples[other - 1U]
                             + ((((int32_t)p_samples[other]
                                  - (int32_t)p_samples[other - 1U])
                                 * (int32_t)(sample_time - p_path[other - 1U]))
                                / (int32_t)(p_path[other]
                                            - p_path[other - 1U])));
            sample_time += BARCODE_PATH_STEP;
        }

        result = sample_count;
    }

    return result;
}

/* ---- Best fit: the course only uses a few letters, so a pass with one
 * wide/narrow element wrong can still be told apart by which allowed letter's
 * *X* pattern it is closest to. ---- */
static uint16_t
barcode_pattern_of (char letter)
{
    for (unsigned index = 0U;
         index < (sizeof g_code39_pattern / sizeof g_code39_pattern[0]);
         ++index)
    {
        if (g_code39_symbol[index] == letter)
        {
            return g_code39_pattern[index];
        }
    }

    return 0U;
}

static unsigned
barcode_bit_count (uint16_t bits)
{
    unsigned count = 0U;

    for (; 0U != bits; bits = (uint16_t)(bits >> 1U))
    {
        count += bits & 1U;
    }

    return count;
}

/* Elements of the three characters at d that disagree with *letter* (letters
 * and '*' all have 2 wide bars and 1 wide space, so each character is marked
 * that way; a swapped pair costs 2). */
static unsigned
barcode_mismatch (const uint32_t * p_widths, uint16_t letter)
{
    const uint16_t star    = barcode_pattern_of('*');
    const uint16_t want[3] = {star, letter, star};
    unsigned       total   = 0U;

    for (unsigned character = 0U; character < 3U; ++character)
    {
        uint16_t seen
            = (uint16_t)(barcode_widest(&p_widths[character * 10U], 0U, 2U)
                         | barcode_widest(&p_widths[character * 10U], 1U, 1U));

        total += barcode_bit_count((uint16_t)(seen ^ want[character]));
    }

    return total;
}

char
barcode_best_fit (uint32_t * p_widths, unsigned count, const char * p_letters)
{
    unsigned best   = 99U;
    unsigned second = 99U;
    char     found  = '\0';
    char     result = 0;

    if ((NULL == p_letters) || (count < 29U)
        || (count > (29U + BARCODE_MAX_STRAY)))
    {
        result = '\0';
    }
    else
    {
        for (const char * p_letter = p_letters; '\0' != *p_letter; ++p_letter)
        {
            uint16_t pattern = barcode_pattern_of(*p_letter);
            unsigned fit     = 99U;

            if (0U == pattern)
            {
                continue;
            }

            for (unsigned dir = 0U; dir < 2U; ++dir)
            { /* Forward, then reversed (restored below). */

                if (1U == dir)
                {
                    barcode_reverse(p_widths, count);
                }

                for (unsigned start = 0U; (start + 29U) <= count; start += 2U)
                { /* Every window starts on a bar. */
                    unsigned misses
                        = barcode_mismatch(&p_widths[start], pattern);

                    if (misses < fit)
                    {
                        fit = misses;
                    }
                }

                if (1U == dir)
                {
                    barcode_reverse(p_widths, count);
                }
            }

            if (fit < best)
            {
                second = best;
                best   = fit;
                found  = *p_letter;
            }
            else if (fit < second)
            {
                second = fit;
            }
            else
            {
                /* Nothing to do in any other case. */
            }
        }

        result = ((best <= BARCODE_FIT_MAX_MISMATCH) && (best < second)) ? found
                                                                         : '\0';
    }

    return result;
}

static bool
barcode_allowed (const char * p_letters, char letter)
{
    for (; '\0' != *p_letters; ++p_letters)
    {
        if (*p_letters == letter)
        {
            return true;
        }
    }

    return false;
}

unsigned
barcode_decode_stretch (const uint16_t * p_samples, const uint32_t * p_path,
                        unsigned count, uint16_t * p_work, uint32_t * p_widths,
                        unsigned * p_elements, char * p_text,
                        unsigned contrast_div, const char * p_letters)
{
    unsigned chars;

    *p_elements = barcode_elements(
        p_work, barcode_resample(p_samples, p_path, count, p_work), p_widths,
        contrast_div);
    chars = barcode_read(p_widths, *p_elements, p_text);

    if ((NULL != p_letters) && ('\0' != p_letters[0])
        && ((3U != chars) || (!barcode_allowed(p_letters, p_text[1]))))
    {
        char letter = barcode_best_fit(p_widths, *p_elements, p_letters);

        chars = 0U;

        if ('\0' != letter)
        {
            p_text[0] = '*';
            p_text[1] = letter;
            p_text[2] = '*';
            p_text[3] = '\0';
            chars     = 3U;
        }
    }

    return chars;
}

void
barcode_seg_init (barcode_seg_t * p_segment)
{
    p_segment->count      = 0U;
    p_segment->head       = 0U;
    p_segment->filled     = 0U;
    p_segment->prominence = 0U;
    p_segment->level      = 0U;
    p_segment->level_ok   = false;
    p_segment->done       = false;
}

barcode_seg_event_t
barcode_seg_feed (barcode_seg_t * p_segment, uint16_t sample, uint32_t path,
                  uint16_t heading)
{
    barcode_seg_event_t event = BARCODE_SEG_RUNNING;

    if (p_segment->done)
    { /* The end of the last stretch is the new lead-in. */

        for (unsigned index = 0U; index < BARCODE_QUIET; ++index)
        {
            p_segment->quiet[index]
                = p_segment->samples[p_segment->count - BARCODE_QUIET + index];
            p_segment->quiet_path[index]
                = p_segment->path[p_segment->count - BARCODE_QUIET + index];
            p_segment->quiet_heading[index]
                = p_segment->heading[p_segment->count - BARCODE_QUIET + index];
        }

        p_segment->head  = 0U;
        p_segment->count = 0U;
        p_segment->done  = false;
    }

    if (0U == p_segment->count)
    {
        uint32_t noise;
        uint32_t mean = 0U;

        if (BARCODE_QUIET == p_segment->filled)
        {
            mean = barcode_level(p_segment->quiet, BARCODE_QUIET, &noise);
            p_segment->prominence = barcode_prominence(noise);
            p_segment->level      = mean;
            p_segment->level_ok   = true;
        }

        if ((p_segment->filled < BARCODE_QUIET)
            || (((sample > mean) ? sample - mean : mean - sample)
                < p_segment->prominence))
        {
            /* Still quiet: keep it as the lead-in. */
            p_segment->quiet[p_segment->head]         = sample;
            p_segment->quiet_path[p_segment->head]    = path;
            p_segment->quiet_heading[p_segment->head] = heading;
            p_segment->head = (p_segment->head + 1U) % BARCODE_QUIET;

            if (p_segment->filled < BARCODE_QUIET)
            {
                ++p_segment->filled;
            }

            return BARCODE_SEG_IDLE;
        }

        /* The reading left the quiet level: start a stretch with the lead-in.
         */
        for (unsigned index = 0U; index < BARCODE_QUIET; ++index)
        {
            p_segment->samples[index]
                = p_segment->quiet[(p_segment->head + index) % BARCODE_QUIET];
            p_segment->path[index]
                = p_segment
                      ->quiet_path[(p_segment->head + index) % BARCODE_QUIET];
            p_segment->heading[index]
                = p_segment->quiet_heading[(p_segment->head + index)
                                           % BARCODE_QUIET];
        }

        p_segment->count = BARCODE_QUIET;
        event            = BARCODE_SEG_STARTED;
    }

    p_segment->path[p_segment->count]      = path;
    p_segment->heading[p_segment->count]   = heading;
    p_segment->samples[p_segment->count++] = sample;

    if ((BARCODE_SAMPLES == p_segment->count)
        || ((p_segment->count >= (BARCODE_QUIET + BARCODE_FLAT))
            && (barcode_flat(p_segment))))
    {
        p_segment->done = true;
        return BARCODE_SEG_DONE;
    }

    return event;
}

unsigned
barcode_ladder_div (unsigned first, unsigned index)
{
    static const unsigned ladder[] = BARCODE_LADDER;

    if (0U == index)
    {
        return first;
    }

    for (size_t position = 0U; position < (sizeof ladder / sizeof ladder[0]);
         ++position)
    {
        if ((ladder[position] != first) && (0U == --index))
        {
            return ladder[position];
        }
    }

    return 0U;
}

/*** end of file ***/
