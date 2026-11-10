/* Barcode decoding, see barcode_decode.h. */
#include "barcode_decode.h"

#ifndef BARCODE_SMOOTH
#define BARCODE_SMOOTH       1U            /* Average over 1 step either side (1.2 mm on the 0.4 mm grid). */
#endif
#ifndef BARCODE_NOISE_GAIN
#define BARCODE_NOISE_GAIN   3U            /* A bar must rise 3x the average white noise ... */
#endif
#ifndef BARCODE_MIN_RISE
#define BARCODE_MIN_RISE     9U            /* ... plus 9 counts. */
#endif
#ifndef BARCODE_BAR_GAIN
#define BARCODE_BAR_GAIN     2U   /* Bar detection inside a stretch: noise multiple ... */
#endif
#ifndef BARCODE_BAR_RISE
#define BARCODE_BAR_RISE     6U   /* ... plus counts (the stretch start/end keep NOISE_GAIN/MIN_RISE). */
#endif
#ifndef BARCODE_FALL_NUM
#define BARCODE_FALL_NUM     1U   /* A bar ends once it has fallen NUM/DEN of the way back to the low */
#endif                            /* point before it: a third, as the small code's narrow spaces stay well above white. */
#ifndef BARCODE_FALL_DEN
#define BARCODE_FALL_DEN     3U
#endif
#ifndef BARCODE_CONTRAST_DIV
#define BARCODE_CONTRAST_DIV 32            /* Prominence at least the bars' median height / this (0 = off). */
#endif
#ifndef BARCODE_REF_SPAN
#define BARCODE_REF_SPAN     3U            /* Width reference: tallest bar within this many either side (0 = whole pass). */
#endif
#ifndef BARCODE_EDGE_REPAIR
#define BARCODE_EDGE_REPAIR  1             /* Rebuild a missing outermost bar from the other '*' (0 = off). */
#endif
#define BARCODE_FLAT_PATH    250U          /* A driven stretch ends only after 25 mm of flat floor ... */
#define BARCODE_STILL_PATH   20U           /* ... unless it moved under 2 mm in the last 0.5 s (standing still). */
#ifndef BARCODE_MAX_STRAY
#define BARCODE_MAX_STRAY    8U            /* A read leaves at most this many elements of the pass unused. */
#endif
#ifndef BARCODE_MIN_MOVE
#define BARCODE_MIN_MOVE     100U          /* A stretch that drove under 1 cm is decoded by time instead. */
#endif

/* Code 39: 9 elements (bar, space, ... bar), bit 8 = first element, 1 = wide.
 * code39_pattern[i] encodes code39_symbol[i]. */
static const char code39_symbol[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%";
static const uint16_t code39_pattern[] = {
    0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025, 0x124, 0x064,  /* 0-9 */
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C, 0x01C,  /* A-J */
    0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,  /* K-T */
    0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0,                              /* U-Z */
    0x085, 0x184, 0x0C4, 0x094, 0x0A8, 0x0A2, 0x08A, 0x02A,                /* - . space * $ / + % */
};
_Static_assert(sizeof code39_symbol - 1U == sizeof code39_pattern / sizeof code39_pattern[0],
               "one Code 39 pattern per symbol");

/* Marks the count longest elements among bars (first = 0) or spaces (first = 1). */
static uint16_t barcode_widest(const uint32_t *d, unsigned first, unsigned count)
{
    uint16_t pattern = 0U;
    for (unsigned w = 0U; w < count; ++w) {
        unsigned widest = 9U;
        for (unsigned i = first; i < 9U; i += 2U) {
            bool taken = (pattern & (1U << (8U - i))) != 0U;
            if (!taken && (widest == 9U || d[i] > d[widest])) { widest = i; }
        }
        pattern = (uint16_t)(pattern | (1U << (8U - widest)));
    }
    return pattern;
}

static char barcode_lookup(uint16_t pattern)
{
    for (unsigned i = 0U; i < sizeof code39_pattern / sizeof code39_pattern[0]; ++i) {
        if (code39_pattern[i] == pattern) { return code39_symbol[i]; }
    }
    return '\0';
}

/* The nth longest (1 = longest) among bars (first = 0) or spaces (first = 1). */
static uint32_t barcode_nth(const uint32_t *d, unsigned first, unsigned nth)
{
    uint16_t taken = barcode_widest(d, first, nth - 1U);
    uint32_t longest = 0U;
    for (unsigned i = first; i < 9U; i += 2U) {
        if ((taken & (1U << (8U - i))) == 0U && d[i] > longest) { longest = d[i]; }
    }
    return longest;
}

/* Bars and spaces are compared only among themselves, because the sensor reads black
 * wider than white. Most characters have 2 wide bars and 1 wide space; $ / + % have
 * 3 wide spaces and no wide bar. Each layout is scored by how well it fits:
 *   2 wide bars + 1 wide space: (bar 2nd / bar 3rd) * (space 1st / space 2nd)
 *   3 wide spaces:              (space 3rd / space 4th) * (bar 5th / bar 1st, all bars even)
 * No speed reference is needed. */
static char barcode_char(const uint32_t *d)
{
    double normal = ((double)barcode_nth(d, 0U, 2U) / (double)barcode_nth(d, 0U, 3U)) *
                    ((double)barcode_nth(d, 1U, 1U) / (double)barcode_nth(d, 1U, 2U));
    double spaces = ((double)barcode_nth(d, 1U, 3U) / (double)barcode_nth(d, 1U, 4U)) *
                    ((double)barcode_nth(d, 0U, 5U) / (double)barcode_nth(d, 0U, 1U));
    bool spaces_only = spaces > normal;
    return barcode_lookup(spaces_only ? barcode_widest(d, 1U, 3U)
                                      : (uint16_t)(barcode_widest(d, 0U, 2U) | barcode_widest(d, 1U, 1U)));
}

/* Elements alternate bar/space starting with a bar; characters are separated by a
 * one-element gap. Returns true only for a complete *...* read. */
static bool barcode_decode(const uint32_t *d, unsigned n, char *out)
{
    unsigned chars = (n + 1U) / 10U;
    if (chars < 3U || chars > BARCODE_MAX_CHARS || n != chars * 10U - 1U) { return false; }
    for (unsigned c = 0U; c < chars; ++c) {
        out[c] = barcode_char(&d[c * 10U]);
        if (out[c] == '\0') { return false; }
    }
    out[chars] = '\0';
    return out[0] == '*' && out[chars - 1U] == '*';
}

/* Finds a complete *...* anywhere in the elements, so stray marks before or after the
 * barcode are skipped. first is the index of the first bar. Returns the character count. */
static unsigned barcode_search(const uint32_t *d, unsigned n, unsigned first, char *out)
{
    for (unsigned start = first; start + 29U <= n; start += 2U) {
        for (unsigned chars = BARCODE_MAX_CHARS; chars >= 3U; --chars) {
            unsigned length = chars * 10U - 1U;
            if (start + length <= n && barcode_decode(&d[start], length, out)) { return chars; }
        }
    }
    return 0U;
}

static void barcode_reverse(uint32_t *d, unsigned n)
{
    for (unsigned i = 0U; i < n / 2U; ++i) {
        uint32_t t = d[i];
        d[i] = d[n - 1U - i];
        d[n - 1U - i] = t;
    }
}

/* Forward first, then reversed (in place, restored before returning).
 * Elements alternate bar/space from a bar at index 0. */
static unsigned barcode_read_strict(uint32_t *d, unsigned n, char *out)
{
    unsigned chars = barcode_search(d, n, 0U, out);
    if (chars == 0U && n != 0U) {
        barcode_reverse(d, n);
        chars = barcode_search(d, n, (n - 1U) % 2U, out);
        barcode_reverse(d, n);
    }
    /* One pass crosses one code: a *...* found among many more elements is a chance match in noise. */
    if (chars != 0U && n > chars * 10U - 1U + BARCODE_MAX_STRAY) { chars = 0U; }
    return chars;
}

#if BARCODE_EDGE_REPAIR
/* The outermost bar of the code, and the wide space inside it, can go missing (that bar sits
 * nearest the sheet's edge): the elements then start (or end) two short. Start and stop are both
 * '*', read the same way round, so the missing pair is copied from the '*' at the other end and
 * the whole code must still decode, both '*' included. Tried only when the strict read fails. */
static unsigned barcode_read(uint32_t *d, unsigned n, char *out)
{
    static uint32_t e[BARCODE_MAX_ELEMENTS];
    unsigned chars = barcode_read_strict(d, n, out);
    if (chars != 0U || n < 27U || n + 2U > BARCODE_MAX_ELEMENTS) { return chars; }
    for (unsigned side = 0U; side < 4U && chars == 0U; ++side) {
        bool reversed = side >= 2U, at_start = (side % 2U) == 0U;
        for (unsigned i = 0U; i < n; ++i) { e[i] = reversed ? d[n - 1U - i] : d[i]; }
        if (at_start) { /* e = [e[n-9], e[n-8]] + e: the stop's first bar and wide space. */
            for (unsigned i = n; i > 0U; --i) { e[i + 1U] = e[i - 1U]; }
            e[0] = e[n - 7U];
            e[1] = e[n - 6U];
        } else {        /* e = e + [e[7], e[8]]: the start's last space and bar. */
            e[n] = e[7];
            e[n + 1U] = e[8];
        }
        chars = barcode_read_strict(e, n + 2U, out);
    }
    return chars;
}
#else
#define barcode_read barcode_read_strict
#endif

/* Black reads higher than white. At the robot's ride height the sensor blurs the
 * bars: narrow ones are faint bumps and wide ones tall, so one threshold misses bars.
 *   1. smooth over 5 samples, in place;
 *   2. the first 0.5 s is white: its level and average spread (noise);
 *   3. a bar rises a prominence above the lowest point since the last bar and ends
 *      once it falls a third of the way back to that low point (and half a prominence below its
 *      peak), so a faint bar ends before the next one and a dip inside a solid bar
 *      does not split it;
 *   4. blur spreads a bar but keeps the area under it, so a bar's width is its area
 *      above the white either side, over the bump only (where it stays half a
 *      prominence above that white), divided by the scan's black-white range,
 *      centred on the area's centre; spaces are the gaps between bars.
 * White before the first bar and after the last is not stored. */
/* Step 3: finds the bars that rise at least prominence. Returns their count. */
static unsigned barcode_bars(const uint16_t *m, unsigned count, uint32_t prominence, unsigned *peak, unsigned *trough)
{
    unsigned bars = 0U, low_at = 0U, high_at = 0U;
    bool in_bar = false;
    for (unsigned i = 0U; i < count; ++i) {
        if (!in_bar) {
            if (m[i] < m[low_at]) { low_at = i; }
            if (m[i] >= m[low_at] + prominence) { in_bar = true; high_at = i; trough[bars] = low_at; }
        } else {
            if (m[i] > m[high_at]) { high_at = i; }
            if (m[i] + prominence / 2U <= m[high_at] &&
                BARCODE_FALL_DEN * ((uint32_t)m[high_at] - m[i]) >= BARCODE_FALL_NUM * ((uint32_t)m[high_at] - m[trough[bars]])) {
                peak[bars++] = high_at;
                in_bar = false;
                low_at = i;
                if (bars == BARCODE_MAX_BARS) { break; }
            }
        }
    }
    if (in_bar && bars < BARCODE_MAX_BARS) { peak[bars++] = high_at; low_at = count - 1U; }
    trough[bars] = low_at;
    return bars;
}

/* Median of the bars' peak heights above white (sorts a copy). */
static uint32_t barcode_median_height(const uint16_t *m, const unsigned *peak, unsigned bars, uint32_t white)
{
    uint32_t h[BARCODE_MAX_BARS];
    for (unsigned k = 0U; k < bars; ++k) { h[k] = (m[peak[k]] > white) ? m[peak[k]] - white : 0U; }
    for (unsigned k = 1U; k < bars; ++k) { /* Insertion sort: at most 50 values. */
        uint32_t v = h[k];
        unsigned j = k;
        while (j > 0U && h[j - 1U] > v) { h[j] = h[j - 1U]; --j; }
        h[j] = v;
    }
    return (bars == 0U) ? 0U : h[bars / 2U];
}

static unsigned barcode_elements(uint16_t *m, unsigned count, uint32_t *d)
{
    uint16_t raw[BARCODE_SMOOTH] = {0}; /* Unsmoothed values of the last BARCODE_SMOOTH samples. */
    unsigned peak[BARCODE_MAX_BARS], trough[BARCODE_MAX_BARS + 1U]; /* Trough: lowest point before each bar, and after the last. */
    unsigned bars, n = 0U;
    uint32_t white = 0U, noise = 0U, prominence, highest = 0U, contrast;
    double previous_end = 0.0;
    if (count < BARCODE_QUIET || count > BARCODE_SAMPLES) { return 0U; }
    for (unsigned i = 0U; i < count; ++i) {
        unsigned from = (i < BARCODE_SMOOTH) ? 0U : i - BARCODE_SMOOTH;
        unsigned to = (i + BARCODE_SMOOTH >= count) ? count - 1U : i + BARCODE_SMOOTH;
        uint32_t sum = 0U;
        for (unsigned j = from; j <= to; ++j) { sum += (j < i) ? raw[j % BARCODE_SMOOTH] : m[j]; }
        raw[i % BARCODE_SMOOTH] = m[i];
        m[i] = (uint16_t)(sum / (to - from + 1U));
    }
    for (unsigned i = 0U; i < BARCODE_QUIET; ++i) { white += m[i]; }
    white /= BARCODE_QUIET;
    for (unsigned i = 0U; i < BARCODE_QUIET; ++i) { noise += (m[i] > white) ? m[i] - white : white - m[i]; }
    noise /= BARCODE_QUIET;
    prominence = BARCODE_BAR_GAIN * noise + BARCODE_BAR_RISE;
    for (unsigned i = 0U; i < count; ++i) { if (m[i] > highest) { highest = m[i]; } }
    if (highest <= white) { return 0U; }
#if BARCODE_CONTRAST_DIV > 0
    /* The noise alone sets the first pass; the bars' median height then scales the prominence, so a
     * faint code keeps its faint bars and a dip inside a solid bar of a strong code does not split it. */
    contrast = barcode_median_height(m, peak, barcode_bars(m, count, prominence, peak, trough), white);
    if (contrast / BARCODE_CONTRAST_DIV > prominence) { prominence = contrast / BARCODE_CONTRAST_DIV; }
#endif
    bars = barcode_bars(m, count, prominence, peak, trough);
#ifdef BARCODE_DEBUG
    BARCODE_DEBUG(m, count, peak, trough, bars, white, prominence);
#endif
    contrast = highest - white;
    for (unsigned k = 0U; k < bars; ++k) {
#if BARCODE_REF_SPAN > 0
        /* Black level near this bar: the tallest bar within BARCODE_REF_SPAN either side (a character
         * spans 5 bars and nearly always has a wide one), so a bar the sensor met off-centre does not
         * shrink the widths of the whole pass. */
        unsigned lo = (k < BARCODE_REF_SPAN) ? 0U : k - BARCODE_REF_SPAN;
        unsigned hi = (k + BARCODE_REF_SPAN >= bars) ? bars - 1U : k + BARCODE_REF_SPAN;
        contrast = 1U;
        for (unsigned j = lo; j <= hi; ++j) {
            if (m[peak[j]] > white && m[peak[j]] - white > contrast) { contrast = m[peak[j]] - white; }
        }
#endif
        double base = ((double)m[trough[k]] + (double)m[trough[k + 1U]]) / 2.0;
        double edge = base + (double)prominence / 2.0;
        double area = 0.0, moment = 0.0, width, centre;
        unsigned start = peak[k], end = peak[k];
        while (start > trough[k] && (double)m[start - 1U] > edge) { --start; }
        while (end < trough[k + 1U] && (double)m[end + 1U] > edge) { ++end; }
        for (unsigned i = start; i <= end; ++i) {
            double above = (double)m[i] - base;
            if (above > 0.0) { area += above; moment += above * (double)i; }
        }
        if (area <= 0.0) { continue; }
        width = area / (double)contrast; /* In samples. */
        centre = moment / area;
        if (n > 0U) {
            double gap = (centre - width / 2.0) - previous_end;
            d[n++] = (uint32_t)(((gap > 1.0) ? gap : 1.0) * BARCODE_SAMPLE_US);
        }
        d[n++] = (uint32_t)(width * BARCODE_SAMPLE_US);
        previous_end = centre + width / 2.0;
    }
    return n;
}

uint32_t barcode_level(const uint16_t *s, unsigned n, uint32_t *noise)
{
    uint32_t mean = 0U, spread = 0U;
    for (unsigned i = 0U; i < n; ++i) { mean += s[i]; }
    mean /= n;
    for (unsigned i = 0U; i < n; ++i) { spread += (s[i] > mean) ? s[i] - mean : mean - s[i]; }
    *noise = spread / n;
    return mean;
}

/* How far a reading must leave the quiet level to count as a bar. */
static uint32_t barcode_prominence(uint32_t noise)
{
    return BARCODE_NOISE_GAIN * noise + BARCODE_MIN_RISE;
}

void barcode_range(const uint16_t *s, unsigned n, uint16_t *low, uint16_t *high)
{
    *low = 0xFFFFU;
    *high = 0U;
    for (unsigned i = 0U; i < n; ++i) {
        if (s[i] < *low) { *low = s[i]; }
        if (s[i] > *high) { *high = s[i]; }
    }
}

/* True when the end of the stretch lies within two prominences (the range of 50 noisy samples is
 * wider than the noise-based prominence; a bar rise is far larger). The flat end is the last
 * BARCODE_FLAT samples, and while the car drives it must also span BARCODE_FLAT_PATH of floor, so a
 * wide space crossed slowly does not end the pass. Standing still (or a hand scan): time only. */
static bool barcode_flat(const barcode_seg_t *g)
{
    unsigned from = g->count - BARCODE_FLAT;
    uint32_t end = g->path[g->count - 1U];
    uint16_t low, high;
    if (end - g->path[from] >= BARCODE_STILL_PATH) {
        while (from > BARCODE_QUIET && end - g->path[from] < BARCODE_FLAT_PATH) { --from; }
        if (end - g->path[from] < BARCODE_FLAT_PATH) { return false; } /* Not yet 25 mm since the start. */
    }
    barcode_range(&g->samples[from], g->count - from, &low, &high);
    return (uint32_t)(high - low) < 2U * g->prominence;
}

/* Copies a stretch into out, resampled to equal steps of driven distance so that a bar's width
 * does not depend on the car's speed (it speeds up and slows down while wiggling). The quiet
 * lead-in is kept as it is: it is only the noise reference. A stretch the car hardly moved
 * through (hand scan, no path) is copied unchanged. Returns the new length. */
static unsigned barcode_resample(const uint16_t *s, const uint32_t *path, unsigned count, uint16_t *out)
{
    unsigned n = BARCODE_QUIET, j = BARCODE_QUIET;
    uint32_t at = path[BARCODE_QUIET - 1U] + BARCODE_PATH_STEP;
    for (unsigned i = 0U; i < BARCODE_QUIET; ++i) { out[i] = s[i]; }
    if (path[count - 1U] < path[BARCODE_QUIET] + BARCODE_MIN_MOVE) {
        for (unsigned i = BARCODE_QUIET; i < count; ++i) { out[i] = s[i]; }
        return count;
    }
    while (n < BARCODE_SAMPLES) {
        while (j < count && path[j] < at) { ++j; }
        if (j >= count) { break; }
        /* path[j - 1] < at <= path[j]: interpolate the reading at the distance 'at'. */
        out[n++] = (uint16_t)((int32_t)s[j - 1U] + ((int32_t)s[j] - (int32_t)s[j - 1U]) *
                              (int32_t)(at - path[j - 1U]) / (int32_t)(path[j] - path[j - 1U]));
        at += BARCODE_PATH_STEP;
    }
    return n;
}

unsigned barcode_decode_stretch(const uint16_t *samples, const uint32_t *path, unsigned count,
                                uint16_t *work, uint32_t *widths, unsigned *elements, char *text)
{
    *elements = barcode_elements(work, barcode_resample(samples, path, count, work), widths);
    return barcode_read(widths, *elements, text);
}

void barcode_seg_init(barcode_seg_t *g)
{
    g->count = 0U;
    g->head = 0U;
    g->filled = 0U;
    g->prominence = 0U;
    g->level = 0U;
    g->level_ok = false;
    g->done = false;
}

barcode_seg_event_t barcode_seg_feed(barcode_seg_t *g, uint16_t s, uint32_t path, uint16_t heading)
{
    barcode_seg_event_t event = BARCODE_SEG_RUNNING;
    if (g->done) { /* The end of the last stretch is the new lead-in. */
        for (unsigned i = 0U; i < BARCODE_QUIET; ++i) {
            g->quiet[i] = g->samples[g->count - BARCODE_QUIET + i];
            g->quiet_path[i] = g->path[g->count - BARCODE_QUIET + i];
            g->quiet_heading[i] = g->heading[g->count - BARCODE_QUIET + i];
        }
        g->head = 0U;
        g->count = 0U;
        g->done = false;
    }
    if (g->count == 0U) {
        uint32_t noise, mean = 0U;
        if (g->filled == BARCODE_QUIET) {
            mean = barcode_level(g->quiet, BARCODE_QUIET, &noise);
            g->prominence = barcode_prominence(noise);
            g->level = mean;
            g->level_ok = true;
        }
        if (g->filled < BARCODE_QUIET || ((s > mean) ? s - mean : mean - s) < g->prominence) {
            g->quiet[g->head] = s; /* Still quiet: keep it as the lead-in. */
            g->quiet_path[g->head] = path;
            g->quiet_heading[g->head] = heading;
            g->head = (g->head + 1U) % BARCODE_QUIET;
            if (g->filled < BARCODE_QUIET) { ++g->filled; }
            return BARCODE_SEG_IDLE;
        }
        /* The reading left the quiet level: start a stretch with the lead-in. */
        for (unsigned i = 0U; i < BARCODE_QUIET; ++i) {
            g->samples[i] = g->quiet[(g->head + i) % BARCODE_QUIET];
            g->path[i] = g->quiet_path[(g->head + i) % BARCODE_QUIET];
            g->heading[i] = g->quiet_heading[(g->head + i) % BARCODE_QUIET];
        }
        g->count = BARCODE_QUIET;
        event = BARCODE_SEG_STARTED;
    }
    g->path[g->count] = path;
    g->heading[g->count] = heading;
    g->samples[g->count++] = s;
    if (g->count == BARCODE_SAMPLES ||
        (g->count >= BARCODE_QUIET + BARCODE_FLAT && barcode_flat(g))) {
        g->done = true;
        return BARCODE_SEG_DONE;
    }
    return event;
}
