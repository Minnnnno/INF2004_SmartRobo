#include "subsystems.h"

/* The barcode reader runs in the line-steering image and in the barcode test image
 * (production profile built with -DCMAKE_C_FLAGS=-DROBO_BARCODE_TEST=1). Every other image
 * has no barcode_task. */
#if ROBO_BARCODE_TASK
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#include "board_config.h"
#include "motion_hal.h"
#include "robot.h"

/* Samples the barcode sensor's analog output (AO on GP28 = ADC2) all the time. A stretch
 * where the reading leaves its quiet level is kept (with 0.5 s of the quiet level before
 * it) and, once the reading is flat again, split into bars and spaces and decoded. */
#define BARCODE_ADC_BASE     0x4004C000U
#define BARCODE_ADC_CS       (BARCODE_ADC_BASE + 0x00U)
#define BARCODE_ADC_RESULT   (BARCODE_ADC_BASE + 0x04U)
#define BARCODE_ADC_EN       (1UL << 0U)
#define BARCODE_ADC_START    (1UL << 2U)
#define BARCODE_ADC_READY    (1UL << 8U)
#define BARCODE_ADC_INPUT    (2UL << 12U)  /* AINSEL 2 = GP28. */
#define BARCODE_ADC_AVERAGE  32U           /* Conversions averaged per sample. */
#define BARCODE_SAMPLE_US    10000U        /* 100 samples per second. */
#define BARCODE_SAMPLES      800U          /* 8 s: the longest stretch kept for decoding. */
#define BARCODE_SMOOTH       2U            /* Average over 2 samples either side. */
#define BARCODE_QUIET        50U           /* First 0.5 s of a stretch is the quiet level: the noise reference. */
#define BARCODE_FLAT         50U           /* A stretch ends after 0.5 s of flat reading. */
#define BARCODE_MIN_ACTIVE   80U           /* Shorter stretches are bumps, not barcode passes (0.8 s). */
#define BARCODE_NOISE_GAIN   3U            /* A bar must rise 3x the average white noise ... */
#define BARCODE_MIN_RISE     12U           /* ... plus 12 counts. */
#define BARCODE_PATH_STEP    8U            /* Decoding step: 0.8 mm of driven distance (path is in 0.1 mm). */
#define BARCODE_MIN_MOVE     100U          /* A stretch that drove under 1 cm is decoded by time instead. */
#define BARCODE_JUNCTION_MIN 60U           /* Smallest rise (counts) that counts as the junction's black. */
#define BARCODE_JUNCTION_HITS 2U           /* Consecutive samples above it (20 ms) before it is reported. */
#define BARCODE_CAP_BEFORE   32U           /* Junction capture: samples kept from before the trigger ... */
#define BARCODE_CAP_AFTER    100U          /* ... and after it (1 s), printed once as the analogue record. */
#define BARCODE_MAX_ELEMENTS 99U           /* 10 characters of 9 elements plus gaps. */
#define BARCODE_MAX_BARS     ((BARCODE_MAX_ELEMENTS + 1U) / 2U)
#define BARCODE_MAX_CHARS    ((BARCODE_MAX_ELEMENTS + 1U) / 10U)

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
        pattern |= (uint16_t)(1U << (8U - widest));
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
static unsigned barcode_read(uint32_t *d, unsigned n, char *out)
{
    unsigned chars = barcode_search(d, n, 0U, out);
    if (chars != 0U || n == 0U) { return chars; }
    barcode_reverse(d, n);
    chars = barcode_search(d, n, (n - 1U) % 2U, out);
    barcode_reverse(d, n);
    return chars;
}

static void barcode_adc_init(void)
{
    clr_w(RESETS_RESET, RESETS_RESET_ADC);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_ADC) == 0U) { }
    out_w(GPIO_CTRL(BOARD_BARCODE), GPIO_CTRL_FUNCSEL_NULL);
    out_w(GPIO(BOARD_BARCODE), GPIO_OD); /* Digital input and pulls off for analog use. */
    out_w(BARCODE_ADC_CS, BARCODE_ADC_EN | BARCODE_ADC_INPUT);
    while ((in_w(BARCODE_ADC_CS) & BARCODE_ADC_READY) == 0U) { }
}

/* Averages BARCODE_ADC_AVERAGE conversions (about 2 us each): the RP2040 ADC's own
 * noise is most of the white-level spread, and the small barcode's narrow bars rise
 * little more than that. */
static uint16_t barcode_adc_read(void)
{
    uint32_t sum = 0U;
    for (unsigned i = 0U; i < BARCODE_ADC_AVERAGE; ++i) {
        out_w(BARCODE_ADC_CS, BARCODE_ADC_EN | BARCODE_ADC_INPUT | BARCODE_ADC_START);
        while ((in_w(BARCODE_ADC_CS) & BARCODE_ADC_READY) == 0U) { }
        sum += in_w(BARCODE_ADC_RESULT) & 0xFFFU;
    }
    return (uint16_t)(sum / BARCODE_ADC_AVERAGE);
}

/* Each sample averages a reading every 1 ms tick across its whole period, so light
 * flicker (the white-level spread that a 64 us burst does not remove) cancels out. */
static uint16_t barcode_sample(uint64_t *next)
{
    uint32_t sum = 0U, reads = 0U;
    if (time_us_64() > *next + 100000U) { *next = time_us_64(); } /* Fell behind: do not burst. */
    do {
        sum += barcode_adc_read();
        ++reads;
        (void)tk_dly_tsk(1U);
    } while (time_us_64() < *next);
    *next += BARCODE_SAMPLE_US;
    return (uint16_t)(sum / reads);
}

/* Black reads higher than white. At the robot's ride height the sensor blurs the
 * bars: narrow ones are faint bumps and wide ones tall, so one threshold misses bars.
 *   1. smooth over 5 samples, in place;
 *   2. the first 0.5 s is white: its level and average spread (noise);
 *   3. a bar rises a prominence above the lowest point since the last bar and ends
 *      once it falls halfway back to that low point (and half a prominence below its
 *      peak), so a faint bar ends before the next one and a dip inside a solid bar
 *      does not split it;
 *   4. blur spreads a bar but keeps the area under it, so a bar's width is its area
 *      above the white either side, over the bump only (where it stays half a
 *      prominence above that white), divided by the scan's black-white range,
 *      centred on the area's centre; spaces are the gaps between bars.
 * White before the first bar and after the last is not stored. */
static unsigned barcode_elements(uint16_t *m, unsigned count, uint32_t *d)
{
    uint16_t raw[BARCODE_SMOOTH] = {0}; /* Unsmoothed values of the last BARCODE_SMOOTH samples. */
    unsigned peak[BARCODE_MAX_BARS], trough[BARCODE_MAX_BARS + 1U]; /* Trough: lowest point before each bar, and after the last. */
    unsigned bars = 0U, n = 0U, low_at = 0U, high_at = 0U;
    uint32_t white = 0U, noise = 0U, prominence, highest = 0U;
    double previous_end = 0.0;
    bool in_bar = false;
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
    prominence = BARCODE_NOISE_GAIN * noise + BARCODE_MIN_RISE;
    for (unsigned i = 0U; i < count; ++i) { if (m[i] > highest) { highest = m[i]; } }
    if (highest <= white) { return 0U; }
    for (unsigned i = 0U; i < count; ++i) {
        if (!in_bar) {
            if (m[i] < m[low_at]) { low_at = i; }
            if (m[i] >= m[low_at] + prominence) { in_bar = true; high_at = i; trough[bars] = low_at; }
        } else {
            if (m[i] > m[high_at]) { high_at = i; }
            if (m[i] + prominence / 2U <= m[high_at] &&
                2U * (uint32_t)m[i] <= (uint32_t)m[high_at] + m[trough[bars]]) {
                peak[bars++] = high_at;
                in_bar = false;
                low_at = i;
                if (bars == BARCODE_MAX_BARS) { break; }
            }
        }
    }
    if (in_bar && bars < BARCODE_MAX_BARS) { peak[bars++] = high_at; low_at = count - 1U; }
    trough[bars] = low_at;
    for (unsigned k = 0U; k < bars; ++k) {
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
        width = area / (double)(highest - white); /* In samples. */
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

/* Mean and mean absolute deviation (the noise) of n samples. */
static uint32_t barcode_level(const uint16_t *s, unsigned n, uint32_t *noise)
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

static void barcode_range(const uint16_t *s, unsigned n, uint16_t *low, uint16_t *high)
{
    *low = 0xFFFFU;
    *high = 0U;
    for (unsigned i = 0U; i < n; ++i) {
        if (s[i] < *low) { *low = s[i]; }
        if (s[i] > *high) { *high = s[i]; }
    }
}

/* True when the last BARCODE_FLAT samples lie within two prominences of each other (the range
 * of 50 noisy samples is wider than the noise-based prominence). A bar rise is far larger. */
static bool barcode_flat(const uint16_t *s, unsigned count, uint32_t prominence)
{
    uint16_t low, high;
    barcode_range(&s[count - BARCODE_FLAT], BARCODE_FLAT, &low, &high);
    return (uint32_t)(high - low) < 2U * prominence;
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

/* Published to the bus (see robot.h): letter, valid read counter, junction counter. */
static unsigned bc_reads;
static double bc_letter;
static unsigned bc_junctions;
static unsigned bc_failures;       /* Long passes that did not decode (line images stop the car on this). */
static uint32_t bc_sample_no;      /* Counts samples, to place stretch starts and the junction in time. */
static uint32_t bc_stretch_start;  /* Sample number at which the current stretch began. */
static uint32_t bc_junction_sample;/* Sample number of the last junction trigger. */
static bool bc_junction_seen;
#define BARCODE_JUNCTION_BEFORE 300U /* A stretch starting this far before the trigger, or ... */
#define BARCODE_FAIL_MIN_DRIVE 500U /* A failed pass counts only if the car drove 5 cm (0.1 mm units) during it. */
#define BARCODE_JUNCTION_AFTER  1000U /* ... up to 10 s after it, is the junction's own bar or the turn: no failure. */
#if ROBO_LINE_TASK
static bool bc_armed;           /* A valid read was made: the next black is the junction. */
static uint32_t bc_arm_rise;    /* Rise over the quiet level that counts as that black. */
#endif

static bool bc_dirty; /* The last publish failed (bus busy): retried every sample until it goes through. */

static void barcode_publish(void)
{
    subsystem_status_t status = {.available = true, .timestamp = motion_hal_time(),
        .values = {bc_letter, (double)bc_reads, (double)bc_junctions, (double)bc_failures}};
    bc_dirty = !robot_update_subsystem(SOURCE_BARCODE, &status);
}

static uint16_t bc_heading; /* Heading (wrapping milliradians) when the status was last read, see barcode_path(). */

/* One number of a prefixed dump line (20 per line); the pause lets the USB buffer drain. */
static void barcode_dump_num(const char *tag, unsigned i, unsigned n, int32_t value)
{
    if ((i % 20U) == 0U) { tm_printf((const UB *)"%s", (const UB *)tag); }
    tm_printf((const UB *)" %d%s", (int)value, ((i % 20U) == 19U || i + 1U == n) ? "\n" : "");
    if ((i % 20U) == 19U) { (void)tk_dly_tsk(20U); }
}

/* Decodes one stretch (a resampled copy: the decoder smooths in place) and reports the result.
 * A pass of at least BARCODE_MIN_ACTIVE samples beyond its quiet lead-in that does not read
 * prints what the sensor saw. */
static void barcode_finish(const uint16_t *samples, const uint32_t *path, const uint16_t *heading,
                           unsigned count, unsigned *reads)
{
    static uint16_t work[BARCODE_SAMPLES];
    static uint32_t durations[BARCODE_MAX_ELEMENTS];
    char text[BARCODE_MAX_CHARS + 1U];
    unsigned n, chars;
    n = barcode_elements(work, barcode_resample(samples, path, count, work), durations);
    chars = barcode_read(durations, n, text);
    if (chars != 0U) {
        text[chars - 1U] = '\0'; /* Drop the closing '*'. */
        ++*reads;
        tm_printf((const UB *)"barcode: *%s*\n", (const UB *)&text[1]);
        bc_reads = *reads;
        bc_letter = (double)(unsigned char)text[1];
        barcode_publish();
#if ROBO_LINE_TASK
        {   /* The next black after this read is the junction: half the bar rise just seen counts. */
            uint32_t noise, white = barcode_level(samples, BARCODE_QUIET, &noise);
            uint16_t low, high;
            barcode_range(samples, count, &low, &high);
            bc_arm_rise = (high > white) ? (high - white) / 2U : 0U;
            if (bc_arm_rise < BARCODE_JUNCTION_MIN) { bc_arm_rise = BARCODE_JUNCTION_MIN; }
            bc_armed = true;
            tm_printf((const UB *)"barcode: armed, white=%u peak=%u junction=white+%u\n",
                      (unsigned)white, (unsigned)high, (unsigned)bc_arm_rise);
        }
#endif
    } else if (count >= 2U * BARCODE_QUIET + BARCODE_MIN_ACTIVE) {
        uint32_t noise, white = barcode_level(samples, BARCODE_QUIET, &noise);
        uint16_t low, high;
        barcode_range(samples, count, &low, &high);
        tm_printf((const UB *)"barcode: no read, elements=%u samples=%u white=%u min=%u max=%u\n",
                  n, count, (unsigned)white, (unsigned)low, (unsigned)high);
#if ROBO_LINE_TASK
        /* Reported before the slow dump below, so the car stops as early as it can. The junction's own
         * bar and the turn that follows are not failed reads. */
        if (path[count - 1U] - path[BARCODE_QUIET] < BARCODE_FAIL_MIN_DRIVE) {
            /* Placing the car on the track, or a spin on the spot, can move the reading without a read. */
            tm_printf((const UB *)"barcode: no read while not driving, not counted\n");
        } else if (!(bc_junction_seen && bc_stretch_start + BARCODE_JUNCTION_BEFORE >= bc_junction_sample &&
                     bc_stretch_start <= bc_junction_sample + BARCODE_JUNCTION_AFTER)) {
            ++bc_failures;
            barcode_publish();
        }
#endif
#ifndef ROBO_BARCODE_STREAM /* The stream already printed every sample; the paced dump would blind the reader. */
        /* bv = readings, bp = distance driven in mm since the first sample, bh = heading change in
         * milliradians (the sensor sits to one side of the wheels, so steering changes how far it
         * travels), bw = the decoded element widths in 0.1 mm. */
        for (unsigned i = 0U; i < count; ++i) { barcode_dump_num("bv", i, count, (int32_t)samples[i]); }
        for (unsigned i = 0U; i < count; ++i) { barcode_dump_num("bp", i, count, (int32_t)((path[i] - path[0]) / 10U)); }
        for (unsigned i = 0U; i < count; ++i) {
            barcode_dump_num("bh", i, count, (int32_t)(int16_t)(uint16_t)(heading[i] - heading[0]));
        }
        for (unsigned i = 0U; i < n; ++i) { barcode_dump_num("bw", i, n, (int32_t)(durations[i] / 1250U)); }
#endif
    } else {
        uint32_t noise, white = barcode_level(samples, BARCODE_QUIET, &noise);
        uint16_t low, high;
        barcode_range(samples, count, &low, &high);
        tm_printf((const UB *)"barcode: stretch ended, no code (samples=%u elements=%u white=%u min=%u max=%u)\n",
                  count, n, (unsigned)white, (unsigned)low, (unsigned)high);
    }
}

/* Distance the car has driven, in 0.1 mm. Holds the last value if the status is unavailable, so
 * a stretch then looks like the car did not move and is decoded by time. */
static uint32_t barcode_path(void)
{
    static uint32_t last;
    robot_status_t status;
    if (motion_get_status(&status)) {
        if (status.motion.path >= 0.0) { last = (uint32_t)(status.motion.path * 10000.0); }
        bc_heading = (uint16_t)(uint32_t)(int32_t)(status.motion.heading * 1000.0); /* Wraps; only differences are used. */
    }
    return last;
}

#ifdef ROBO_BARCODE_STREAM
/* Analysis build: prints every reading, 10 per line (0.1 s), with the time in ms, the line IR
 * state of each sample ('0' + centre + 2 * right) and the distance the car has driven at each
 * sample (0.1 mm units), to see how the car's wiggle and speed line up with the sensor. */
static void barcode_stream(uint16_t s, uint32_t path)
{
    static uint16_t v[10];
    static char ir[10];
    static uint32_t p[10];
    static unsigned n;
    v[n] = s;
    ir[n] = (char)('0' + (unsigned)gpio_get(BOARD_LINE_C) + 2U * (unsigned)gpio_get(BOARD_LINE_R));
    p[n] = path;
    if (++n < 10U) { return; }
    n = 0U;
    tm_printf((const UB *)"bc t=%u v=%u %u %u %u %u %u %u %u %u %u ir=%c%c%c%c%c%c%c%c%c%c p=%u %u %u %u %u %u %u %u %u %u\n",
              (unsigned)(motion_hal_time() * 1000.0),
              (unsigned)v[0], (unsigned)v[1], (unsigned)v[2], (unsigned)v[3], (unsigned)v[4],
              (unsigned)v[5], (unsigned)v[6], (unsigned)v[7], (unsigned)v[8], (unsigned)v[9],
              ir[0], ir[1], ir[2], ir[3], ir[4], ir[5], ir[6], ir[7], ir[8], ir[9],
              p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9]);
}
#else
#define barcode_stream(s, path) ((void)(s), (void)(path))
#endif

#if defined(ROBO_BARCODE_TEST) && !defined(ROBO_LINE_STEERING)
/* Barcode-only image: shows the sensor level every 2 s while idle so it never looks dead. */
static void barcode_idle(uint16_t s)
{
    static unsigned ticks;
    if (++ticks >= 200U) {
        ticks = 0U;
        tm_printf((const UB *)"barcode: idle, sensor level=%u\n", (unsigned)s);
    }
}
#else
#define barcode_idle(s) ((void)(s))
#endif

#if ROBO_LINE_TASK
/* The analogue record of the junction's black: BARCODE_CAP_BEFORE samples up to the trigger and
 * BARCODE_CAP_AFTER after it, printed once when complete (20 values per line, 10 ms each). */
static uint16_t cap[BARCODE_CAP_BEFORE + BARCODE_CAP_AFTER];
static unsigned cap_n, cap_target;

static void barcode_cap_print(void)
{
    tm_printf((const UB *)"barcode: junction samples, %u before the trigger + %u after, 10 ms each\n",
              BARCODE_CAP_BEFORE, BARCODE_CAP_AFTER);
    for (unsigned i = 0U; i < cap_n; ++i) {
        tm_printf((const UB *)"%u%s", (unsigned)cap[i], ((i % 20U) == 19U || i + 1U == cap_n) ? "\n" : " ");
        if ((i % 20U) == 19U) { (void)tk_dly_tsk(20U); } /* Let the USB buffer drain. */
    }
    cap_target = 0U;
}
#endif

void barcode_task(INT code, void *context)
{
    static uint16_t samples[BARCODE_SAMPLES];
    static uint32_t path[BARCODE_SAMPLES]; /* Distance driven at each sample. */
    static uint16_t heading[BARCODE_SAMPLES]; /* Heading at each sample, wrapping milliradians. */
    uint16_t quiet[BARCODE_QUIET]; /* Ring of the last 0.5 s before a stretch starts. */
    uint32_t quiet_path[BARCODE_QUIET];
    uint16_t quiet_heading[BARCODE_QUIET];
    unsigned head = 0U, filled = 0U, count = 0U, reads = 0U;
    uint32_t prominence = 0U;
    uint64_t next;
#if ROBO_LINE_TASK
    uint16_t hist[BARCODE_CAP_BEFORE]; /* Ring of the last samples, for the junction record. */
    unsigned hist_head = 0U, hits = 0U;
    uint32_t ref = 0U; /* Quiet level, from the last time no stretch was running. */
    bool ref_ok = false;
    for (unsigned i = 0U; i < BARCODE_CAP_BEFORE; ++i) { hist[i] = 0U; }
#endif
    (void)code; (void)context;
    barcode_adc_init();
    tm_printf((const UB *)"barcode: reader running on GP28 (distance widths)\n");
    next = time_us_64() + BARCODE_SAMPLE_US;
    for (;;) {
        uint16_t s = barcode_sample(&next);
        uint32_t at = barcode_path();
        ++bc_sample_no;
        barcode_stream(s, at);
        if (bc_dirty) { barcode_publish(); }
#if ROBO_LINE_TASK
        hist[hist_head] = s;
        hist_head = (hist_head + 1U) % BARCODE_CAP_BEFORE;
        if (cap_n < cap_target) { /* Samples after the trigger. */
            cap[cap_n++] = s;
            if (cap_n == cap_target) { barcode_cap_print(); }
        }
        /* After a valid read, the first black (clearly above the quiet level for two samples in a row)
         * is the junction. Reported at once; the line task decides what to do about it. */
        if (bc_armed && ref_ok && s > ref + bc_arm_rise) {
            if (++hits >= BARCODE_JUNCTION_HITS) {
                hits = 0U;
                bc_armed = false;
                bc_junction_sample = bc_sample_no;
                bc_junction_seen = true;
                ++bc_junctions;
                barcode_publish();
                tm_printf((const UB *)"barcode: junction black=%u white=%u threshold=%u path=%u\n",
                          (unsigned)s, (unsigned)ref, (unsigned)(ref + bc_arm_rise), (unsigned)at);
                for (unsigned i = 0U; i < BARCODE_CAP_BEFORE; ++i) {
                    cap[i] = hist[(hist_head + i) % BARCODE_CAP_BEFORE]; /* Oldest first, trigger last. */
                }
                cap_n = BARCODE_CAP_BEFORE;
                cap_target = BARCODE_CAP_BEFORE + BARCODE_CAP_AFTER;
            }
        } else {
            hits = 0U;
        }
#endif
        if (count == 0U) {
            uint32_t noise, mean = 0U;
            if (filled == BARCODE_QUIET) {
                mean = barcode_level(quiet, BARCODE_QUIET, &noise);
                prominence = barcode_prominence(noise);
#if ROBO_LINE_TASK
                ref = mean;
                ref_ok = true;
#endif
            }
            if (filled < BARCODE_QUIET || ((s > mean) ? s - mean : mean - s) < prominence) {
                quiet[head] = s; /* Still quiet: keep it as the lead-in. */
                quiet_path[head] = at;
                quiet_heading[head] = bc_heading;
                head = (head + 1U) % BARCODE_QUIET;
                if (filled < BARCODE_QUIET) { ++filled; }
                barcode_idle(s);
                continue;
            }
            /* The reading left the quiet level: start a stretch with the lead-in. */
            for (unsigned i = 0U; i < BARCODE_QUIET; ++i) {
                samples[i] = quiet[(head + i) % BARCODE_QUIET];
                path[i] = quiet_path[(head + i) % BARCODE_QUIET];
                heading[i] = quiet_heading[(head + i) % BARCODE_QUIET];
            }
            count = BARCODE_QUIET;
            bc_stretch_start = bc_sample_no;
            tm_printf((const UB *)"barcode: reading...\n");
        }
        path[count] = at;
        heading[count] = bc_heading;
        samples[count++] = s;
        if (count == BARCODE_SAMPLES ||
            (count >= BARCODE_QUIET + BARCODE_FLAT && barcode_flat(samples, count, prominence))) {
            barcode_finish(samples, path, heading, count, &reads);
            for (unsigned i = 0U; i < BARCODE_QUIET; ++i) { /* The end of the stretch is the new lead-in. */
                quiet[i] = samples[count - BARCODE_QUIET + i];
                quiet_path[i] = path[count - BARCODE_QUIET + i];
                quiet_heading[i] = heading[count - BARCODE_QUIET + i];
            }
            head = 0U;
            count = 0U;
        }
    }
}
#endif
