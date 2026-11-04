#include "subsystems.h"

/* Opt-in hand-scan test: build the production profile with
 * -DCMAKE_C_FLAGS=-DROBO_BARCODE_TEST=1. Every other image keeps barcode_task idle. */
#if defined(ROBO_BARCODE_TEST) && defined(ROBO_PRODUCTION_BUILD) && !defined(ROBO_LINE_STEERING)
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#include "board_config.h"

/* Captures the barcode sensor's analog output (AO on GP28 = ADC2) while a Code 39
 * barcode is moved by hand, prints the raw samples, then splits them into bars and
 * spaces at the midpoint of that scan's darkest and lightest reading and decodes. */
#define BARCODE_ADC_BASE     0x4004C000U
#define BARCODE_ADC_CS       (BARCODE_ADC_BASE + 0x00U)
#define BARCODE_ADC_RESULT   (BARCODE_ADC_BASE + 0x04U)
#define BARCODE_ADC_EN       (1UL << 0U)
#define BARCODE_ADC_START    (1UL << 2U)
#define BARCODE_ADC_READY    (1UL << 8U)
#define BARCODE_ADC_INPUT    (2UL << 12U)  /* AINSEL 2 = GP28. */
#define BARCODE_ADC_AVERAGE  32U           /* Conversions averaged per sample. */
#define BARCODE_SAMPLE_US    10000U        /* 100 samples per second. */
#define BARCODE_SAMPLES      3000U         /* 30 s capture. */
#define BARCODE_SMOOTH       2U            /* Average over 2 samples either side. */
#define BARCODE_QUIET        50U           /* First 0.5 s is white: the noise reference. */
#define BARCODE_NOISE_GAIN   4U            /* A bar must rise 4x the average white noise ... */
#define BARCODE_MIN_RISE     25U           /* ... plus 25 counts. */
#define BARCODE_MAX_ELEMENTS 99U           /* 10 characters of 9 elements plus gaps. */
#define BARCODE_MAX_BARS     ((BARCODE_MAX_ELEMENTS + 1U) / 2U)
#define BARCODE_MAX_CHARS    ((BARCODE_MAX_ELEMENTS + 1U) / 10U)

/* Code 39: 9 elements (bar, space, ... bar), bit 8 = first element, 1 = wide. */
static const struct { uint16_t pattern; char symbol; } code39[] = {
    {0x034, '0'}, {0x121, '1'}, {0x061, '2'}, {0x160, '3'}, {0x031, '4'}, {0x130, '5'},
    {0x070, '6'}, {0x025, '7'}, {0x124, '8'}, {0x064, '9'}, {0x109, 'A'}, {0x049, 'B'},
    {0x148, 'C'}, {0x019, 'D'}, {0x118, 'E'}, {0x058, 'F'}, {0x00D, 'G'}, {0x10C, 'H'},
    {0x04C, 'I'}, {0x01C, 'J'}, {0x103, 'K'}, {0x043, 'L'}, {0x142, 'M'}, {0x013, 'N'},
    {0x112, 'O'}, {0x052, 'P'}, {0x007, 'Q'}, {0x106, 'R'}, {0x046, 'S'}, {0x016, 'T'},
    {0x181, 'U'}, {0x0C1, 'V'}, {0x1C0, 'W'}, {0x091, 'X'}, {0x190, 'Y'}, {0x0D0, 'Z'},
    {0x085, '-'}, {0x184, '.'}, {0x0C4, ' '}, {0x094, '*'}, {0x0A8, '$'}, {0x0A2, '/'},
    {0x08A, '+'}, {0x02A, '%'},
};

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
    for (unsigned i = 0U; i < sizeof code39 / sizeof code39[0]; ++i) {
        if (code39[i].pattern == pattern) { return code39[i].symbol; }
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
    if (chars < 2U || chars > BARCODE_MAX_CHARS || n != chars * 10U - 1U) { return false; }
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
    for (unsigned start = first; start + 19U <= n; start += 2U) {
        for (unsigned chars = BARCODE_MAX_CHARS; chars >= 2U; --chars) {
            unsigned length = chars * 10U - 1U;
            if (start + length <= n && barcode_decode(&d[start], length, out)) { return chars; }
        }
    }
    return 0U;
}

/* Forward first, then reversed. Elements alternate bar/space from a bar at index 0. */
static unsigned barcode_read(const uint32_t *d, unsigned n, char *out)
{
    uint32_t reversed[BARCODE_MAX_ELEMENTS];
    unsigned chars = barcode_search(d, n, 0U, out);
    if (chars != 0U || n == 0U) { return chars; }
    for (unsigned i = 0U; i < n; ++i) { reversed[i] = d[n - 1U - i]; }
    return barcode_search(reversed, n, (n - 1U) % 2U, out);
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
static void barcode_capture(uint16_t *s)
{
    uint64_t next = time_us_64() + BARCODE_SAMPLE_US;
    uint32_t sum = 0U, reads = 0U;
    for (unsigned i = 0U; i < BARCODE_SAMPLES; ) {
        sum += barcode_adc_read();
        ++reads;
        if (time_us_64() >= next) {
            s[i++] = (uint16_t)(sum / reads);
            sum = 0U;
            reads = 0U;
            next += BARCODE_SAMPLE_US;
        }
        (void)tk_dly_tsk(1U);
    }
}

/* Black reads higher than white. At the robot's ride height the sensor blurs the
 * bars: narrow ones are faint bumps and wide ones tall, so one threshold misses bars.
 *   1. smooth over 5 samples;
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
static unsigned barcode_elements(const uint16_t *s, unsigned count, uint32_t *d)
{
    static uint16_t m[BARCODE_SAMPLES];
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
        for (unsigned j = from; j <= to; ++j) { sum += s[j]; }
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

void barcode_task(INT code, void *context)
{
    static uint16_t samples[BARCODE_SAMPLES];
    static uint32_t durations[BARCODE_MAX_ELEMENTS];
    char text[BARCODE_MAX_CHARS + 1U];
    (void)code; (void)context;
    barcode_adc_init();
    for (;;) {
        uint16_t low = 4095U, high = 0U;
        unsigned n;
        tm_printf((const UB *)"barcode: hold on white, scan starts in 3 s\n");
        (void)tk_dly_tsk(3000U);
        tm_printf((const UB *)"barcode: GO (30 s)\n");
        barcode_capture(samples);
        for (unsigned i = 0U; i < BARCODE_SAMPLES; ++i) {
            if (samples[i] < low) { low = samples[i]; }
            if (samples[i] > high) { high = samples[i]; }
        }
        tm_printf((const UB *)"barcode_adc: min=%u max=%u first=%u every %u ms avg=%u spread\n",
                  (unsigned)low, (unsigned)high, (unsigned)samples[0], BARCODE_SAMPLE_US / 1000U, BARCODE_ADC_AVERAGE);
        for (unsigned i = 0U; i < BARCODE_SAMPLES; ++i) {
            tm_printf((const UB *)"%u%s", (unsigned)samples[i], ((i % 20U) == 19U) ? "\n" : " ");
            if ((i % 20U) == 19U) { (void)tk_dly_tsk(20U); } /* Let the USB buffer drain. */
        }
        n = barcode_elements(samples, BARCODE_SAMPLES, durations);
        unsigned chars = barcode_read(durations, n, text);
        if (chars != 0U) {
            text[chars - 1U] = '\0'; /* Drop the closing '*'. */
            tm_printf((const UB *)"barcode: *%s* -> %s\n", (const UB *)&text[1], (const UB *)&text[1]);
        } else {
            tm_printf((const UB *)"barcode: decode failed (%u elements)\n", n);
        }
        for (unsigned i = 0U; i < n; ++i) {
            tm_printf((const UB *)"%c%lu ", (i % 2U == 0U) ? 'B' : 'W', (unsigned long)(durations[i] / 1000U));
        }
        tm_printf((const UB *)"\n");
        (void)tk_dly_tsk(2000U);
    }
}

#else
/* Placeholder: barcode sensing (GP28, BOARD_BARCODE) and decoding are not implemented. */
void barcode_task(INT code, void *context)
{ (void)code; (void)context; for (;;) { (void)tk_slp_tsk(TMO_FEVR); } }
#endif
