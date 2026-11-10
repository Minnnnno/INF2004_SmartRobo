/* Barcode replay test: feeds recorded sensor streams through the robot's own segmenter and decoder
 * (app_program/subsystem3/barcode_decode.c) and counts the reads.
 *
 * Input lines are the firmware's stream format (ROBO_BARCODE_STREAM builds):
 *   bc t=<ms> v=<10 readings> ir=<10 flags> p=<10 path values, 0.1 mm>
 * Everything else is ignored, except run headers in fixture files:
 *   @run <name> <letter or -> <minimum reads>
 * A run with a letter fails when it reads fewer than the minimum or reads any other letter.
 *
 *   barcode_decode_checks FIXTURE            checks every @run in the fixture
 *   barcode_decode_checks --replay [-v] LOG  one run per log, no expectations (tuning) */
#include "barcode_decode.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    char name[96];
    char expect;       /* '\0' = no expectation. */
    unsigned minimum;
    unsigned reads, wrong, failed, fragments;
    char letters[64];  /* Letters read, in order (truncated). */
} run_t;

static barcode_seg_t seg;
static uint16_t work[BARCODE_SAMPLES];
static uint32_t widths[BARCODE_MAX_ELEMENTS];
static int verbose;

static void run_start(run_t *r, const char *name, char expect, unsigned minimum)
{
    memset(r, 0, sizeof *r);
    (void)snprintf(r->name, sizeof r->name, "%s", name);
    r->expect = expect;
    r->minimum = minimum;
    barcode_seg_init(&seg);
}

static void describe(const char *what, const char *text, unsigned elements)
{
    uint32_t noise, white = barcode_level(seg.samples, BARCODE_QUIET, &noise);
    uint16_t low, high;
    barcode_range(seg.samples, seg.count, &low, &high);
    printf("  %s %s elements=%u samples=%u white=%u max=%u driven=%umm\n", what, text, elements, seg.count,
           (unsigned)white, (unsigned)high, (unsigned)((seg.path[seg.count - 1U] - seg.path[BARCODE_QUIET]) / 10U));
}

static void run_sample(run_t *r, uint16_t s, uint32_t path)
{
    char text[BARCODE_MAX_CHARS + 1U];
    unsigned elements, chars;
    if (barcode_seg_feed(&seg, s, path, 0U) != BARCODE_SEG_DONE) { return; }
    chars = barcode_decode_stretch(seg.samples, seg.path, seg.count, work, widths, &elements, text);
    if (chars != 0U) {
        size_t used = strlen(r->letters);
        ++r->reads;
        if ((r->expect != '\0' && text[1] != r->expect) || verbose > 1) {
            if (r->expect != '\0' && text[1] != r->expect) { ++r->wrong; }
            printf("  %s read %s, element widths:", (text[1] != r->expect) ? "WRONG" : "good", text);
            for (unsigned i = 0U; i < elements; ++i) { printf(" %u", (unsigned)(widths[i] / 1000U)); }
            printf("\n");
        }
        if (used + 1U < sizeof r->letters) { r->letters[used] = text[1]; r->letters[used + 1U] = '\0'; }
        if (verbose) { describe("read", text, elements); }
    } else if (seg.count >= 2U * BARCODE_QUIET + BARCODE_MIN_ACTIVE) {
        ++r->failed;
        if (verbose) { describe("no read", "", elements); }
    } else {
        ++r->fragments;
    }
}

static int run_end(const run_t *r)
{
    int bad = r->expect != '\0' && (r->reads < r->minimum || r->wrong != 0U);
    printf("%-28s expect=%c reads=%u (min %u) wrong=%u failed=%u fragments=%u letters=%s %s\n",
           r->name, r->expect ? r->expect : '-', r->reads, r->minimum, r->wrong, r->failed, r->fragments,
           r->letters, r->expect ? (bad ? "FAIL" : "ok") : "");
    return bad;
}

/* Parses one stream line into the run. */
static void stream_line(run_t *r, const char *line)
{
    unsigned long v[10], p[10];
    const char *vs = strstr(line, " v="), *ps = strstr(line, " p=");
    char *end;
    if (strncmp(line, "bc t=", 5U) != 0 || vs == NULL || ps == NULL) { return; }
    vs += 3;
    ps += 3;
    for (unsigned i = 0U; i < 10U; ++i) {
        v[i] = strtoul(vs, &end, 10);
        if (end == vs) { return; }
        vs = end;
        p[i] = strtoul(ps, &end, 10);
        if (end == ps) { return; }
        ps = end;
    }
    for (unsigned i = 0U; i < 10U; ++i) { run_sample(r, (uint16_t)v[i], (uint32_t)p[i]); }
}

static int replay(const char *file, int fixture)
{
    static char line[1024];
    run_t r;
    int failures = 0, open_run = 0;
    FILE *f = fopen(file, "r");
    if (f == NULL) { printf("cannot open %s\n", file); return 1; }
    if (!fixture) { run_start(&r, strrchr(file, '/') ? strrchr(file, '/') + 1 : file, '\0', 0U); open_run = 1; }
    while (fgets(line, (int)sizeof line, f) != NULL) {
        if (fixture && strncmp(line, "@run ", 5U) == 0) {
            char name[96], letter[8];
            unsigned minimum;
            if (open_run) { failures += run_end(&r); }
            if (sscanf(line + 5, "%95s %7s %u", name, letter, &minimum) != 3) {
                printf("bad run header: %s", line);
                (void)fclose(f);
                return 1;
            }
            run_start(&r, name, (letter[0] == '-') ? '\0' : letter[0], minimum);
            open_run = 1;
            if (verbose) { printf("%s\n", name); }
            continue;
        }
        if (open_run) { stream_line(&r, line); }
    }
    (void)fclose(f);
    if (open_run) { failures += run_end(&r); }
    return failures;
}

/* ---- Synthetic passes: the logs were all taken at 0.1 m/s; these cover the 0.05 m/s line speed. ----
 * Code 39 "*Z*" (wide = 3 x narrow) is drawn on a 0.1 mm grid with quiet floor either side, seen
 * through a 3.5 mm spot whose dark fraction is raised to the 4th power (the TCRT5000 sits near
 * saturation on white). That matches the logged small code: a 3 mm bar rises about half way, while a
 * 3 mm space between two bars still dips almost to white. Scaled to the given contrast over a white of
 * 150 counts, with +-3 counts of noise. The car's speed wiggles +-30 %, as line following does. */
#define SYN_SPOT 35U /* 0.1 mm cells. */
#define SYN_GRID_MAX 6000U /* 0.1 mm cells: up to 60 cm of floor. */
static float syn_dark[SYN_GRID_MAX], syn_blur[SYN_GRID_MAX];

static unsigned syn_lcg(void)
{
    static unsigned long state = 12345UL;
    state = (state * 1103515245UL + 12345UL) & 0x7FFFFFFFUL;
    return (unsigned)(state >> 16);
}

static unsigned syn_draw(const char *text, double narrow_mm)
{
    static const char symbols[] = "*Z";
    static const uint16_t patterns[] = {0x094, 0x0D0};
    unsigned at = 400U, cells = (unsigned)(narrow_mm * 10.0 + 0.5); /* 40 mm quiet floor first. */
    memset(syn_dark, 0, sizeof syn_dark);
    for (const char *c = text; *c != '\0'; ++c) {
        uint16_t pattern = patterns[strchr(symbols, *c) - symbols];
        for (unsigned e = 0U; e < 10U; ++e) { /* 9 elements, then the narrow gap. */
            unsigned width = (e < 9U && (pattern & (1U << (8U - e))) != 0U) ? 3U * cells : cells;
            if ((e % 2U) == 0U && e < 9U) {
                for (unsigned i = at; i < at + width; ++i) { syn_dark[i] = 1.0f; }
            }
            at += width;
        }
    }
    return at + 600U; /* 60 mm quiet floor after. */
}

static void syn_feed(run_t *r, double contrast, double pos_mm, uint32_t path)
{
    unsigned cell = (unsigned)(pos_mm * 10.0);
    double v = 150.0 + contrast * (double)syn_blur[cell] + (double)(syn_lcg() % 7U) - 3.0;
    run_sample(r, (uint16_t)v, path);
}

#define SYN_PASSES 10U /* Each case: 10 passes, each with its own noise and speed-wiggle phase. */

static int synthetic(const char *name, double narrow_mm, double contrast, double speed, int backward)
{
    run_t r;
    unsigned length = syn_draw("*Z*", narrow_mm);
    double travelled = 0.0;
    float sum = 0.0f;
    if (length > SYN_GRID_MAX) { printf("%s: grid too small\n", name); return 1; }
    for (unsigned i = 0U; i < length; ++i) { /* Spot average, centred, then the sensor's response. */
        float f;
        sum += syn_dark[i];
        if (i >= SYN_SPOT) { sum -= syn_dark[i - SYN_SPOT]; }
        f = sum / (float)SYN_SPOT;
        syn_blur[(i >= SYN_SPOT / 2U) ? i - SYN_SPOT / 2U : 0U] = f * f * f * f;
    }
    run_start(&r, name, 'Z', SYN_PASSES - 1U); /* The model is approximate: 9 of 10 must read, none wrong. */
    for (unsigned pass = 0U; pass < SYN_PASSES; ++pass) {
        double pos = backward ? (double)length / 10.0 - 4.0 : 4.0;
        for (unsigned k = 0U; k < 100U; ++k) { syn_feed(&r, contrast, pos, (uint32_t)(travelled * 10.0)); }
        for (unsigned k = 0U; ; ++k) { /* Standing still above, then across the code. */
            double step = speed * 1000.0 * 0.010 *
                          (1.0 + 0.3 * sin((double)k / 7.0 + 0.9 * (double)pass)); /* mm per 10 ms sample. */
            pos += backward ? -step : step;
            travelled += step;
            if (pos < 4.0 || pos > (double)length / 10.0 - 4.0) { break; }
            syn_feed(&r, contrast, pos, (uint32_t)(travelled * 10.0));
        }
        for (unsigned k = 0U; k < 100U; ++k) { syn_feed(&r, contrast, pos, (uint32_t)(travelled * 10.0)); }
    }
    return run_end(&r);
}

static int synthetic_checks(void)
{
    int failures = 0;
    for (int backward = 0; backward < 2; ++backward) {
        const char *dir = backward ? "backward" : "forward";
        char name[64];
        (void)snprintf(name, sizeof name, "synthetic small 0.05 %s", dir);
        failures += synthetic(name, 2.9, 90.0, 0.05, backward);
        (void)snprintf(name, sizeof name, "synthetic big 0.05 %s", dir);
        failures += synthetic(name, 5.9, 3300.0, 0.05, backward);
        (void)snprintf(name, sizeof name, "synthetic big weak 0.05 %s", dir);
        failures += synthetic(name, 5.9, 120.0, 0.05, backward);
        (void)snprintf(name, sizeof name, "synthetic small 0.10 %s", dir);
        failures += synthetic(name, 2.9, 90.0, 0.10, backward);
    }
    return failures;
}

int main(int argc, char **argv)
{
    int failures = 0, first = 1, fixture = 1;
    if (argc > first && strcmp(argv[first], "--replay") == 0) { fixture = 0; ++first; }
    if (argc > first && strcmp(argv[first], "-v") == 0) { verbose = 1; ++first; }
    if (argc > first && strcmp(argv[first], "-vv") == 0) { verbose = 2; ++first; }
    if (argc <= first) {
        printf("usage: %s [--replay] [-v] FILE...\n", argv[0]);
        return 2;
    }
    if (fixture) { failures += synthetic_checks(); }
    for (int i = first; i < argc; ++i) { failures += replay(argv[i], fixture); }
    printf("%s\n", failures ? "barcode decode checks FAILED" : "barcode decode checks passed");
    return failures ? 1 : 0;
}
