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
#include "barcode_decode.h"

/* Samples the barcode sensor's analog output (AO on GP28 = ADC2) all the time. barcode_decode.c
 * keeps a stretch where the reading leaves its quiet level (with 0.5 s of the quiet level before
 * it) and, once the reading is flat again, splits it into bars and spaces and decodes it. */
#define BARCODE_ADC_BASE     0x4004C000U
#define BARCODE_ADC_CS       (BARCODE_ADC_BASE + 0x00U)
#define BARCODE_ADC_RESULT   (BARCODE_ADC_BASE + 0x04U)
#define BARCODE_ADC_EN       (1UL << 0U)
#define BARCODE_ADC_START    (1UL << 2U)
#define BARCODE_ADC_READY    (1UL << 8U)
#define BARCODE_ADC_INPUT    (2UL << 12U)  /* AINSEL 2 = GP28. */
#define BARCODE_ADC_AVERAGE  32U           /* Conversions averaged per sample. */
#define BARCODE_JUNCTION_MIN 60U           /* Smallest rise (counts) that counts as the junction's black. */
#define BARCODE_JUNCTION_HITS 2U           /* Consecutive samples above it (20 ms) before it is reported. */
#define BARCODE_CAP_BEFORE   32U           /* Junction capture: samples kept from before the trigger ... */
#define BARCODE_CAP_AFTER    100U          /* ... and after it (1 s), printed once as the analogue record. */

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
#define BARCODE_FAIL_MIN_DRIVE 1000U /* A failed pass counts only if the car drove 10 cm (0.1 mm units) during it:
                                       * at 0.05 m/s the sample-count rule alone lets a ~4 cm dark patch (tape edge,
                                       * hump) qualify; a real small-code pass drives 13 cm plus its 25 mm flat tail. */
/* Junction window: the junction's black and whatever the barcode sensor crosses while the car leaves it
 * (50 cm at 0.05 m/s) are not failed reads. Kept for the route action that will follow the junction. */
#define BARCODE_JUNCTION_AFTER  1000U /* ... up to 10 s after it, is the junction's own bar: no failure. */
#if ROBO_LINE_TASK
static bool bc_armed;           /* A valid read was made: the next black is the junction. */
static uint32_t bc_arm_rise;    /* Rise over the quiet level that counts as that black. */
#endif

static bool bc_dirty; /* The last publish failed (bus busy): retried every sample until it goes through. */

/* True from "reading..." to the end of that stretch. The line task (subsystems.h) steers more gently and
 * bridges short losses while it is set, and tells a tape crossing on a bend apart by the car's heading.
 * Neither brightness nor dark width can: with the sensor low the small code reads +3000 like tape, and its
 * bars blur into dark runs of 22-38 mm (2026-11-10 course runs), as long as a tape crossing (23-27 mm).
 * A bool store is atomic (SMP is off). */
static volatile bool bc_code_pass;
bool barcode_code_pass(void) { return bc_code_pass; }
/* A failed pass in which the car turned this far (wrapping milliradians) was a bend, not a code:
 * the codes lie on straight line. Logged: 713 and 1248 mrad on bends, under 100 over the codes. */
#define BARCODE_FAIL_MAX_TURN 350

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
    chars = barcode_decode_stretch(samples, path, count, work, durations, &n, text);
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
         * bar and what follows it are not failed reads. */
        int turned = 0;
        for (unsigned i = BARCODE_QUIET; i < count; ++i) {
            int d = (int)(int16_t)(uint16_t)(heading[i] - heading[BARCODE_QUIET]);
            if (d < 0) { d = -d; }
            if (d > turned) { turned = d; }
        }
        if (path[count - 1U] - path[BARCODE_QUIET] < BARCODE_FAIL_MIN_DRIVE) {
            /* Placing the car on the track, or a spin on the spot, can move the reading without a read. */
            tm_printf((const UB *)"barcode: no read while not driving, not counted\n");
        } else if (turned > BARCODE_FAIL_MAX_TURN) {
            tm_printf((const UB *)"barcode: no read while turning (%d mrad), not counted\n", turned);
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
        for (unsigned i = 0U; i < n; ++i) {
            barcode_dump_num("bw", i, n, (int32_t)(durations[i] / BARCODE_WIDTH_PER_TENTH_MM));
        }
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
 * sample (0.1 mm units), to see how the car's wiggle and speed line up with the sensor.
 * tests/test_barcode_decode.c replays exactly this format. */
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
    static barcode_seg_t seg; /* The stretch being collected, with its lead-in ring. */
    unsigned reads = 0U;
    uint64_t next;
#if ROBO_LINE_TASK
    uint16_t hist[BARCODE_CAP_BEFORE]; /* Ring of the last samples, for the junction record. */
    unsigned hist_head = 0U, hits = 0U;
    for (unsigned i = 0U; i < BARCODE_CAP_BEFORE; ++i) { hist[i] = 0U; }
#endif
    (void)code; (void)context;
    barcode_seg_init(&seg);
    barcode_adc_init();
    tm_printf((const UB *)"barcode: reader running on GP28 (distance widths, %u.%u mm steps)\n",
              BARCODE_PATH_STEP / 10U, BARCODE_PATH_STEP % 10U);
    next = time_us_64() + BARCODE_SAMPLE_US;
    for (;;) {
        uint16_t s = barcode_sample(&next);
        uint32_t at = barcode_path();
        barcode_seg_event_t event;
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
         * is the junction. Reported at once; the line task decides what to do about it. The quiet
         * level is the one from the last sample taken while no stretch was running. */
        if (bc_armed && seg.level_ok && s > seg.level + bc_arm_rise) {
            if (++hits >= BARCODE_JUNCTION_HITS) {
                hits = 0U;
                bc_armed = false;
                bc_junction_sample = bc_sample_no;
                bc_junction_seen = true;
                ++bc_junctions;
                barcode_publish();
                tm_printf((const UB *)"barcode: junction black=%u white=%u threshold=%u path=%u\n",
                          (unsigned)s, (unsigned)seg.level, (unsigned)(seg.level + bc_arm_rise), (unsigned)at);
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
        event = barcode_seg_feed(&seg, s, at, bc_heading);
        if (event == BARCODE_SEG_IDLE) {
            barcode_idle(s);
        } else if (event == BARCODE_SEG_STARTED) {
            bc_stretch_start = bc_sample_no;
            bc_code_pass = true;
            tm_printf((const UB *)"barcode: reading...\n");
        } else if (event == BARCODE_SEG_DONE) {
            bc_code_pass = false;
            barcode_finish(seg.samples, seg.path, seg.heading, seg.count, &reads);
        }
    }
}
#endif
