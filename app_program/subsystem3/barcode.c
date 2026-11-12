#include "subsystems.h"

/* The barcode reader runs in the line-steering image and in the barcode test image
 * (production profile built with -DCMAKE_C_FLAGS=-DROBO_BARCODE_TEST=1). Every other image
 * has no barcode_task.
 *
 * Reader modes (what the next pass or black means), printed as [S3-BARCODE] A -> B:
 *   WATCHING     normal: a full pass is decoded against the route's letters (route.h).
 *   CALIBRATING  after GP21: the next usable full pass is the calibration code; it sets the sensitivity.
 *   ARMED        after a valid read (line images): the next black is the junction.
 *   IGNORING     after a U-turn: the car drives back over the code it just read, until a set path.
 * Separately, a pass is being captured ("reading") from the segmenter's start to its end. */
#if ROBO_BARCODE_TASK
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#include "board_config.h"
#include "motion_hal.h"
#include "robot.h"
#include "barcode_decode.h"
#include "route.h"
#include "ir_io.h"

/* Samples the barcode sensor's analog output (AO on GP28 = ADC2) all the time. barcode_decode.c
 * keeps a stretch where the reading leaves its quiet level (with 0.5 s of the quiet level before
 * it) and, once the reading is flat again, splits it into bars and spaces and decodes it. */
#define BARCODE_ADC_AVERAGE  32U           /* Conversions averaged per ADC read. */
#define BARCODE_JUNCTION_MIN 60U           /* Smallest rise (counts) that counts as the junction's black. */
#define BARCODE_JUNCTION_HITS 2U           /* Consecutive samples above it (20 ms) before it is reported. */
#define BARCODE_JUNCTION_BEFORE 300U /* Junction window (samples): a stretch starting this far before the trigger, or ... */
#define BARCODE_JUNCTION_AFTER  1000U /* ... up to 10 s after it, is the junction's own bar (or what the sensor crosses
                                       * while the car leaves it, 50 cm at 0.05 m/s): not a failed read. */
#define BARCODE_FAIL_MIN_DRIVE 1000U /* A failed pass counts only if the car drove 10 cm (0.1 mm units) during it:
                                       * at 0.05 m/s the sample-count rule alone lets a ~4 cm dark patch (tape edge,
                                       * hump) qualify; a real small-code pass drives 13 cm plus its 25 mm flat tail. */
/* A failed pass in which the car turned this far (wrapping milliradians) was a bend, not a code:
 * the codes lie on straight line. Logged: 713 and 1248 mrad on bends, under 100 over the codes. */
#define BARCODE_FAIL_MAX_TURN 350
/* Analysis builds (-DROBO_BARCODE_JUNCTION_DUMP=1): print the analogue record of each junction's black. */
#if ROBO_LINE_TASK && defined(ROBO_BARCODE_JUNCTION_DUMP) && ROBO_BARCODE_JUNCTION_DUMP
#define BARCODE_JUNCTION_RECORD 1
#define BARCODE_CAP_BEFORE   32U           /* Samples kept from before the trigger ... */
#define BARCODE_CAP_AFTER    100U          /* ... and after it (1 s). */
#else
#define BARCODE_JUNCTION_RECORD 0
#endif

/* ---- Reader mode (owned by the barcode task) ---- */
typedef enum { BC_WATCHING, BC_CALIBRATING, BC_ARMED, BC_IGNORING, BC_MODE_COUNT } barcode_mode_t;
static barcode_mode_t bc_mode = BC_WATCHING;

static void barcode_set_mode(barcode_mode_t next)
{
    static const char *const names[] = {"WATCHING", "CALIBRATING", "ARMED", "IGNORING"};
    _Static_assert(sizeof names / sizeof names[0] == BC_MODE_COUNT, "one name per reader mode");
    if (next != bc_mode) {
        tm_printf((const UB *)"[S3-BARCODE] %s -> %s\n", (const UB *)names[bc_mode], (const UB *)names[next]);
        bc_mode = next;
    }
}

/* Only the route's letters are read (best fit among them, barcode_decode.h). */
static const char bc_letters[] = { ROUTE_TABLE(ROUTE_LETTER_OF) '\0' };
/* Sensitivity (contrast divisor), tried first on every pass. The sensor's height over the floor changes
 * whenever the robot is handled, so GP21 calibrates it on a known code (RAM only: lost at power-off on
 * purpose). A pass that does not decode is retried down BARCODE_LADDER (barcode_decode.h) before it fails. */
static unsigned bc_div = BARCODE_CONTRAST_DIV;

/* ---- Requests between the line task and this task. Every variable below has exactly one writer
 * task, and each is a single aligned 32-bit (or smaller) store, so no lock is needed (SMP is off):
 * the line task bumps a request counter, this task applies it on its next sample (within 10 ms, well
 * before the line task's brake ends) and answers with its own counter. ---- */
static volatile uint32_t bc_cal_requests;      /* Written by the line task: calibrations asked for. */
static volatile uint32_t bc_ignore_requests;   /* Written by the line task: U-turns announced ... */
static volatile uint32_t bc_ignore_distance;   /* ... and the path (0.1 mm) to skip after each (written first). */
static volatile uint32_t bc_cal_results;       /* Written by this task: calibrations finished ... */
static volatile bool bc_cal_ok;                /* ... and whether the last one read the code (written first). */
static uint32_t bc_cal_results_seen;           /* Written by the line task (barcode_calibration_result). */

void barcode_calibrate_begin(void) { ++bc_cal_requests; }

void barcode_ignore_for(double metres)
{
    bc_ignore_distance = (uint32_t)(metres * 10000.0);
    ++bc_ignore_requests;
}

barcode_cal_t barcode_calibration_result(void)
{
    if (bc_cal_results == bc_cal_results_seen) { return BARCODE_CAL_NONE; }
    bc_cal_results_seen = bc_cal_results;
    return bc_cal_ok ? BARCODE_CAL_DONE : BARCODE_CAL_FAILED;
}

static uint32_t bc_path_now;      /* Latest path (0.1 mm), see barcode_path(). */
static uint32_t bc_ignore_until;  /* Passes starting before this path (0.1 mm) are skipped. */

/* Applies the line task's requests (this task only). A U-turn also disarms the junction check. */
static void barcode_take_requests(void)
{
    static uint32_t cal_seen, ignore_seen;
    if (bc_cal_requests != cal_seen) {
        cal_seen = bc_cal_requests;
        barcode_set_mode(BC_CALIBRATING);
    }
    if (bc_ignore_requests != ignore_seen) {
        ignore_seen = bc_ignore_requests;
        bc_ignore_until = bc_path_now + bc_ignore_distance;
        barcode_set_mode(BC_IGNORING);
    }
}

/* True from "reading..." to the end of that stretch. The line task (subsystems.h) steers more gently and
 * bridges short losses while it is set, and tells a tape crossing on a bend apart by the car's heading.
 * Neither brightness nor dark width can: with the sensor low the small code reads +3000 like tape, and its
 * bars blur into dark runs of 22-38 mm (2026-11-10 course runs), as long as a tape crossing (23-27 mm).
 * A bool store is atomic (SMP is off). */
static volatile bool bc_code_pass;
bool barcode_code_pass(void) { return bc_code_pass; }

/* ---- Sampling ---- */
/* Each sample averages an ADC read every 1 ms tick across its whole period, so light flicker
 * (the white-level spread that a 64 us burst does not remove) cancels out. */
static uint16_t barcode_sample(uint64_t *next)
{
    uint32_t sum = 0U, reads = 0U;
    if (time_us_64() > *next + 100000U) { *next = time_us_64(); } /* Fell behind: do not burst. */
    do {
        sum += ir_adc_read(BOARD_BARCODE, BARCODE_ADC_AVERAGE);
        ++reads;
        (void)tk_dly_tsk(1U);
    } while (time_us_64() < *next);
    *next += BARCODE_SAMPLE_US;
    return (uint16_t)(sum / reads);
}

static uint16_t bc_heading; /* Heading (wrapping milliradians) when the status was last read, see barcode_path(). */

/* Distance the car has driven, in 0.1 mm. Holds the last value if the status is unavailable, so
 * a stretch then looks like the car did not move and is decoded by time. */
static uint32_t barcode_path(void)
{
    robot_status_t status;
    if (motion_get_status(&status)) {
        if (status.motion.path >= 0.0) { bc_path_now = (uint32_t)(status.motion.path * 10000.0); }
        bc_heading = (uint16_t)(uint32_t)(int32_t)(status.motion.heading * 1000.0); /* Wraps; only differences are used. */
    }
    return bc_path_now;
}

/* ---- Published status (see robot.h): letter, valid read counter, junction counter, failed reads ---- */
static unsigned bc_reads;
static double bc_letter;
static unsigned bc_junctions;
static unsigned bc_failures;       /* Long passes that did not decode (line images stop the car on this). */
static bool bc_dirty;              /* The last publish failed (bus busy): retried every sample until it goes through. */
static uint32_t bc_sample_no;      /* Counts samples, to place stretch starts and the junction in time. */
static uint32_t bc_stretch_start;  /* Sample number at which the current stretch began. */
static uint32_t bc_junction_sample;/* Sample number of the last junction trigger. */
static bool bc_junction_seen;
#if ROBO_LINE_TASK
static uint32_t bc_arm_rise;       /* ARMED: rise over the quiet level that counts as the junction's black. */
#endif

static void barcode_publish(void)
{
    subsystem_status_t status = {.available = true, .timestamp = motion_hal_time(),
        .values = {bc_letter, (double)bc_reads, (double)bc_junctions, (double)bc_failures}};
    bc_dirty = !robot_update_subsystem(SOURCE_BARCODE, &status);
}

/* ---- End of a pass ---- */
/* One number of a prefixed dump line (20 per line); the pause lets the USB buffer drain. */
static void barcode_dump_num(const char *tag, unsigned i, unsigned n, int32_t value)
{
    if ((i % 20U) == 0U) { tm_printf((const UB *)"%s", (const UB *)tag); }
    tm_printf((const UB *)" %d%s", (int)value, ((i % 20U) == 19U || i + 1U == n) ? "\n" : "");
    if ((i % 20U) == 19U) { (void)tk_dly_tsk(20U); }
}

/* Scratch for decoding (the decoder resamples and smooths in place). */
static uint16_t bc_work[BARCODE_SAMPLES];
static uint32_t bc_widths[BARCODE_MAX_ELEMENTS];

/* Decodes a pass at 'first', then (a full pass only) down the sensitivity ladder until one reads.
 * *used = the setting of the last attempt. Returns the character count, 0 if every setting failed. */
static unsigned barcode_decode_any(const uint16_t *samples, const uint32_t *path, unsigned count, unsigned *n,
                                   char *text, unsigned first, const char *letters, unsigned *used)
{
    unsigned chars = 0U, div;
    for (unsigned i = 0U; chars == 0U && (div = barcode_ladder_div(first, i)) != 0U; ++i) {
        if (i > 0U) {
            if (count < BARCODE_FULL_PASS) { break; } /* A fragment (bump, tape edge): one try is enough. */
            (void)tk_dly_tsk(20U); /* Same priority as the line task: let it refresh its steering. */
        }
        chars = barcode_decode_stretch(samples, path, count, bc_work, bc_widths, n, text, div, letters);
        *used = div;
    }
    return chars;
}

/* Why a pass cannot be judged: the car hardly drove (placed on the track, a spin on the spot), or it
 * turned (a bend: the codes lie on straight line). *turned = the largest turn (wrapping mrad). */
typedef enum { BC_PASS_OK, BC_PASS_NOT_DRIVING, BC_PASS_TURNING } barcode_pass_t;

static barcode_pass_t barcode_pass_check(const uint32_t *path, const uint16_t *heading, unsigned count, int *turned)
{
    *turned = 0;
    for (unsigned i = BARCODE_QUIET; i < count; ++i) {
        int d = (int)(int16_t)(uint16_t)(heading[i] - heading[BARCODE_QUIET]);
        if (d < 0) { d = -d; }
        if (d > *turned) { *turned = d; }
    }
    if (path[count - 1U] - path[BARCODE_QUIET] < BARCODE_FAIL_MIN_DRIVE) { return BC_PASS_NOT_DRIVING; }
    return (*turned > BARCODE_FAIL_MAX_TURN) ? BC_PASS_TURNING : BC_PASS_OK;
}

/* CALIBRATING: the first usable full pass is the calibration code. It must read as that letter at some
 * setting of the ladder (default first); that setting is kept. Never counted as a read, a failure or a
 * junction. An unusable pass (not driving, or turning) leaves the reader waiting for the next one. */
static void barcode_calibrate_pass(const uint16_t *samples, const uint32_t *path, const uint16_t *heading,
                                   unsigned count)
{
    static const char expected[] = {ROUTE_CALIBRATION_LETTER, '\0'};
    char text[BARCODE_MAX_CHARS + 1U];
    unsigned n, used = BARCODE_CONTRAST_DIV;
    int turned;
    barcode_pass_t check = barcode_pass_check(path, heading, count, &turned);
    bool ok;
    if (check != BC_PASS_OK) {
        tm_printf((const UB *)"calibration: pass not usable (%s), still waiting for *%c*\n",
                  (check == BC_PASS_NOT_DRIVING) ? "car not driving" : "car turning", ROUTE_CALIBRATION_LETTER);
        return;
    }
    ok = barcode_decode_any(samples, path, count, &n, text, BARCODE_CONTRAST_DIV, expected, &used) != 0U;
    if (ok) { bc_div = used; }
    /* Published once, after every setting was tried: the line task acts on the first result it sees. */
    bc_cal_ok = ok;
    ++bc_cal_results;
    if (ok) {
        tm_printf((const UB *)"barcode: calibration done, read *%c* at sensitivity div=%u\n", ROUTE_CALIBRATION_LETTER, bc_div);
    } else {
        tm_printf((const UB *)"barcode: calibration FAILED: no *%c* at any sensitivity (elements=%u), keeping div=%u"
                  " - check the sensor\n", ROUTE_CALIBRATION_LETTER, n, bc_div);
    }
    barcode_set_mode(BC_WATCHING);
}

/* A full pass that did not decode: counted as a failure (line images stop the car) unless the car
 * hardly drove, was turning (a bend), or was at the junction just armed. Reported before the slow dump
 * below, so the car stops as early as it can. */
static void barcode_count_failure(const uint32_t *path, const uint16_t *heading, unsigned count)
{
#if ROBO_LINE_TASK
    int turned;
    barcode_pass_t check = barcode_pass_check(path, heading, count, &turned);
    if (check == BC_PASS_NOT_DRIVING) {
        tm_printf((const UB *)"barcode: no read while not driving, not counted\n");
    } else if (check == BC_PASS_TURNING) {
        tm_printf((const UB *)"barcode: no read while turning (%d mrad), not counted\n", turned);
    } else if (!(bc_junction_seen && bc_stretch_start + BARCODE_JUNCTION_BEFORE >= bc_junction_sample &&
                 bc_stretch_start <= bc_junction_sample + BARCODE_JUNCTION_AFTER)) {
        ++bc_failures;
        barcode_publish();
    }
#else
    (void)path; (void)heading; (void)count;
#endif
}

/* bv = readings, bp = distance driven in mm since the first sample, bh = heading change in
 * milliradians (the sensor sits to one side of the wheels, so steering changes how far it
 * travels), bw = the decoded element widths in 0.1 mm. tools/barcode_log.py reads these. */
static void barcode_dump(const uint16_t *samples, const uint32_t *path, const uint16_t *heading,
                         unsigned count, unsigned n)
{
#ifndef ROBO_BARCODE_STREAM /* The stream already printed every sample; the paced dump would blind the reader. */
    for (unsigned i = 0U; i < count; ++i) { barcode_dump_num("bv", i, count, (int32_t)samples[i]); }
    for (unsigned i = 0U; i < count; ++i) { barcode_dump_num("bp", i, count, (int32_t)((path[i] - path[0]) / 10U)); }
    for (unsigned i = 0U; i < count; ++i) {
        barcode_dump_num("bh", i, count, (int32_t)(int16_t)(uint16_t)(heading[i] - heading[0]));
    }
    for (unsigned i = 0U; i < n; ++i) {
        barcode_dump_num("bw", i, n, (int32_t)(bc_widths[i] / BARCODE_WIDTH_PER_TENTH_MM));
    }
#else
    (void)samples; (void)path; (void)heading; (void)count; (void)n;
#endif
}

static void barcode_finish(const uint16_t *samples, const uint32_t *path, const uint16_t *heading, unsigned count)
{
    char text[BARCODE_MAX_CHARS + 1U];
    unsigned n, chars, used = bc_div;
    uint32_t noise, white;
    uint16_t low, high;
    /* Decided by where the pass started, so a pass straddling the end of the window is still skipped. */
    if (path[BARCODE_QUIET] < bc_ignore_until) {
        tm_printf((const UB *)"barcode: pass ignored (driving back after a U-turn)\n");
        return;
    }
    if (bc_mode == BC_CALIBRATING) {
        if (count >= BARCODE_FULL_PASS) { barcode_calibrate_pass(samples, path, heading, count); } /* Else a fragment. */
        return;
    }
    chars = barcode_decode_any(samples, path, count, &n, text, bc_div, bc_letters, &used);
    white = barcode_level(samples, BARCODE_QUIET, &noise);
    barcode_range(samples, count, &low, &high);
    if (chars != 0U) {
        text[chars - 1U] = '\0'; /* Drop the closing '*'. */
        ++bc_reads;
        if (used == bc_div) {
            tm_printf((const UB *)"barcode: *%s*\n", (const UB *)&text[1]);
        } else { /* Read only at a fallback setting of the ladder (calibration still owns bc_div). */
            tm_printf((const UB *)"barcode: *%s* (div=%u)\n", (const UB *)&text[1], used);
        }
        bc_letter = (double)(unsigned char)text[1];
        barcode_publish();
#if ROBO_LINE_TASK
        /* The next black after this read is the junction: half the bar rise just seen counts. */
        bc_arm_rise = (high > white) ? (high - white) / 2U : 0U;
        if (bc_arm_rise < BARCODE_JUNCTION_MIN) { bc_arm_rise = BARCODE_JUNCTION_MIN; }
        tm_printf((const UB *)"barcode: armed, white=%u peak=%u junction=white+%u\n",
                  (unsigned)white, (unsigned)high, (unsigned)bc_arm_rise);
        barcode_set_mode(BC_ARMED);
#endif
    } else if (count >= BARCODE_FULL_PASS) {
        /* Every setting failed: this is the "really failed" exit (line images stop the car). */
        tm_printf((const UB *)"barcode: no read at any sensitivity (div=%u,%u,%u,%u), elements=%u samples=%u"
                  " white=%u min=%u max=%u\n", barcode_ladder_div(bc_div, 0U), barcode_ladder_div(bc_div, 1U),
                  barcode_ladder_div(bc_div, 2U), barcode_ladder_div(bc_div, 3U),
                  n, count, (unsigned)white, (unsigned)low, (unsigned)high);
        barcode_count_failure(path, heading, count);
        barcode_dump(samples, path, heading, count, n);
    } else {
        tm_printf((const UB *)"barcode: stretch ended, no code (samples=%u elements=%u white=%u min=%u max=%u)\n",
                  count, n, (unsigned)white, (unsigned)low, (unsigned)high);
    }
}

/* ---- Analysis output ---- */
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

#if BARCODE_JUNCTION_RECORD
/* The analogue record of the junction's black: BARCODE_CAP_BEFORE samples up to the trigger and
 * BARCODE_CAP_AFTER after it, printed once when complete (20 values per line, 10 ms each). */
static uint16_t cap_hist[BARCODE_CAP_BEFORE]; /* Ring of the last samples. */
static unsigned cap_head;
static uint16_t cap[BARCODE_CAP_BEFORE + BARCODE_CAP_AFTER];
static unsigned cap_n, cap_target;

static void barcode_record_sample(uint16_t s)
{
    cap_hist[cap_head] = s;
    cap_head = (cap_head + 1U) % BARCODE_CAP_BEFORE;
    if (cap_n < cap_target) { /* Samples after the trigger. */
        cap[cap_n++] = s;
        if (cap_n == cap_target) {
            tm_printf((const UB *)"barcode: junction samples, %u before the trigger + %u after, 10 ms each\n",
                      BARCODE_CAP_BEFORE, BARCODE_CAP_AFTER);
            for (unsigned i = 0U; i < cap_n; ++i) {
                tm_printf((const UB *)"%u%s", (unsigned)cap[i], ((i % 20U) == 19U || i + 1U == cap_n) ? "\n" : " ");
                if ((i % 20U) == 19U) { (void)tk_dly_tsk(20U); } /* Let the USB buffer drain. */
            }
            cap_target = 0U;
        }
    }
}

static void barcode_record_junction(void)
{
    for (unsigned i = 0U; i < BARCODE_CAP_BEFORE; ++i) {
        cap[i] = cap_hist[(cap_head + i) % BARCODE_CAP_BEFORE]; /* Oldest first, trigger last. */
    }
    cap_n = BARCODE_CAP_BEFORE;
    cap_target = BARCODE_CAP_BEFORE + BARCODE_CAP_AFTER;
}
#else
#define barcode_record_sample(s) ((void)(s))
#define barcode_record_junction() ((void)0)
#endif

#if ROBO_LINE_TASK
/* ARMED: the first black after a valid read (clearly above the quiet level for two samples in a row)
 * is the junction. Reported at once; the line task decides what to do about it. The quiet level is
 * the one from the last sample taken while no stretch was running. */
static void barcode_check_junction(const barcode_seg_t *seg, uint16_t s, uint32_t at)
{
    static unsigned hits;
    if (bc_mode != BC_ARMED || !seg->level_ok || s <= seg->level + bc_arm_rise) { hits = 0U; return; }
    if (++hits < BARCODE_JUNCTION_HITS) { return; }
    hits = 0U;
    barcode_set_mode(BC_WATCHING);
    bc_junction_sample = bc_sample_no;
    bc_junction_seen = true;
    ++bc_junctions;
    barcode_publish();
    tm_printf((const UB *)"barcode: junction black=%u white=%u threshold=%u path=%u\n",
              (unsigned)s, (unsigned)seg->level, (unsigned)(seg->level + bc_arm_rise), (unsigned)at);
    barcode_record_junction();
}
#else
#define barcode_check_junction(seg, s, at) ((void)(seg), (void)(s), (void)(at))
#endif

void barcode_task(INT code, void *context)
{
    static barcode_seg_t seg; /* The stretch being collected, with its lead-in ring. */
    uint64_t next;
    (void)code; (void)context;
    barcode_seg_init(&seg);
    if (!ir_adc_init(BOARD_BARCODE)) {
        /* No reader: the line task never sees a read or a failure, so it just follows the line. */
        tm_printf((const UB *)"barcode: ADC not responding, reader off\n");
        for (;;) { (void)tk_dly_tsk(1000U); } /* A task must never return (pc=0 hard fault). */
    }
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
        barcode_take_requests();
        if (bc_mode == BC_IGNORING && at >= bc_ignore_until) { barcode_set_mode(BC_WATCHING); }
        barcode_record_sample(s);
        barcode_check_junction(&seg, s, at);
        event = barcode_seg_feed(&seg, s, at, bc_heading);
        if (event == BARCODE_SEG_IDLE) {
            barcode_idle(s);
        } else if (event == BARCODE_SEG_STARTED) {
            bc_stretch_start = bc_sample_no;
            bc_code_pass = true;
            tm_printf((const UB *)"barcode: reading...\n");
        } else if (event == BARCODE_SEG_DONE) {
            bc_code_pass = false;
            barcode_finish(seg.samples, seg.path, seg.heading, seg.count);
        }
    }
}
#endif
