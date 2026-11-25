#include "../../include/float_types.h"
#include <stdint.h>
#include "subsystems.h"

/* The barcode reader runs in the line-steering image and in the barcode test
 * image (production profile built with -DCMAKE_C_FLAGS=-DROBO_BARCODE_TEST=1).
 * Every other image has no barcode_task.
 *
 * Reader modes (what the next pass or black means), printed as [S3-BARCODE] A
 * -> B: WATCHING     normal: a full pass is decoded against the route's letters
 * (route.h). CALIBRATING  after GP21: the next usable full pass is the
 * calibration code; it sets the sensitivity. ARMED        after a valid read
 * (line images): the next black is the junction. IGNORING     after a U-turn:
 * the car drives back over the code it just read, until a set path. Separately,
 * a pass is being captured ("reading") from the segmenter's start to its end.
 */
#if ROBO_BARCODE_TASK
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#include "board_config.h"
#include "motion_hal.h"
#include "robot.h"
#include "barcode_decode.h"
#include "route.h"
#include "ir_io.h"

/* Samples the barcode sensor's analog output (AO on GP28 = ADC2) all the time.
 * barcode_decode.c keeps a stretch where the reading leaves its quiet level
 * (with 0.5 s of the quiet level before it) and, once the reading is flat
 * again, splits it into bars and spaces and decodes it. */
/* Conversions averaged per ADC read. */
#define BARCODE_ADC_AVERAGE 32U
/* Smallest rise (counts) that counts as the junction's black. */
#define BARCODE_JUNCTION_MIN 60U
/* Consecutive samples above it (20 ms) before it is reported. */
#define BARCODE_JUNCTION_HITS 2U
/* Junction window (samples): a stretch starting this far before the trigger, or
 * ... */
#define BARCODE_JUNCTION_BEFORE 300U
/* ... up to 10 s after it, is the junction's own bar (or what the sensor
 * crosses while the car leaves it, 50 cm at 0.05 m/s): not a failed read. */
#define BARCODE_JUNCTION_AFTER 1000U
/* A failed pass counts only if the car drove 10 cm (0.1 mm units) during it: at
 * 0.05 m/s the sample-count rule alone lets a ~4 cm dark patch (tape edge,
 * hump) qualify; a real small-code pass drives 13 cm plus its 25 mm flat tail.
 */
#define BARCODE_FAIL_MIN_DRIVE 1000U
/* A failed pass in which the car turned this far (wrapping milliradians) was a
 * bend, not a code: the codes lie on straight line. Logged: 713 and 1248 mrad
 * on bends, under 100 over the codes. */
#define BARCODE_FAIL_MAX_TURN 350
/* Analysis builds (-DROBO_BARCODE_JUNCTION_DUMP=1): print the analogue record
 * of each junction's black. */
#if ROBO_LINE_TASK && defined(ROBO_BARCODE_JUNCTION_DUMP) \
    && ROBO_BARCODE_JUNCTION_DUMP
#define BARCODE_JUNCTION_RECORD 1
/* Samples kept from before the trigger ... */
#define BARCODE_CAP_BEFORE 32U
#define BARCODE_CAP_AFTER  100U /* ... and after it (1 s). */
#else
#define BARCODE_JUNCTION_RECORD 0
#endif

/* ---- Reader mode (owned by the barcode task) ---- */
typedef enum
{
    BC_WATCHING,
    BC_CALIBRATING,
    BC_ARMED,
    BC_IGNORING,
    BC_MODE_COUNT
} barcode_mode_t;

static barcode_mode_t g_bc_mode = BC_WATCHING;

static void
barcode_set_mode (barcode_mode_t next)
{
    static const char * const names[]
        = {"WATCHING", "CALIBRATING", "ARMED", "IGNORING"};

    _Static_assert(BC_MODE_COUNT == (sizeof names / sizeof names[0]),
                   "one name per reader mode");

    if (next != g_bc_mode)
    {
        tm_printf((const UB *)"[S3-BARCODE] %s -> %s\n",
                  (const UB *)names[g_bc_mode], (const UB *)names[next]);
        g_bc_mode = next;
    }
}

/* Sensitivity (contrast divisor), tried first on every pass. The sensor's
 * height over the floor changes whenever the robot is handled, so GP21
 * calibrates it on a known code (RAM only: lost at power-off on purpose). A
 * pass that does not decode is retried down BARCODE_LADDER (barcode_decode.h)
 * before it fails. */
static unsigned g_bc_div = BARCODE_CONTRAST_DIV;

/* ---- Requests between the line task and this task. Every variable below has
 * exactly one writer task, and each is a single aligned 32-bit (or smaller)
 * store, so no lock is needed (SMP is off): the line task bumps a request
 * counter, this task applies it on its next sample (within 10 ms, well before
 * the line task's brake ends) and answers with its own counter. ---- */
/* Written by the line task: calibrations asked for. */
static volatile uint32_t g_bc_cal_requests;
/* Written by the line task: U-turns announced ... */
static volatile uint32_t g_bc_ignore_requests;
/* ... and the path (0.1 mm) to skip after each (written first). */
static volatile uint32_t g_bc_ignore_distance;
/* Written by this task: calibrations finished ... */
static volatile uint32_t g_bc_cal_results;
/* ... and whether the last one read the code (written first). */
static volatile bool gb_bc_cal_ok;
/* Written by the line task (barcode_calibration_result). */
static uint32_t g_bc_cal_results_seen;

void
barcode_calibrate_begin (void)
{
    ++g_bc_cal_requests;
}

void
barcode_ignore_for (float64_t metres)
{
    g_bc_ignore_distance = (uint32_t)(metres * 10000.0);
    ++g_bc_ignore_requests;
}

barcode_cal_t
barcode_calibration_result (void)
{
    if (g_bc_cal_results == g_bc_cal_results_seen)
    {
        return BARCODE_CAL_NONE;
    }

    g_bc_cal_results_seen = g_bc_cal_results;
    return gb_bc_cal_ok ? BARCODE_CAL_DONE : BARCODE_CAL_FAILED;
}

static uint32_t g_bc_path_now; /* Latest path (0.1 mm), see barcode_path(). */
/* Passes starting before this path (0.1 mm) are skipped. */
static uint32_t g_bc_ignore_until;

/* Applies the line task's requests (this task only). A U-turn also disarms the
 * junction check. */
static void
barcode_take_requests (void)
{
    static uint32_t cal_seen;
    static uint32_t ignore_seen;

    if (g_bc_cal_requests != cal_seen)
    {
        cal_seen = g_bc_cal_requests;
        barcode_set_mode(BC_CALIBRATING);
    }

    if (g_bc_ignore_requests != ignore_seen)
    {
        ignore_seen       = g_bc_ignore_requests;
        g_bc_ignore_until = g_bc_path_now + g_bc_ignore_distance;
        barcode_set_mode(BC_IGNORING);
    }
}

/* True from "reading..." to the end of that stretch. The line task
 * (subsystems.h) steers more gently and bridges short losses while it is set,
 * and tells a tape crossing on a bend apart by the car's heading. Neither
 * brightness nor dark width can: with the sensor low the small code reads +3000
 * like tape, and its bars blur into dark runs of 22-38 mm (2026-11-10 course
 * runs), as long as a tape crossing (23-27 mm). A bool store is atomic (SMP is
 * off). */
static volatile bool gb_bc_code_pass;
bool
barcode_code_pass (void)
{
    return gb_bc_code_pass;
}

/* ---- Sampling ---- */
/* Each sample averages an ADC read every 1 ms tick across its whole period, so
 * light flicker (the white-level spread that a 64 us burst does not remove)
 * cancels out. */
static uint16_t
barcode_sample (uint64_t * p_next)
{
    uint32_t sum   = 0U;
    uint32_t reads = 0U;

    if (time_us_64() > (*p_next + 100000U))
    {
        *p_next = time_us_64();
    } /* Fell behind: do not burst. */

    do
    {
        sum += ir_adc_read(BOARD_BARCODE, BARCODE_ADC_AVERAGE);
        ++reads;
        (void)tk_dly_tsk(1U);
    } while (time_us_64() < *p_next);
    *p_next += BARCODE_SAMPLE_US;
    return (uint16_t)(sum / reads);
}

/* Heading (wrapping milliradians) when the status was last read, see
 * barcode_path(). */
static uint16_t g_bc_heading;

/* Distance the car has driven, in 0.1 mm. Holds the last value if the status is
 * unavailable, so a stretch then looks like the car did not move and is decoded
 * by time. */
static uint32_t
barcode_path (void)
{
    robot_status_t status;

    if (motion_get_status(&status))
    {
        if (status.motion.path >= 0.0)
        {
            g_bc_path_now = (uint32_t)(status.motion.path * 10000.0);
        }

        /* Wraps; only differences are used. */
        g_bc_heading
            = (uint16_t)(uint32_t)(int32_t)(status.motion.heading * 1000.0);
    }

    return g_bc_path_now;
}

/* ---- Published status (see robot.h): letter, valid read counter, junction
 * counter, failed reads ---- */
static unsigned  g_bc_reads;
static float64_t g_bc_letter;
static unsigned  g_bc_junctions;
/* Long passes that did not decode (line images stop the car on this). */
static unsigned g_bc_failures;
/* The last publish failed (bus busy): retried every sample until it goes
 * through. */
static bool gb_bc_dirty;
/* Counts samples, to place stretch starts and the junction in time. */
static uint32_t g_bc_sample_no;
/* Sample number at which the current stretch began. */
static uint32_t g_bc_stretch_start;
/* Sample number of the last junction trigger. */
static uint32_t g_bc_junction_sample;
static bool     gb_bc_junction_seen;
#if ROBO_LINE_TASK
/* ARMED: rise over the quiet level that counts as the junction's black. */
static uint32_t g_bc_arm_rise;
#endif

static void
barcode_publish (void)
{
    subsystem_status_t status
        = {.available = true,
           .timestamp = motion_hal_time(),
           .values    = {g_bc_letter, (float64_t)g_bc_reads,
                         (float64_t)g_bc_junctions, (float64_t)g_bc_failures}};

    gb_bc_dirty = !robot_update_subsystem(SOURCE_BARCODE, &status);
}

/* ---- End of a pass ---- */
/* One number of a prefixed dump line (20 per line); the pause lets the USB
 * buffer drain. */
static void
barcode_dump_num (const char * p_tag, unsigned index, unsigned count,
                  int32_t value)
{
    if (0U == (index % 20U))
    {
        tm_printf((const UB *)"%s", (const UB *)p_tag);
    }

    tm_printf((const UB *)" %d%s", (int)value,
              ((19U == (index % 20U)) || ((index + 1U) == count)) ? "\n" : "");

    if (19U == (index % 20U))
    {
        (void)tk_dly_tsk(20U);
    }
}

/* Scratch for decoding (the decoder resamples and smooths in place). */
static uint16_t g_bc_work[BARCODE_SAMPLES];
static uint32_t g_bc_widths[BARCODE_MAX_ELEMENTS];

/* Decodes a pass at 'first', then (a full pass only) down the sensitivity
 * ladder until one reads. *used = the setting of the last attempt. Returns the
 * character count, 0 if every setting failed. */
static unsigned
barcode_decode_any (const uint16_t * p_samples, const uint32_t * p_path,
                    unsigned count, unsigned * p_count, char * p_text,
                    unsigned first, const char * p_letters, unsigned * p_used)
{
    unsigned chars = 0U;
    unsigned div;

    for (unsigned index = 0U;
         (0U == chars) && (0U != (div = barcode_ladder_div(first, index)));
         ++index)
    {
        if (index > 0U)
        {
            if (count < BARCODE_FULL_PASS)
            {
                break;

                /* A fragment (bump, tape edge): one try is enough. */
            }

            /* Same priority as the line task: let it refresh its steering. */
            (void)tk_dly_tsk(20U);
        }

        chars   = barcode_decode_stretch(p_samples, p_path, count, g_bc_work,
                                         g_bc_widths, p_count, p_text, div,
                                         p_letters);
        *p_used = div;
    }

    return chars;
}

/* Why a pass cannot be judged: the car hardly drove (placed on the track, a
 * spin on the spot), or it turned (a bend: the codes lie on straight line).
 * *turned = the largest turn (wrapping mrad). */
typedef enum
{
    BC_PASS_OK,
    BC_PASS_NOT_DRIVING,
    BC_PASS_TURNING
} barcode_pass_t;

static barcode_pass_t
barcode_pass_check (const uint32_t * p_path, const uint16_t * p_heading,
                    unsigned count, int * p_turned)
{
    *p_turned = 0;

    for (unsigned index = BARCODE_QUIET; index < count; ++index)
    {
        int difference = (int)(int16_t)(uint16_t)(p_heading[index]
                                                  - p_heading[BARCODE_QUIET]);

        if (difference < 0)
        {
            difference = -difference;
        }

        if (difference > *p_turned)
        {
            *p_turned = difference;
        }
    }

    if ((p_path[count - 1U] - p_path[BARCODE_QUIET]) < BARCODE_FAIL_MIN_DRIVE)
    {
        return BC_PASS_NOT_DRIVING;
    }

    return (*p_turned > BARCODE_FAIL_MAX_TURN) ? BC_PASS_TURNING : BC_PASS_OK;
}

/* CALIBRATING: the first usable full pass is the calibration code. It must read
 * as that letter at some setting of the ladder (default first); that setting is
 * kept. Never counted as a read, a failure or a junction. An unusable pass (not
 * driving, or turning) leaves the reader waiting for the next one. */
static void
barcode_calibrate_pass (const uint16_t * p_samples, const uint32_t * p_path,
                        const uint16_t * p_heading, unsigned count)
{
    static const char expected[] = {ROUTE_CALIBRATION_LETTER, '\0'};
    char              text[BARCODE_MAX_CHARS + 1U];

    unsigned       sample_count;
    unsigned       used = BARCODE_CONTRAST_DIV;
    int            turned;
    barcode_pass_t check
        = barcode_pass_check(p_path, p_heading, count, &turned);
    bool b_ok;

    if (BC_PASS_OK != check)
    {
        tm_printf((const UB *)"calibration: pass not usable (%s), still "
                              "waiting for *%c*\n",
                  (BC_PASS_NOT_DRIVING == check) ? "car not driving"
                                                 : "car turning",
                  ROUTE_CALIBRATION_LETTER);
        return;
    }

    b_ok = 0U
           != barcode_decode_any(p_samples, p_path, count, &sample_count, text,
                                 BARCODE_CONTRAST_DIV, expected, &used);

    if (b_ok)
    {
        g_bc_div = used;
    }

    /* Published once, after every setting was tried: the line task acts on the
     * first result it sees. */
    gb_bc_cal_ok = b_ok;
    ++g_bc_cal_results;

    if (b_ok)
    {
        tm_printf((const UB *)"barcode: calibration done, read *%c* at "
                              "sensitivity div=%u\n",
                  ROUTE_CALIBRATION_LETTER, g_bc_div);
    }
    else
    {
        tm_printf((const UB *)"barcode: calibration FAILED: no *%c* at any "
                              "sensitivity (elements=%u), keeping div=%u"
                              " - check the sensor\n",
                  ROUTE_CALIBRATION_LETTER, sample_count, g_bc_div);
    }

    barcode_set_mode(BC_WATCHING);
}

/* A full pass that did not decode: counted as a failure (line images stop the
 * car) unless the car hardly drove, was turning (a bend), or was at the
 * junction just armed. Reported before the slow dump below, so the car stops as
 * early as it can. */
static void
barcode_count_failure (const uint32_t * p_path, const uint16_t * p_heading,
                       unsigned count)
{
#if ROBO_LINE_TASK
    int            turned;
    barcode_pass_t check
        = barcode_pass_check(p_path, p_heading, count, &turned);

    if (BC_PASS_NOT_DRIVING == check)
    {
        tm_printf(
            (const UB *)"barcode: no read while not driving, not counted\n");
    }
    else if (BC_PASS_TURNING == check)
    {
        tm_printf((const UB *)"barcode: no read while turning (%d mrad), not "
                              "counted\n",
                  turned);
    }
    else if (!(gb_bc_junction_seen
               && ((g_bc_stretch_start + BARCODE_JUNCTION_BEFORE)
                   >= g_bc_junction_sample)
               && (g_bc_stretch_start
                   <= (g_bc_junction_sample + BARCODE_JUNCTION_AFTER))))
    {
        ++g_bc_failures;
        barcode_publish();
    }
#else
    (void)p_path;
    (void)p_heading;
    (void)count;
#endif
}

/* bv = readings, bp = distance driven in mm since the first sample, bh =
 * heading change in milliradians (the sensor sits to one side of the wheels, so
 * steering changes how far it travels), bw = the decoded element widths in 0.1
 * mm. tools/barcode_log.py reads these. */
static void
barcode_dump (const uint16_t * p_samples, const uint32_t * p_path,
              const uint16_t * p_heading, unsigned count, unsigned sample_count)
{
/* The stream already printed every sample; the paced dump would blind the
 * reader. */
#ifndef ROBO_BARCODE_STREAM
    for (unsigned index = 0U; index < count; ++index)
    {
        barcode_dump_num("bv", index, count, (int32_t)p_samples[index]);
    }

    for (unsigned index = 0U; index < count; ++index)
    {
        barcode_dump_num("bp", index, count,
                         (int32_t)((p_path[index] - p_path[0]) / 10U));
    }

    for (unsigned index = 0U; index < count; ++index)
    {
        barcode_dump_num(
            "bh", index, count,
            (int32_t)(int16_t)(uint16_t)(p_heading[index] - p_heading[0]));
    }

    for (unsigned index = 0U; index < sample_count; ++index)
    {
        barcode_dump_num(
            "bw", index, sample_count,
            (int32_t)(g_bc_widths[index] / BARCODE_WIDTH_PER_TENTH_MM));
    }
#else
    (void)p_samples;
    (void)p_path;
    (void)p_heading;
    (void)count;
    (void)sample_count;
#endif
}

static void
barcode_finish (const uint16_t * p_samples, const uint32_t * p_path,
                const uint16_t * p_heading, unsigned count)
{
    char text[BARCODE_MAX_CHARS + 1U];

    unsigned sample_count;
    unsigned chars;
    unsigned used = g_bc_div;
    uint32_t noise;
    uint32_t white;
    uint16_t low;
    uint16_t high;

    /* Decided by where the pass started, so a pass straddling the end of the
     * window is still skipped. */
    if (p_path[BARCODE_QUIET] < g_bc_ignore_until)
    {
        tm_printf((
            const UB *)"barcode: pass ignored (driving back after a U-turn)\n");
        return;
    }

    if (BC_CALIBRATING == g_bc_mode)
    {
        if (count >= BARCODE_FULL_PASS)
        {
            barcode_calibrate_pass(p_samples, p_path, p_heading, count);
        } /* Else a fragment. */
        return;
    }

    /* Only the route's letters are read (best fit among them,
     * barcode_decode.h). */
    char     letters[5];
    unsigned position = 0U;

    for (uint32_t route = route_get(); 0U != route; route >>= 8)
    {
        if (0U != (route & 0xFFU))
        {
            letters[position++] = (char)(route & 0xFFU);
        }
    }

    letters[position] = '\0';
    chars = barcode_decode_any(p_samples, p_path, count, &sample_count, text,
                               g_bc_div, letters, &used);
    white = barcode_level(p_samples, BARCODE_QUIET, &noise);
    barcode_range(p_samples, count, &low, &high);

    if (0U != chars)
    {
        text[chars - 1U] = '\0'; /* Drop the closing '*'. */
        ++g_bc_reads;

        if (used == g_bc_div)
        {
            tm_printf((const UB *)"barcode: *%s*\n", (const UB *)&text[1]);
        }
        else
        /* Read only at a fallback setting of the ladder (calibration still owns
         * bc_div). */
        {
            tm_printf((const UB *)"barcode: *%s* (div=%u)\n",
                      (const UB *)&text[1], used);
        }

        g_bc_letter = (float64_t)(uint8_t)text[1];
        barcode_publish();
#if ROBO_LINE_TASK
        /* The next black after this read is the junction: half the bar rise
         * just seen counts. */
        g_bc_arm_rise = (high > white) ? (high - white) / 2U : 0U;

        if (g_bc_arm_rise < BARCODE_JUNCTION_MIN)
        {
            g_bc_arm_rise = BARCODE_JUNCTION_MIN;
        }

        tm_printf(
            (const UB *)"barcode: armed, white=%u peak=%u junction=white+%u\n",
            (unsigned)white, (unsigned)high, (unsigned)g_bc_arm_rise);
        barcode_set_mode(BC_ARMED);
#endif
    }
    else if (count >= BARCODE_FULL_PASS)
    {
        /* Every setting failed: this is the "really failed" exit (line images
         * stop the car). */
        tm_printf((const UB *)"barcode: no read at any sensitivity "
                              "(div=%u,%u,%u,%u), elements=%u samples=%u"
                              " white=%u min=%u max=%u\n",
                  barcode_ladder_div(g_bc_div, 0U),
                  barcode_ladder_div(g_bc_div, 1U),
                  barcode_ladder_div(g_bc_div, 2U),
                  barcode_ladder_div(g_bc_div, 3U), sample_count, count,
                  (unsigned)white, (unsigned)low, (unsigned)high);
        barcode_count_failure(p_path, p_heading, count);
        barcode_dump(p_samples, p_path, p_heading, count, sample_count);
    }
    else
    {
        tm_printf((const UB *)"barcode: stretch ended, no code (samples=%u "
                              "elements=%u white=%u min=%u max=%u)\n",
                  count, sample_count, (unsigned)white, (unsigned)low,
                  (unsigned)high);
    }
}

/* ---- Analysis output ---- */
#ifdef ROBO_BARCODE_STREAM
/* Analysis build: prints every reading, 10 per line (0.1 s), with the time in
 * ms, the line IR state of each sample ('0' + centre + 2 * right) and the
 * distance the car has driven at each sample (0.1 mm units), to see how the
 * car's wiggle and speed line up with the sensor.
 * tests/subsystem3/test_barcode_decode.c replays exactly this format. */
static void
barcode_stream (uint16_t sample, uint32_t path)
{
    static uint16_t value[10];
    static char     ir_bits[10];
    static uint32_t point[10];
    static unsigned count;

    value[count]   = sample;
    ir_bits[count] = (char)('0' + (unsigned)gpio_get(BOARD_LINE_C)
                            + (2U * (unsigned)gpio_get(BOARD_LINE_R)));
    point[count]   = path;

    if (++count < 10U)
    {
        return;
    }

    count = 0U;
    tm_printf(
        (const UB *)"bc t=%u v=%u %u %u %u %u %u %u %u %u %u "
                    "ir=%c%c%c%c%c%c%c%c%c%c p=%u %u %u %u %u %u %u %u %u %u\n",
        (unsigned)(motion_hal_time() * 1000.0), (unsigned)value[0],
        (unsigned)value[1], (unsigned)value[2], (unsigned)value[3],
        (unsigned)value[4], (unsigned)value[5], (unsigned)value[6],
        (unsigned)value[7], (unsigned)value[8], (unsigned)value[9], ir_bits[0],
        ir_bits[1], ir_bits[2], ir_bits[3], ir_bits[4], ir_bits[5], ir_bits[6],
        ir_bits[7], ir_bits[8], ir_bits[9], point[0], point[1], point[2],
        point[3], point[4], point[5], point[6], point[7], point[8], point[9]);
}
#else
static void
barcode_stream (uint16_t sample, uint32_t path)
{
    (void)sample;
    (void)path;
}
#endif

#if defined(ROBO_BARCODE_TEST) && !defined(ROBO_LINE_STEERING)
/* Barcode-only image: shows the sensor level every 2 s while idle so it never
 * looks dead. */
static void
barcode_idle (uint16_t sample)
{
    static unsigned ticks;

    if (++ticks >= 200U)
    {
        ticks = 0U;
        tm_printf((const UB *)"barcode: idle, sensor level=%u\n",
                  (unsigned)sample);
    }
}
#else
static void
barcode_idle (uint16_t sample)
{
    (void)sample;
}
#endif

#if BARCODE_JUNCTION_RECORD
/* The analogue record of the junction's black: BARCODE_CAP_BEFORE samples up to
 * the trigger and BARCODE_CAP_AFTER after it, printed once when complete (20
 * values per line, 10 ms each). */
static uint16_t g_cap_hist[BARCODE_CAP_BEFORE]; /* Ring of the last samples. */
static unsigned g_cap_head;
static uint16_t g_cap[BARCODE_CAP_BEFORE + BARCODE_CAP_AFTER];
static unsigned g_cap_n;
static unsigned g_cap_target;

static void
barcode_record_sample (uint16_t sample)
{
    g_cap_hist[g_cap_head] = sample;
    g_cap_head             = (g_cap_head + 1U) % BARCODE_CAP_BEFORE;

    if (g_cap_n < g_cap_target)
    { /* Samples after the trigger. */
        g_cap[g_cap_n++] = sample;

        if (g_cap_n == g_cap_target)
        {
            tm_printf((const UB *)"barcode: junction samples, %u before the "
                                  "trigger + %u after, 10 ms each\n",
                      BARCODE_CAP_BEFORE, BARCODE_CAP_AFTER);

            for (unsigned index = 0U; index < g_cap_n; ++index)
            {
                tm_printf((const UB *)"%u%s", (unsigned)g_cap[index],
                          ((19U == (index % 20U)) || ((index + 1U) == g_cap_n))
                              ? "\n"
                              : " ");

                if (19U == (index % 20U))
                {
                    (void)tk_dly_tsk(20U);
                } /* Let the USB buffer drain. */
            }

            g_cap_target = 0U;
        }
    }
}

static void
barcode_record_junction (void)
{
    for (unsigned index = 0U; index < BARCODE_CAP_BEFORE; ++index)
    {
        /* Oldest first, trigger last. */
        g_cap[index] = g_cap_hist[(g_cap_head + index) % BARCODE_CAP_BEFORE];
    }

    g_cap_n      = BARCODE_CAP_BEFORE;
    g_cap_target = BARCODE_CAP_BEFORE + BARCODE_CAP_AFTER;
}
#else
static void
barcode_record_sample (uint16_t sample)
{
    (void)sample;
}

#if ROBO_LINE_TASK
static void
barcode_record_junction (void)
{
}
#endif
#endif

#if ROBO_LINE_TASK
/* ARMED: the first black after a valid read (clearly above the quiet level for
 * two samples in a row) is the junction. Reported at once; the line task
 * decides what to do about it. The quiet level is the one from the last sample
 * taken while no stretch was running. */
static void
barcode_check_junction (const barcode_seg_t * p_seg, uint16_t sample,
                        uint32_t sample_time)
{
    static unsigned hits;

    if ((BC_ARMED != g_bc_mode) || (!p_seg->level_ok)
        || (sample <= (p_seg->level + g_bc_arm_rise)))
    {
        hits = 0U;
        return;
    }

    if (++hits < BARCODE_JUNCTION_HITS)
    {
        return;
    }

    hits = 0U;
    barcode_set_mode(BC_WATCHING);
    g_bc_junction_sample = g_bc_sample_no;
    gb_bc_junction_seen  = true;
    ++g_bc_junctions;
    barcode_publish();
    tm_printf((const UB *)"barcode: junction black=%u white=%u threshold=%u "
                          "path=%u\n",
              (unsigned)sample, (unsigned)p_seg->level,
              (unsigned)(p_seg->level + g_bc_arm_rise), (unsigned)sample_time);
    barcode_record_junction();
}
#else
static void
barcode_check_junction (const barcode_seg_t * p_seg, uint16_t sample,
                        uint32_t sample_time)
{
    (void)p_seg;
    (void)sample;
    (void)sample_time;
}
#endif

void
barcode_task (INT code, void * p_context)
{
    /* The stretch being collected, with its lead-in ring. */
    static barcode_seg_t seg;
    uint64_t             next;

    (void)code;
    (void)p_context;
    barcode_seg_init(&seg);

    if (!ir_adc_init(BOARD_BARCODE))
    {
        /* No reader: the line task never sees a read or a failure, so it just
         * follows the line. */
        tm_printf((const UB *)"barcode: ADC not responding, reader off\n");

        for (;;)
        {
            (void)tk_dly_tsk(1000U);
        } /* A task must never return (pc=0 hard fault). */
    }

    tm_printf((const UB *)"barcode: reader running on GP28 (distance widths, "
                          "%u.%u mm steps)\n",
              BARCODE_PATH_STEP / 10U, BARCODE_PATH_STEP % 10U);
    next = time_us_64() + BARCODE_SAMPLE_US;

    for (;;)
    {
        uint16_t            sample      = barcode_sample(&next);
        uint32_t            sample_time = barcode_path();
        barcode_seg_event_t event;

        ++g_bc_sample_no;
        barcode_stream(sample, sample_time);

        if (gb_bc_dirty)
        {
            barcode_publish();
        }

        barcode_take_requests();

        if ((BC_IGNORING == g_bc_mode) && (sample_time >= g_bc_ignore_until))
        {
            barcode_set_mode(BC_WATCHING);
        }

        barcode_record_sample(sample);
        barcode_check_junction(&seg, sample, sample_time);
        event = barcode_seg_feed(&seg, sample, sample_time, g_bc_heading);

        if (BARCODE_SEG_IDLE == event)
        {
            barcode_idle(sample);
        }
        else if (BARCODE_SEG_STARTED == event)
        {
            g_bc_stretch_start = g_bc_sample_no;
            gb_bc_code_pass    = true;
            tm_printf((const UB *)"barcode: reading...\n");
        }
        else if (BARCODE_SEG_DONE == event)
        {
            gb_bc_code_pass = false;
            barcode_finish(seg.samples, seg.path, seg.heading, seg.count);
        }
    }
}
#endif

/*** end of file ***/
