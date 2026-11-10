/* Barcode decoding: pure computation, no hardware or kernel, so the host replay test
 * (tests/test_barcode_decode.c) runs exactly the code the robot runs. barcode.c samples the
 * sensor, feeds each sample to the segmenter and decodes every stretch it hands back.
 *
 * Readings: higher = darker (the TCRT5000 AO rises over black). Path: distance driven, in 0.1 mm,
 * never decreasing (motion status path). Heading: wrapping milliradians, only kept for dumps. */
#ifndef BARCODE_DECODE_H
#define BARCODE_DECODE_H
#include <stdbool.h>
#include <stdint.h>

#define BARCODE_SAMPLE_US    10000U  /* 100 samples per second. */
#ifndef BARCODE_SAMPLES
#define BARCODE_SAMPLES      1000U   /* Longest stretch kept for decoding (10 s; 0.4 m on the 0.4 mm grid). */
#endif
#define BARCODE_QUIET        50U     /* First 0.5 s of a stretch is the quiet level: the noise reference. */
#define BARCODE_FLAT         50U     /* A stretch ends after 0.5 s of flat reading. */
#define BARCODE_MIN_ACTIVE   80U     /* Shorter stretches are bumps, not barcode passes (0.8 s). */
#define BARCODE_MAX_ELEMENTS 99U     /* 10 characters of 9 elements plus gaps. */
#define BARCODE_MAX_BARS     ((BARCODE_MAX_ELEMENTS + 1U) / 2U)
#define BARCODE_MAX_CHARS    ((BARCODE_MAX_ELEMENTS + 1U) / 10U)

/* Splits the sample stream into stretches: a stretch starts when a reading leaves the quiet level
 * (keeping the BARCODE_QUIET samples before it as the lead-in) and ends once the reading is flat again. */
typedef struct {
    uint16_t samples[BARCODE_SAMPLES];
    uint32_t path[BARCODE_SAMPLES];
    uint16_t heading[BARCODE_SAMPLES];
    unsigned count;          /* Samples in the current stretch; 0 while idle. */
    uint16_t quiet[BARCODE_QUIET];
    uint32_t quiet_path[BARCODE_QUIET];
    uint16_t quiet_heading[BARCODE_QUIET];
    unsigned head, filled;
    uint32_t prominence;     /* Rise that leaves the quiet level, from the last idle lead-in. */
    uint32_t level;          /* Quiet level, from the last sample taken while idle. */
    bool level_ok;
    bool done;               /* The last feed completed a stretch; the next feed starts afresh. */
} barcode_seg_t;

typedef enum {
    BARCODE_SEG_IDLE,     /* Quiet: the sample went into the lead-in ring. */
    BARCODE_SEG_STARTED,  /* A stretch just started (the lead-in plus this sample). */
    BARCODE_SEG_RUNNING,  /* Inside a stretch. */
    BARCODE_SEG_DONE      /* The stretch is complete: samples/path/heading/count hold it until the next feed. */
} barcode_seg_event_t;

void barcode_seg_init(barcode_seg_t *g);
barcode_seg_event_t barcode_seg_feed(barcode_seg_t *g, uint16_t s, uint32_t path, uint16_t heading);

/* Decodes one stretch. work is scratch (BARCODE_SAMPLES); widths receives the element widths
 * (BARCODE_MAX_ELEMENTS, in BARCODE_SAMPLE_US units per decoding step) and *elements their count.
 * Returns the character count of a complete *...* read (text holds it, '\0'-terminated), else 0. */
unsigned barcode_decode_stretch(const uint16_t *samples, const uint32_t *path, unsigned count,
                                uint16_t *work, uint32_t *widths, unsigned *elements, char *text);

/* Mean and mean absolute deviation (the noise) of n samples. */
uint32_t barcode_level(const uint16_t *s, unsigned n, uint32_t *noise);
void barcode_range(const uint16_t *s, unsigned n, uint16_t *low, uint16_t *high);

#ifndef BARCODE_PATH_STEP
#define BARCODE_PATH_STEP    4U      /* Decoding step: 0.4 mm of driven distance (path is in 0.1 mm). */
#endif
/* Width units in widths[] per 0.1 mm of driven distance (for dumps). */
#define BARCODE_WIDTH_PER_TENTH_MM (BARCODE_SAMPLE_US / BARCODE_PATH_STEP)
#endif
