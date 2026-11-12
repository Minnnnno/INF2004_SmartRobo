/* Barcode-only serial log (analysis image, -DROBO_BARCODE_LOG_ONLY=1 on the integrated build).
 * tm_printf (lib/libtm/libtm_printf.c) asks this filter about every message, by its format string.
 * Kept: the barcode reader, line/barcode state changes, route actions and calibration, plus the
 * firmware banner, crash reports and errors, so a stop never goes unexplained.
 * Also kept: the raw sensor stream ("bc t=" lines, with -DROBO_BARCODE_STREAM=1), which
 * tests/test_barcode_decode.c replays. Dropped: everything else (line_pid, IMU, obstacle, stack
 * reports); the bv/bp/bh/bw dumps are off when the stream is on (barcode.c). */
#ifdef ROBO_BARCODE_LOG_ONLY
#include <tk/tkernel.h>

static BOOL log_starts_with(const UB *text, const char *prefix)
{
    while (*prefix != '\0') {
        if ((char)*text++ != *prefix++) { return FALSE; }
    }
    return TRUE;
}

BOOL robo_log_allowed(const UB *format)
{
    static const char *const keep[] = {
        "barcode", "[S3-", "route", "calibration", "bc t=", "firmware=", "LAST CRASH", "[ERROR]"
    };
    if (format == NULL) { return FALSE; }
    for (unsigned i = 0U; i < sizeof keep / sizeof keep[0]; ++i) {
        if (log_starts_with(format, keep[i])) { return TRUE; }
    }
    return FALSE;
}
#else
typedef int robo_log_filter_unused; /* Not an empty file in normal builds. */
#endif
