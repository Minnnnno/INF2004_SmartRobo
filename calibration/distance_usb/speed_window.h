#ifndef CALIBRATION_SPEED_WINDOW_H
#define CALIBRATION_SPEED_WINDOW_H
#include <stdbool.h>
#include <stdint.h>

/* Read-only measurement. No motor commands or printing in timer context. */
typedef struct {
    bool started, complete;
    uint64_t start_us, end_us;
    double start_mm[2], end_mm[2];
} speed_window_t;

static inline void speed_window_step(speed_window_t *w, const double distance[2], uint64_t elapsed_us) {
    if (w->complete) { return; }
    double centre=(distance[0]+distance[1])*0.5;
    if (!w->started && centre>=70.0) {
        w->started=true; w->start_us=elapsed_us;
        for (unsigned i=0U;i<2U;++i) { w->start_mm[i]=distance[i]; }
    }
    if (w->started && centre>=170.0 && elapsed_us>w->start_us) {
        w->complete=true; w->end_us=elapsed_us;
        for (unsigned i=0U;i<2U;++i) { w->end_mm[i]=distance[i]; }
    }
}
#endif
