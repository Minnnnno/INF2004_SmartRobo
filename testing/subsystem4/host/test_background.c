/* Exercise the actual infinite task using finite mocked scheduler ticks. */
#include "subsystem4.h"
#include "gy511_read.h"
#include "robot.h"
#include <stdarg.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static jmp_buf g_exit;
static unsigned g_tick;
static unsigned g_limit;
static unsigned g_reads;
static unsigned g_messages;
static ER g_configuration;
static ER g_results[32];
static subsystem_status_t g_history[32];
static bool g_online;
static unsigned g_checks;

static void require(bool b_condition, char const *p_name)
{
    if (!b_condition)
    {
        fprintf(stderr, "FAIL: %s\n", p_name);
        exit(EXIT_FAILURE);
    }
    g_checks++;
}

ER subsystem4_gy511_configure(void)
{
    return g_configuration;
}

ER gy511_read_sample(gy511_sample_t *p_sample)
{
    g_reads++;
    if (g_results[g_tick] == E_OK)
    {
        *p_sample = (gy511_sample_t){-61, 27, -1039,
                                    INT16_MIN, INT16_MAX, -100};
    }
    return g_results[g_tick];
}

double motion_hal_time(void)
{
    return (double)g_tick * 0.05;
}

bool robot_update_subsystem(request_source_t source,
                            subsystem_status_t const *p_status)
{
    require(source == SOURCE_TERRAIN, "only updates terrain status");
    g_history[g_tick] = *p_status;
    return true;
}

ER tk_dly_tsk(UW delay)
{
    require(delay == ((g_configuration == E_OK) ? 50U : 1000U),
            "task yields between samples");
    g_tick++;
    if (g_tick >= g_limit)
    {
        longjmp(g_exit, 1);
    }
    return E_OK;
}

INT tm_printf(UB const *p_format, ...)
{
    (void)p_format;
    return 0;
}

INT tm_sprintf(UB *p_text, UB const *p_format, ...)
{
    int length;
    va_list args;
    va_start(args, p_format);
    length = vsprintf((char *)p_text, (char const *)p_format, args);
    va_end(args);
    return length;
}

BOOL comms_is_online(void)
{
    return g_online ? 1 : 0;
}

ER comms_send_telemetry(char const *p_name, char const *p_payload)
{
    require(strlen(p_payload) < 96U, "telemetry fits the queue limit");
    require((strcmp(p_name, "imu_acc") == 0) ||
            (strcmp(p_name, "imu_mag") == 0) ||
            (strcmp(p_name, "terrain") == 0), "documented telemetry names");
    g_messages++;
    /* Full queues must never stop the sampling task. */
    return E_TMOUT;
}

static void reset(unsigned limit)
{
    g_tick = 0U;
    g_limit = limit;
    g_reads = 0U;
    g_messages = 0U;
    g_configuration = E_OK;
    g_online = false;
    memset(g_results, 0, sizeof(g_results));
    memset(g_history, 0, sizeof(g_history));
}

static void run(void)
{
    if (setjmp(g_exit) == 0)
    {
        terrain_task(0, NULL);
    }
}

int main(void)
{
    reset(15U);
    run();
    require(g_reads == 15U, "continuous background samples");
    require(g_history[14].available, "fresh data available");
    require(g_history[14].values[3] == 2.0, "only pitch is valid");
    require(g_history[14].values[0] == 0.0 &&
            g_history[14].values[2] == 0.0, "no invented heading or height");
    require(g_messages == 0U, "offline does not fill MQTT queues");

    reset(3U);
    g_results[1] = E_BUSY;
    run();
    require(g_history[1].available, "one not-ready poll keeps fresh sample");
    require(g_history[1].timestamp == 0.0, "old data never gets a new time");
    require(g_history[2].timestamp == 0.10, "next sample advances time");

    reset(8U);
    for (unsigned index = 1U; index < 8U; index++)
    {
        g_results[index] = E_BUSY;
    }
    run();
    require(!g_history[5].available, "stale readings become unavailable");
    require(g_history[5].values[3] == 0.0, "stale tilt validity cleared");

    reset(4U);
    g_results[1] = E_IO;
    run();
    require(!g_history[1].available, "I2C error invalidates immediately");
    require(g_history[2].available, "sampling recovers on next good read");

    reset(3U);
    g_configuration = E_IO;
    run();
    require(g_reads == 0U, "failed configuration prevents reading");
    require(!g_history[2].available, "missing sensor stays unavailable");

    reset(15U);
    g_online = true;
    run();
    require(g_messages >= 6U, "raw and terrain data queued online");
    require(g_reads == 15U, "full telemetry queues do not stop monitoring");
    printf("PASS: %u background task checks\n", g_checks);
    return EXIT_SUCCESS;
}
