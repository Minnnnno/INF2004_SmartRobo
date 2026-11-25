/** @file test_compact.c
 * @brief Packing, numerical precision and owner-state regression checks.
 */
#include "motion_compact.h"
#include "board_config.h"
#include "encoder_capture.h"
#include <stdio.h>
#include <string.h>

static uint32_t g_checks = 0u;
#define CHECK(CONDITION) do { ++g_checks; if (!(CONDITION)) { \
    (void)fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #CONDITION); \
    return false; } } while (false)
#define NEAR(LEFT, RIGHT, ERROR) CHECK(fabs((LEFT) - (RIGHT)) <= (ERROR))

static bool test_integer_fields(void)
{
    const uint32_t widths[] = {14u, 9u, 10u};
    const int32_t limits[] = {5000, 250, 360};
    const uint32_t offsets[] = {0u, 14u, 32u};
    for (uint32_t field = 0u; field < 3u; ++field)
    {
        for (int32_t value = -limits[field]; value <= limits[field]; ++value)
        {
            uint8_t bytes[MOTION_PAYLOAD_BYTES] = {0u};
            motion_compact_put(bytes, offsets[field], widths[field], value);
            CHECK(value == motion_compact_get(bytes, offsets[field], widths[field]));
        }
    }
    uint8_t negative_one[MOTION_PAYLOAD_BYTES] = {0u};
    motion_compact_put(negative_one, 0u, 14u, -1);
    CHECK((0xffu == negative_one[0]) && (0x3fu == negative_one[1]));
    return true;
}

static bool test_round_trips(void)
{
    motion_config_t config = board_motion_config();
    motion_command_t original = {.id=UINT32_MAX, .generation=UINT32_MAX,
        .mode=DISTANCE_MOVE, .expires=12345678.3000001, .deadline=12345699.987654};
    motion_command_t decoded = {0};
    motion_storage_command_t stored = {0};
    CHECK(motion_compact_limits(&config));
    for (int32_t value = -5000; value <= 5000; ++value)
    {
        original.value = (float64_t)value / 1000.0;
        CHECK(motion_compact_encode(&original, &config, &stored));
        CHECK(motion_compact_decode(&stored, &config, &decoded));
        NEAR(decoded.value, original.value, 1e-12);
        CHECK((UINT32_MAX == decoded.id) && (UINT32_MAX == decoded.generation));
        CHECK(0 == memcmp(&decoded.expires, &original.expires, sizeof(float64_t)));
        CHECK(0 == memcmp(&decoded.deadline, &original.deadline, sizeof(float64_t)));
    }
    original.mode = WHEEL_SPEED_TRACKING;
    original.value = 0.0;
    for (int32_t value = -250; value <= 250; ++value)
    {
        original.speed[0] = (float64_t)value / 1000.0;
        original.speed[1] = -(float64_t)value / 1000.0;
        CHECK(motion_compact_encode(&original, &config, &stored));
        CHECK(motion_compact_decode(&stored, &config, &decoded));
        NEAR(decoded.speed[0], original.speed[0], 1e-12);
        NEAR(decoded.speed[1], original.speed[1], 1e-12);
    }
    original.mode = ENCODER_TURN;
    original.speed[0] = 0.0;
    original.speed[1] = 0.0;
    for (int32_t value = -360; value <= 360; ++value)
    {
        original.value = (float64_t)value * MOTION_RAD_PER_DEG;
        CHECK(motion_compact_encode(&original, &config, &stored));
        CHECK(motion_compact_decode(&stored, &config, &decoded));
        NEAR(decoded.value, original.value, 1e-12);
    }
    int32_t rounded = 0;
    CHECK(motion_compact_quantize(0.0005, 1000.0, 5000, &rounded) && (1 == rounded));
    CHECK(motion_compact_quantize(-0.0005, 1000.0, 5000, &rounded) && (-1 == rounded));
    CHECK(motion_compact_quantize(-0.00049, 1000.0, 5000, &rounded) && (0 == rounded));
    CHECK(motion_compact_quantize(-90.5, 1.0, 360, &rounded) && (-91 == rounded));
    CHECK(motion_compact_quantize(0.1255, 1000.0, 250, &rounded) && (126 == rounded));
    return true;
}

static bool test_invalid_inputs(void)
{
    motion_config_t config = board_motion_config();
    motion_command_t original = {.id=1u, .mode=DISTANCE_MOVE, .expires=1.0, .deadline=20.0};
    motion_storage_command_t stored = {0};
    motion_storage_command_t unchanged = {0};
    motion_command_t decoded = {.id=99u};
    const float64_t bad[] = {NAN, INFINITY, -INFINITY, 5.00000001, -5.00000001};
    for (uint32_t index = 0u; index < sizeof(bad)/sizeof(bad[0]); ++index)
    {
        original.value = bad[index];
        CHECK(!motion_compact_encode(&original, &config, &stored));
        CHECK(0 == memcmp(&stored, &unchanged, sizeof(stored)));
    }
    original.value = 0.0;
    original.mode = WHEEL_SPEED_TRACKING;
    original.speed[0] = 0.25000001;
    CHECK(!motion_compact_encode(&original, &config, &stored));
    original.speed[0] = -0.25000001;
    CHECK(!motion_compact_encode(&original, &config, &stored));
    original.speed[0] = 0.0;
    original.mode = ENCODER_TURN;
    original.value = 360.000001 * MOTION_RAD_PER_DEG;
    CHECK(!motion_compact_encode(&original, &config, &stored));
    original.mode = (motion_mode_t)-1;
    CHECK(!motion_compact_encode(&original, &config, &stored));
    original.mode = DISTANCE_MOVE;
    original.value = 0.1;
    CHECK(motion_compact_encode(&original, &config, &stored));
    stored.payload[5] |= 0x80u;
    CHECK(!motion_compact_decode(&stored, &config, &decoded) && (99u == decoded.id));
    CHECK(motion_compact_encode(&original, &config, &stored));
    motion_compact_put(stored.payload, 0u, 14u, 5001);
    CHECK(!motion_compact_decode(&stored, &config, &decoded));
    CHECK(motion_compact_encode(&original, &config, &stored));
    motion_compact_put(stored.payload, 14u, 9u, -251);
    CHECK(!motion_compact_decode(&stored, &config, &decoded));
    CHECK(motion_compact_encode(&original, &config, &stored));
    motion_compact_put(stored.payload, 32u, 10u, -361);
    CHECK(!motion_compact_decode(&stored, &config, &decoded));
    CHECK(motion_compact_encode(&original, &config, &stored));
    stored.payload[5] = 28u; /* Unsigned mode7, not a negative numerical field. */
    CHECK(!motion_compact_decode(&stored, &config, &decoded));
    config.max_distance = 6.0;
    CHECK(!motion_compact_encode(&original, &config, &stored));
    config = board_motion_config();
    config.max_speed = 0.1006;
    original.mode = WHEEL_SPEED_TRACKING;
    original.speed[0] = 0.1006;
    CHECK(motion_compact_encode(&original, &config, &stored));
    CHECK(!motion_compact_decode(&stored, &config, &decoded)); /* Rounded target exceeds configuration. */
    CHECK(!motion_compact_encode(NULL, &config, &stored));
    CHECK(!motion_compact_decode(NULL, &config, &decoded));
    return true;
}

static bool test_arc_and_manual(void)
{
    motion_config_t config = board_motion_config();
    motion_command_t original = {.id=4u, .mode=ARC_MOVE, .value=0.2004,
        .angle=20.4*MOTION_RAD_PER_DEG, .duration=3.01234567, .expires=0.3, .deadline=20.0};
    original.speed[0] = (original.value-original.angle*config.track/2.0)/original.duration;
    original.speed[1] = (original.value+original.angle*config.track/2.0)/original.duration;
    motion_storage_command_t stored = {0};
    motion_command_t decoded = {0};
    CHECK(motion_compact_encode(&original, &config, &stored));
    CHECK(motion_compact_decode(&stored, &config, &decoded));
    NEAR(decoded.value, 0.200, 1e-12);
    NEAR(decoded.angle, 20.0*MOTION_RAD_PER_DEG, 1e-12);
    NEAR(decoded.duration, original.duration, 1e-12);
    NEAR(decoded.speed[0]*decoded.duration, decoded.value-decoded.angle*config.track/2.0, 1e-12);
    NEAR(decoded.speed[1]*decoded.duration, decoded.value+decoded.angle*config.track/2.0, 1e-12);
    original.mode = MANUAL_OUTPUT;
    original.value = 0.0;
    original.speed[0] = -0.123456789;
    original.speed[1] = 0.0123456789;
    CHECK(motion_compact_encode(&original, &config, &stored));
    CHECK(motion_compact_decode(&stored, &config, &decoded));
    CHECK(0 == memcmp(original.speed, decoded.speed, sizeof(original.speed)));
    return true;
}

static bool test_owner_behaviour(void)
{
    motion_config_t config = board_motion_config();
    encoder_sample_t sample = {.reliable=true};
    motion_t motion;
    motion_init(&motion, &config, sample, 0.0, true);
    for (uint32_t step = 1u; step <= 40u; ++step)
    {
        motion_step(&motion, sample, (float64_t)step*0.01);
    }
    CHECK(READY == motion.status.state);
    motion_command_t command = {.id=1u, .generation=motion.status.generation,
        .mode=WHEEL_SPEED_TRACKING, .speed={0.0804,-0.0804}, .expires=0.65, .deadline=20.0};
    CHECK(R_OK == motion_accept(&motion, &command, 0.4));
    motion_command_t decoded;
    CHECK(motion_compact_decode(&motion.command, &config, &decoded));
    NEAR(decoded.speed[0], 0.080, 1e-12);
    NEAR(decoded.speed[1], -0.080, 1e-12);
    command.expires = 0.64;
    CHECK(R_STALE == motion_accept(&motion, &command, 0.4));
    command.expires = 0.66;
    command.id = 2u;
    CHECK(R_BUSY == motion_accept(&motion, &command, 0.4));
    for (uint32_t step = 41u; step <= 65u; ++step)
    {
        motion_step(&motion, sample, (float64_t)step*0.01);
    }
    CHECK((FAULT == motion.status.state) && (F_EXPIRED == motion.status.fault));
    CHECK(motion.status.inhibited);
    NEAR(motion.status.output[0], 0.0, 1e-15);
    CHECK(0u != ((uint32_t)motion.fault_flags & (UINT32_C(1) << ((uint32_t)F_EXPIRED-1u))));
    motion_raise_fault(&motion, F_SYSTEM, 0.66);
    CHECK(F_EXPIRED == motion.status.fault); /* First public cause remains latched. */
    CHECK(0u != ((uint32_t)motion.fault_flags & (UINT32_C(1) << ((uint32_t)F_SYSTEM-1u))));
    motion_event_t event;
    CHECK(motion_next_event(&motion, &event));
    CHECK((EV_FAULT == event.type) && (F_EXPIRED == event.fault) && (1u == event.id));
    motion_cancel(&motion, 0.67);
    command.id = 1u;
    command.expires = 0.9;
    CHECK(R_STALE == motion_accept(&motion, &command, 0.68));
    CHECK(motion.status.hold && motion.status.inhibited);
    return true;
}

static bool test_numerical_precision(void)
{
    motion_config_t config = board_motion_config();
    float64_t quantum = 3.141592653589793*config.diameter[0]/config.counts_per_rev[0];
    float single_before = 2048.0f;
    float single_after = single_before+(float)quantum;
    float64_t double_after = 2048.0+quantum;
    CHECK(fabs((float64_t)single_after-(float64_t)single_before) < quantum/2.0);
    NEAR(double_after-2048.0, quantum, 1e-12);
    odometry_t odometry = {.previous={UINT32_MAX-2u,2u}, .travel={2048.0,2048.0}};
    encoder_sample_t sample = {.count={UINT32_MAX-2u,2u}, .reliable=true};
    for (uint32_t step = 0u; step < 10000u; ++step)
    {
        ++sample.count[0];
        --sample.count[1];
        CHECK(odometry_step(&odometry, &config, &sample, 0.01));
    }
    NEAR(odometry.travel[0], 2048.0+10000.0*quantum, 5e-9);
    CHECK(odometry.speed[1] < 0.0);
    CHECK(5 == encoder_delta(2u, UINT32_MAX-2u));
    CHECK(-5 == encoder_delta(UINT32_MAX-2u, 2u));
    (void)printf("precision: one count %.9f mm; binary32 loses it at 2048 m, binary64 preserves it\n", quantum*1000.0);
    return true;
}

int main(void)
{
    bool b_passed = test_integer_fields() && test_round_trips() && test_invalid_inputs() &&
                    test_arc_and_manual() && test_owner_behaviour() && test_numerical_precision();
    (void)printf("Buddy 2 compact: %s (%u checks)\n", b_passed ? "PASS" : "FAIL", g_checks);
    return b_passed ? 0 : 1;
}
