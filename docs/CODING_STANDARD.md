# Coding standard

The team's C code follows Barr Group's *Embedded C Coding Standard*
(BARR-C:2018, https://barrgroup.com/coding-standard). This page lists only how the
project applies it and where it deviates. Rule numbers are BARR-C's.

**Scope:** `app_program/` and the team headers directly in `include/`. The
micro T-Kernel template, drivers and libraries (`kernel/`, `lib/`, `device/`,
`include/sys|tk|tm|bsp`) are third-party and left alone. Host tests in `tests/`
only follow the names they share with the firmware.

## Tools

| Command | What it does |
|---|---|
| `python tools/barr_format.py` | Lays out every team file (clang-format 20 or later, found automatically) |
| `python tools/barr_format.py --check` | Reports files that would change |
| `python tools/barr_check.py` | Checks the enforceable rules; exits 1 on a violation |
| `python tools/barr_check.py --notes` | Also lists "preferred practice" cases |

Run the formatter and the checker before every commit.

## Project settings

| Rule | Setting |
|---|---|
| 1.1 | C11 (the kernel and `_Static_assert` need it) instead of C99 |
| 5.4.b | `float32_t` and `float64_t` from `include/float_types.h` |
| 6.2.a | 125 lines per function: 80-column wrapping lengthens functions |
| 7.1 | `g_` global or file static, `p_` pointer, `b_` Boolean, combined as `gp_`, `gb_` |

## Deviations

Each needs the team lead's approval under BARR-C's deviation procedure.
**None has been approved yet.**

| Rule | Where | Why |
|---|---|---|
| 1.2.a | `TF_FIELDS` in `include/telemetry_frame.h` | `tools/dashboard.py` parses one field per line |
| 1.6.a | Most casts have no comment | About 340 cast a string to the kernel's `UB` type and 420 discard a result with `(void)`. Most others convert a checked value for printing |
| 1.7.d | `continue` in 7 functions | An `if`/`else` there would nest deeper than 8.2.b allows |
| 1.8.c | Not every global shared by tasks is `volatile` | Tasks share state through the status mutex or an interrupt lock; this has not been audited object by object |
| 2.1.c | `ROBO_*_TEST` and `S5_DIAG` code has no `NDEBUG` guard | Separate build options select it |
| 4.1.b, 4.2.a | Module names and one header per source | File names and shared headers are in the build, tests and documents |
| 4.1.c | `app_program/subsystem2/math.c` | There is no project `math.h`, so nothing is hidden |
| 4.3.b | Files split by `#if` are ordered within each part | Reordering across `#if` would change which code is built |
| 5.1.c, 6.1.i | Public names use the subsystem's prefix (`s5_`, `motion_`, `comms_`) | The prefix is shared by a subsystem's files and is the interface other subsystems use |
| 5.2.a | `int` and `unsigned` where the width does not matter | Matches the kernel and register functions |
| 5.4.a, 5.4.b.iv | Floating point, and `== 0.0` in `motion.c` and `steering.c` | Positions are running sums; an exact 0 means "not used" in a command |
| 5.5.b | Bit-fields in private state | Never sent or stored outside the program; sizes are asserted |
| 6.2.a | `manual_console()`, `line_drive()` | Four or more build options interleaved; owners to split with a hardware test |
| 6.2.c | More than one `return` in 128 functions | Guards and loops, where one exit would nest deeper than 8.2.b allows |
| 6.3.a | `TF_FIELDS`, `ROUTE_TABLE`, `CHECK`, `TEST_CHECK` | One table generates several lists, or the file and line are needed |
| 6.5.b | Interrupt handlers are not all named `_isr` | Registered by address; names kept for the fault records |
| 7.1 | Structure members keep their names (`hold`, `kp`, `id`) | Shared status and configuration are used by every subsystem |
| 8.5.b | `setjmp` in host tests | Ends a task's endless loop in a test |

## Abbreviations (1.5.b)

In addition to BARR-C's Appendix A:

| Short | Meaning |
|---|---|
| bc | barcode |
| ddeg | tenth of a degree |
| deg, mm, mg | degree, millimetre, thousandth of g |
| enc | encoder |
| ir | infrared line sensor |
| mag | magnetometer |
| mc | motion class |
| odom | odometry |
| s4, s5 | Subsystem 4, Subsystem 5 |
| tf | telemetry frame |
| us (file names, `s5_us_`) | ultrasonic |

## File templates (4.4.a)

Header `module.h`:

```c
/** @file module.h
 *
 * @brief What the module is for.
 */

#ifndef MODULE_H
#define MODULE_H

/*! @brief What it does. */
int32_t module_read(uint32_t channel);

#endif /* MODULE_H */

/*** end of file ***/
```

Source `module.c`: the same `@file` block, includes, definitions, static data,
private prototypes, public functions, private functions, end-of-file comment.

```c
/*! @brief What it does. */
int32_t
module_read (uint32_t channel)
{
    int32_t result = 0;

    if (channel < MODULE_CHANNELS)
    {
        result = module_sample(channel);
    }

    return result;
}
```
