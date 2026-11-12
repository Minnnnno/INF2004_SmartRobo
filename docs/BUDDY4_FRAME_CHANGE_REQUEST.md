# Change request: Buddy 4 fields in the telemetry frame (layout v1 → v2)

**From:** Buddy 4 (IMU / terrain) · **To:** Buddy 1 (Wi-Fi / MQTT / dashboard)
**Status:** proposed, needs your review · **Frame size:** stays **21 bytes**

## Summary

Buddy 4 now does motion classification and turn-rate monitoring, but the
results only reach the bus slot and the serial console. This request adds them
to the status frame **without growing it**:

1. **Add `M_CLASS`** (3 bits): the motion class.
2. **Rename `T_HEADING` → `TURN_RATE`** (9 bits): `T_HEADING` is always 0
   (no magnetic heading is published), so its 9 bits carry the turn rate instead.
3. **Shrink `HUMP_MM` 8 → 7 bits**: Buddy 4 stores the hump peak in 7 bits
   (0..127 mm), so the 8th bit is never used.
4. **`T_VALID` bit 0** changes meaning: "heading valid" (never set) → "motion
   class and turn rate valid".
5. **Bump `TF_LAYOUT_VERSION` 1 → 2**, so an old dashboard rejects new frames
   instead of misreading them.

Frame: 161 → **163 of 168 bits** (+3 `M_CLASS`, −1 `HUMP_MM`), 5 spare bits.

## Why

| Source | Requirement | Covered by |
|---|---|---|
| Project write-up, Buddy 4 tasks | "Turn-rate monitoring", "Motion event reporting" | `TURN_RATE`, `M_CLASS` |
| Week 6 FR9 | "identify stationary, accelerating, turning, climbing, descending and sudden-impact events and **provide yaw-rate measurements**" | `M_CLASS`, `TURN_RATE` |
| Change Requirement 1 | minimum bits per field | dead `T_HEADING` bits reused; `HUMP_MM` trimmed to its real range |
| Change Requirement, 11 November 2026 | sudden-impact (collision) detection removed; the IMU detects **lifted or overturned** and the motors stop | lifted = `T_STATE` 7 (already in the v1 frame); overturned = `M_CLASS` 7, so it reaches the dashboard only once `M_CLASS` is added |

## Field changes (Buddy 4 part of `TF_FIELDS`)

| Field | v1 | v2 | Range | Meaning |
|---|---|---|---|---|
| `T_VALID` | 3 | 3 | 0..7 | bit0 **motion class + turn rate valid** (was heading), bit1 pitch, bit2 hump |
| `T_STATE` | 3 | 3 | 0..7 | 7 = LIFTED (Change Requirement, 11 November 2026) |
| **`M_CLASS`** | — | **3** | 0..7 | **new**, see codes below |
| `PITCH` | 8 | 8 | −90..90 | unchanged |
| `T_HEADING` → **`TURN_RATE`** | 9 | 9 | **−256..255** | turn rate, **deg/s, + = left (anticlockwise), − = right** |
| `HUMP_MM` | 8 | **7** | **0..127** | highest hump peak this run, mm |

`M_CLASS` codes (same as `TERRAIN_MC_*` in `include/terrain_status.h`):

| Code | Class | Code | Class |
|---|---|---|---|
| 0 | STATIONARY | 4 | TURNING |
| 1 | MOVING | 5 | CLIMBING |
| 2 | ACCELERATING (speed changing, braking included) | 6 | DESCENDING |
| 3 | REVERSING | 7 | OVERTURNED (robot overturned; Buddy 4 has stopped the motors) |

**Why 9 bits for `TURN_RATE`:** the fastest possible pivot is both wheels at
the 250 mm/s limit in opposite directions: 500 / 115 mm = 4.35 rad/s ≈ 249 deg/s.
±255 covers the full physical range; 8 bits (±127) would clip a fast pivot. The
fastest seen on the robot so far is −100 deg/s.

## Code changes

### 1. `include/telemetry_frame.h`

```c
#define TF_LAYOUT_VERSION	2U
...
	X(T_VALID,     0,     7,      3,  B4, "bit0 motion class+turn rate, bit1 pitch, bit2 hump height valid") \
	X(T_STATE,     0,     7,      3,  B4, "terrain state (8 states, B4 statechart; 7 = LIFTED)") \
	X(M_CLASS,     0,     7,      3,  B4, "motion class: 0 still 1 moving 2 accel 3 reverse 4 turn 5 climb 6 descend 7 overturned") \
	X(PITCH,       -90,   90,     8,  B4, "pitch, degrees, + nose up") \
	X(TURN_RATE,   -256,  255,    9,  B4, "turn rate from wheel speeds, deg/s, + left") \
	X(HUMP_MM,     0,     127,    7,  B4, "peak hump height this run, mm") \
...
#define TF_TV_MOTION		0x1U	/* was TF_TV_HEADING (never set) */
```

Keep `M_CLASS` right after `T_STATE` so the Buddy 4 fields stay together.

### 2. `app_program/subsystem1/telemetry_map.c` (terrain block)

Buddy 4's bus slot (`include/terrain_status.h`): `values[0]` turn rate in
**rad/s**; `values[3]` = validity bits 0..2 | `T_STATE` << 3 | class << 6 |
motion-valid << 9 (`TERRAIN_MOTION_VALID`, 0x200).

```c
	flags = bits_of(&st->terrain, 3U);
	motion = (flags & TERRAIN_MOTION_VALID) != 0U;
	values[TF_F_T_VALID]   = (int32_t)((flags & (TF_TV_PITCH | TF_TV_HUMP)) |
				 (motion ? TF_TV_MOTION : 0U));
	values[TF_F_T_STATE]   = (int32_t)((flags >> 3) & 0x7U);
	values[TF_F_M_CLASS]   = motion ? (int32_t)((flags >> TERRAIN_MC_SHIFT) & 0x7U) : 0;
	values[TF_F_PITCH]     = ((flags & TF_TV_PITCH) != 0U) ?
				 to_int(st->terrain.values[1] * 180.0 / TM_PI) : 0;
	values[TF_F_TURN_RATE] = motion ? to_int(st->terrain.values[0] * 180.0 / TM_PI) : 0;
	values[TF_F_HUMP_MM]   = ((flags & TF_TV_HUMP) != 0U) ?
				 to_int(st->terrain.values[2] * 1000.0) : 0;
```

`#include "terrain_status.h"` gives `TERRAIN_MOTION_VALID` and
`TERRAIN_MC_SHIFT`. It is header-only and safe when Buddy 4 is disabled (the
slot stays unavailable, so everything sends 0). `tf_pack` already clamps
out-of-range values, so no extra range checks are needed.

### 3. Tests: `tests/buddy1_frame/test_frame.c`

- `golden_values()`: replace `v[TF_F_T_HEADING] = 0;` with
  `v[TF_F_M_CLASS] = 4; v[TF_F_TURN_RATE] = -46;`, and set
  `v[TF_F_T_VALID] = TF_TV_MOTION | TF_TV_PITCH | TF_TV_HUMP;`.
- **Regenerate `GOLDEN_HEX`**, as its comment says ("bump TF_LAYOUT_VERSION
  and regenerate both"), and update the same golden values in
  `tools/dashboard.py` (`GOLDEN` dict, lines 40–42).
- Terrain mapping check (around line 300): feed a slot with motion-valid, a
  class and a turn rate, and check `M_CLASS`, `TURN_RATE` (rad/s → deg/s) and
  `T_VALID` bit 0. Add one case **without** motion-valid that sends class 0 and
  rate 0.
- Add a round-trip for `TURN_RATE = -256, -1, +255` (the signed offset) and
  `HUMP_MM = 127`.

Note: this suite currently doesn't compile under clang because of an existing
`-Wenum-float-conversion` error at `test_frame.c:349`. That error is unrelated
to this change, but it needs fixing to run the suite.

### 4. Dashboard: `tools/dashboard.py`

The decoder reads the field table from the header, so new fields decode
automatically. The printed status line (lines 192–198) is hand-written, so
please add the Buddy 4 fields to it, e.g.:

```
... pitch=+1 hump= 11 terrain=CLIMBING motion=TURNING turn=-46 ...
```

- `terrain=`: name from `T_STATE` (INIT, WAITING, CLIMBING, DESCENDING, RECOVERY, FAULT, STOP, LIFTED)
- `motion=`: name from `M_CLASS`, only when `T_VALID` bit 0 is set, otherwise `-`
- `turn=`: `TURN_RATE` deg/s, only when `T_VALID` bit 0 is set

## What is not in the frame (serial console only)

- each turn's angle, duration and peak (`[TURN] right 47.0 deg in 1.2 s (peak 62 deg/s)`);
- the compass heading and field strength, and the compass angle per turn;
- each individual hump's peak and live hump height, roll, lift details (upright %, vertical mg), acceleration.

No new events: the event byte is full, and `TF_EV_HUMP_PEAK` is unchanged.

## Acceptance checks

1. `TF_FRAME_BYTES` is still 21, and `TF_FRAME_BITS` is 163.
2. `tests/buddy1_frame` passes with the regenerated golden frame, and
   `python tools/dashboard.py --selftest` passes.
3. On the robot (Wi-Fi build): the dashboard shows `motion=` changing
   STATIONARY → MOVING → TURNING, `turn=` negative on right turns and positive
   on left turns, and `hump=` after a hump.
4. A v1 dashboard rejects v2 frames (version mismatch) instead of misreading them.

## Buddy 4 side

Nothing further is needed from Buddy 4: the slot already publishes all of
this (`include/terrain_status.h`, `app_program/subsystem4/terrain_task.c`).
Questions to Buddy 4.
