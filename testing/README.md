# Testing

All test records for the project live in this folder. A test only counts as
done when it is **written down here with its result and a saved log**.

```
testing/
├── README.md              this guide
├── subsystem1/            one folder per subsystem, kept by its owner
│   ├── TEST_LOG.md        test cases + results
│   └── logs/              saved console output, photos, scope captures
├── ...
├── subsystem5/
├── integration/           tests across two or more subsystems (shared)
└── system/                tests of the whole product against its requirements (shared)
```

## What is in each folder

| Folder | Covers | Owner | Contents besides `TEST_LOG.md` and `logs/` |
|---|---|---|---|
| `subsystem1/` | Wi-Fi, MQTT, commands, telemetry frame and web dashboard | Buddy 1 (2500842) | `logs/2026-11-17_console.txt`: serial capture of the 17 November dashboard session (image before the Start gate) |
| `subsystem2/` | Motion control: motors, encoders, wheel-speed PID, odometry | Buddy 2 | `BUDDY2_COMPACT_REPRESENTATION.md`: minimum-bit representation and measured memory |
| `subsystem3/` | Line following and barcode navigation | Buddy 3 (2500622) | `BUDDY3_COMPACT_REPRESENTATION.md`: minimum-bit representation, measured RAM and message sizes; `logs/compact_representation_2026-11-25/`: its evidence |
| `subsystem4/` | IMU tilt, hump height, motion class, lift/overturn stop | Buddy 4 (2500910) | `BUDDY4_COMPACT_REPRESENTATION.md`: minimum-bit representation, measured RAM and message sizes; `logs/compact_representation_2026-11-25/`: its evidence |
| `subsystem5/` | Obstacle detection, width estimation and avoidance | Buddy 5 (2500344) | — |
| `integration/` | Interfaces between two or more subsystems (`INT-..`) | the owners on both sides | — |
| `system/` | The whole robot against the mission requirements (`SYS-..`) | whole team | `com4_2026-11-0*.log`: Buddy 2's raw serial captures from 3–4 November debugging, cited by `docs/` investigation notes, not test results |

Each `TEST_LOG.md` has the same parts: requirements, test cases (laptop and
robot), and a results table. Robot tests are evidenced by videos; every video
is listed in the **video index, [`videos/README.md`](../videos/README.md)**,
and linked from the result row it supports.

## Three levels of test

| Level | ID format | Tests what | Written by | Where |
|---|---|---|---|---|
| Unit | `S1-U01`, `S2-U01`, … | one subsystem on its own | the subsystem owner | `subsystemN/TEST_LOG.md` |
| Integration | `INT-01`, … | an interface between subsystems (`docs/API.md`) | the owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-01`, … | the finished product against a requirement | whole team | `system/TEST_LOG.md` |

Test IDs never change and are never reused, even if a test is dropped.

## How to run and record a test

1. **Define it first.** Add a row to the *Test cases* table in the right
   `TEST_LOG.md`: ID, what it checks, how, and the expected result.
2. **Automate it if you can, as a host test on the laptop.** Host tests run the
   real subsystem code with the hardware faked, so they need no robot:
   - team suite (all subsystems' portable logic, under `tests/`):
     ```sh
     cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
     cmake --build build-host-cmake
     ctest --test-dir build-host-cmake --output-on-failure
     ```
     On Windows with MSYS2, add `-DCMAKE_C_COMPILER=clang`: MinGW `gcc` reports
     false float-conversion errors under the team's `-Werror` flags.
   - subsystem-specific suites, if your subsystem has one (e.g.
     `python tests/subsystem4/run_tests.py`, or Buddy 1's dashboard check
     `python tests/subsystem1/web_dashboard_check.py`).

   Each `TEST_LOG.md` lists the exact commands for its tests.
3. **Otherwise it is a robot test.** A test that needs eyes or instruments (a
   motor turns, a sensor reacts, the dashboard shows a value) is a *manual* test:
   write its steps and expected result in the table, build the normal firmware
   (VS Code "Compile Project", `build/mtkernel_3.uf2`), flash it with motor power
   off, and record a video (see below).
4. **Save the evidence** to `logs/`, named `YYYY-MM-DD_<what>.txt`:
   - host tests: redirect the output, e.g.
     `python tests/subsystem4/run_tests.py > testing/subsystem4/logs/2026-11-25_host.txt`;
   - robot console: in PuTTY use *Session → Logging → All session output*
     (on macOS/Linux, `cat /dev/cu.usbmodem*` or `/dev/ttyACM0` piped to `tee`);
   - dashboard: start `tools/web_dashboard.py` with `--log` / `--csv`.

   Keep each file under a few MB. Videos go on YouTube instead (see below).
5. **Record the result.** Add a row to the *Results* table: date, test ID,
   the commit you tested (`git rev-parse --short HEAD`), who ran it,
   PASS/FAIL, and the log file. Commit the log and the table together.

## Videos of tests and demos

A manual test (a motor, a sensor reacting, the dashboard showing a value) is
best shown with a short video. Record it showing the test ID and the hardware,
share it as an **unlisted YouTube** link (the only way videos are shared), add
it to the video index in [`videos/README.md`](../videos/README.md), and put the
link in the *Log / evidence* column of the test's result row. The video index
explains how.

## Rules

- **Never edit or delete an old result.** If a test fails, record the FAIL,
  fix the code, re-run, and add a new PASS row. The history is the evidence.
- **Always record the commit.** A result is meaningless without the exact
  code it was run on. Commit your work *before* testing.
- **A FAIL needs a note** saying what went wrong and, later, which commit fixed it.
- Before merging a change into `main`, re-run your subsystem's unit tests.
  Before a demo or hand-in, run every test and record the full set.
