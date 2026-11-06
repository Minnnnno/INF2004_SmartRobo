"""Compile and run Buddy 4 with mocked kernel/I2C calls (stage 1: IMU, tilt, terrain).

Heading and magnetic calibration are not integrated yet, so their suites are
not part of this runner. Usage: python testing/subsystem4/host/run_tests.py
"""

from pathlib import Path
import subprocess
import sys

test_dir = Path(__file__).resolve().parent
project_root = test_dir.parents[2]
source_dir = project_root / "app_program/subsystem4"
output_dir = project_root / "build" / "gy511-host"
output_dir.mkdir(parents=True, exist_ok=True)

STRICT = ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-Wconversion"]
# terrain_task.c includes the real team headers; use the repository's host flags.
TEAM = ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wconversion", "-Wshadow"]
STUBS = ["-I", str(test_dir / "include")]


def build_and_run(name, flags, sources, extra=()):
    executable = output_dir / f"{name}.exe"
    subprocess.run(
        flags + list(extra) + [str(s) for s in sources] + ["-lm", "-o", str(executable)],
        check=True,
    )
    subprocess.run([str(executable)], check=True)


build_and_run("test_gy511", STRICT,
              [source_dir / "subsystem4.c", test_dir / "test_gy511.c"],
              ["-include", str(test_dir / "include/tk/device.h")] + STUBS)
build_and_run("test_acceleration", STRICT,
              [source_dir / "gy511_read.c", test_dir / "test_acceleration.c"], STUBS)
build_and_run("test_magnetic", STRICT,
              [source_dir / "gy511_read.c", test_dir / "test_magnetic.c"], STUBS)
build_and_run("test_tilt", STRICT,
              [source_dir / "tilt.c", test_dir / "test_tilt.c"])
build_and_run("test_terrain", STRICT,
              [source_dir / "terrain.c", source_dir / "tilt.c", test_dir / "test_terrain.c"])
build_and_run("test_buddy4", STRICT,
              [source_dir / "buddy4.c", source_dir / "terrain.c", source_dir / "tilt.c",
               test_dir / "test_buddy4.c"], STUBS)
build_and_run("test_terrain_task", TEAM,
              [source_dir / "buddy4.c", source_dir / "terrain.c", source_dir / "tilt.c",
               test_dir / "test_terrain_task.c"],
              STUBS + ["-I", str(project_root / "include")])

# Exercise the production I2C interrupt receive arm with mocked register accesses.
subprocess.run([sys.executable, str(test_dir / "test_i2c_receive.py")], check=True)
print("All Buddy 4 host suites passed.")
