"""Compile and run the actual subsystem with mocked kernel/I2C calls."""

from pathlib import Path
import subprocess


test_dir = Path(__file__).resolve().parent
project_root = test_dir.parents[2]
output_dir = project_root / "build" / "gy511-host"
output_dir.mkdir(parents=True, exist_ok=True)
executable = output_dir / "test_gy511.exe"

subprocess.run(
    [
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wpedantic",
        "-Wconversion", "-include", str(test_dir / "include/tk/device.h"),
        "-I", str(test_dir / "include"),
        str(project_root / "app_program/subsystem4/gy511_config.c"),
        str(test_dir / "test_gy511.c"), "-o", str(executable),
    ],
    check=True,
)
subprocess.run([str(executable)], check=True)

read_executable = output_dir / "test_imu_read.exe"
subprocess.run(
    [
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wpedantic",
        "-Wconversion", "-include", str(test_dir / "include/tk/device.h"),
        "-I", str(test_dir / "include"),
        str(project_root / "app_program/subsystem4/gy511_read.c"),
        str(test_dir / "test_imu_read.c"), "-o", str(read_executable),
    ],
    check=True,
)
subprocess.run([str(read_executable)], check=True)

terrain_executable = output_dir / "test_terrain.exe"
subprocess.run(
    [
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wpedantic",
        "-Wconversion", "-DTERRAIN_FORWARD_AXIS_Y=0",
        "-I", str(project_root / "app_program/subsystem4"),
        str(project_root / "app_program/subsystem4/terrain.c"),
        str(test_dir / "test_terrain.c"), "-lm", "-o", str(terrain_executable),
    ],
    check=True,
)
subprocess.run([str(terrain_executable)], check=True)

axes_executable = output_dir / "test_terrain_axes.exe"
subprocess.run(
    [
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wpedantic",
        "-Wconversion", "-I", str(project_root / "app_program/subsystem4"),
        str(project_root / "app_program/subsystem4/terrain.c"),
        str(test_dir / "test_terrain_axes.c"), "-lm", "-o",
        str(axes_executable),
    ],
    check=True,
)
subprocess.run([str(axes_executable)], check=True)

background_executable = output_dir / "test_background.exe"
subprocess.run([
    "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wpedantic",
    "-Wconversion", "-include", str(test_dir / "include/tk/device.h"),
    "-I", str(test_dir / "include"), "-I", str(project_root / "include"),
    "-I", str(project_root / "app_program/subsystem4"),
    str(project_root / "app_program/subsystem4/subsystem4.c"),
    str(project_root / "app_program/subsystem4/terrain.c"),
    str(test_dir / "test_background.c"), "-lm", "-o", str(background_executable),
], check=True)
subprocess.run([str(background_executable)], check=True)
