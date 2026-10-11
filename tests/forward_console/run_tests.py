from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
out = root / "build/forward-console-host"
out.mkdir(parents=True, exist_ok=True)
exe = out / "test_forward_console.exe"
subprocess.run([
    "clang", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wconversion",
    # Existing shared parser resets ARM state even in profiles without ARM.
    "-Wno-unused-but-set-variable",
    "-DROBO_MANUAL_CONSOLE=1", "-DROBO_SAFE_MOTOR_TEST=1",
    "-DROBO_TEMPLATE_HAL_H", "-DTM_CONSOLE_USB_CDC=0",
    "-I", str(tests / "include"), "-I", str(root / "include"),
    str(root / "app_program/subsystem2/manual_console.c"),
    str(tests / "test_forward_console.c"), "-o", str(exe)
], check=True)
subprocess.run([str(exe)], check=True)
