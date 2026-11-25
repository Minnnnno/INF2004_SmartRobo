"""Measure Buddy 4's minimum-bit representation (Change Requirement 1, Week 13).

Usage (from the repository root, after building the firmware):
    python tests/subsystem4/measure_compact.py <firmware build dir> <output dir>

1. RAM per structure: compiles tests/subsystem4/memory_probe.c with the Pico's
   arm-none-eabi-gcc and the firmware's own flags, and reads every object's
   size with arm-none-eabi-nm -S (real ARM sizes, padding included).
2. Buddy 4's RAM in the firmware: arm-none-eabi-size on the firmware's
   subsystem4 object files.
3. Message sizes: the three MQTT JSON messages the first Buddy 4 (b96a929)
   sent, against Buddy 4's fields in the binary status frame (v1 and now).
Writes sizes.txt and messages.txt to the output directory and prints both.
"""

from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TOOLS = Path.home() / ".pico-sdk" / "toolchain" / "15_2_Rel1" / "bin"
GCC = str(TOOLS / "arm-none-eabi-gcc")
NM = str(TOOLS / "arm-none-eabi-nm")
SIZE = str(TOOLS / "arm-none-eabi-size")
# Same code-generation flags as the firmware build (build.ninja for subsystem4).
FIRMWARE_FLAGS = ["-mcpu=cortex-m0plus", "-mthumb", "-mfloat-abi=soft",
                  "-ffreestanding", "-O2", "-std=gnu11"]
V1_FRAME_COMMIT = "e564fd7"  # last commit with telemetry frame layout v1
JSON_RATE_HZ = 4             # b96a929: S4_REPORT_S 0.25 s
FRAME_RATE_HZ = 10           # comms.c: COMMS_STATUS_MS 100

PAIRS = [  # (what, before symbol, after symbol, after representation)
    ("Raw GY-511 sample (acc + mag, 3 axes each)", "before_sample", "after_sample",
     "2 x 5-byte packed 3 x 12-bit"),
    ("Tilt result (pitch, roll, conditions, valid)", "before_tilt_result", "after_tilt_result",
     "27 bits in one uint32_t"),
    ("Lifecycle control (state, attempts, phase)", "before_control", "after_control",
     "3+2+2-bit field, 1 byte"),
    ("Hump results (last peak + side, run max + valid)", "before_hump", "after_hump",
     "7-bit mm + 1 flag bit, x2"),
    ("Hump detector phase + 4 flags", "before_terrain_flags", "after_terrain_flags",
     "2-bit phase + flags, 1 byte"),
    ("Tilt capture progress + 2 conditions", "before_tilt_progress", "after_tilt_progress",
     "6-bit count + 2 flags; 2+2-bit conditions"),
    ("Motion class debounce (reported, candidate, count)", "before_class_state", "after_class_state",
     "3+3+2 bits, 1 byte"),
    ("Compass validity, 5 heading slots", "before_compass_window", "after_compass_window",
     "5-bit mask"),
    ("Lift latch (condition + 3 timing flags)", "before_lift_latch", "after_lift_latch",
     "2-bit condition + 3 flags, 1 byte"),
]
WHOLE = ["whole_buddy4", "whole_motion_class", "whole_tilt", "whole_terrain", "whole_lift"]


def run(args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def probe_sizes():
    with tempfile.TemporaryDirectory() as tmp:
        obj = Path(tmp) / "memory_probe.o"
        run([GCC] + FIRMWARE_FLAGS + ["-Wall", "-Wextra", "-Werror", "-c",
             str(ROOT / "tests/subsystem4/memory_probe.c"),
             "-I", str(ROOT / "tests/subsystem4/include"),
             "-I", str(ROOT / "include"),
             "-I", str(ROOT / "app_program/subsystem4"),
             "-o", str(obj)])
        sizes = {}
        for line in run([NM, "-S", str(obj)]).splitlines():
            parts = line.split()
            if len(parts) == 4:
                sizes[parts[3]] = int(parts[1], 16)
        return sizes


def ram_report(fw_dir):
    s = probe_sizes()
    out = ["RAM per structure (arm-none-eabi-gcc 15.2, firmware flags: "
           + " ".join(FIRMWARE_FLAGS) + ")",
           "before = b96a929 type or plain unpacked type; after = current header type",
           "",
           f"{'Data':52} {'before B':>8} {'after B':>8} {'saved B':>8}  after representation"]
    total_b = total_a = 0
    for what, b, a, rep in PAIRS:
        total_b += s[b]
        total_a += s[a]
        out.append(f"{what:52} {s[b]:8} {s[a]:8} {s[b] - s[a]:8}  {rep}")
    out.append(f"{'Total':52} {total_b:8} {total_a:8} {total_b - total_a:8}  "
               f"({100.0 * (total_b - total_a) / total_b:.0f} % smaller)")
    out += ["", "Today's whole Buddy 4 structures (bytes):"]
    out += [f"  {name[6:]:14} {s[name]:5}" for name in WHOLE]

    objs = sorted((Path(fw_dir) / "CMakeFiles/mtkernel_3.dir/app_program/subsystem4").glob("*.obj"))
    out += ["", "Buddy 4 in the production firmware (arm-none-eabi-size on its object files):"]
    data = bss = 0
    for line in run([SIZE] + [str(o) for o in objs]).splitlines()[1:]:
        parts = line.split()
        data += int(parts[1])
        bss += int(parts[2])
        out.append(f"  {Path(parts[5]).name:28} text {parts[0]:>6}  data {parts[1]:>4}  bss {parts[2]:>5}")
    out.append(f"  Buddy 4 static RAM = data {data} + bss {bss} = {data + bss} bytes "
               "(+ 4096-byte task stack, app_main.c)")
    elf = Path(fw_dir) / "mtkernel_3.elf"
    out += ["", "Whole firmware image:", "  " + run([SIZE, str(elf)]).strip().replace("\n", "\n  ")]
    return out


def json_bytes(fmt, values):
    return len(fmt % values)


def mqtt_packet(topic, payload_len):
    """QoS 0 PUBLISH: fixed header 1 + remaining length (1-2) + topic length 2 + topic + payload."""
    remaining = 2 + len(topic) + payload_len
    return 1 + (1 if remaining < 128 else 2) + remaining


def frame_fields(text):
    rows = re.findall(r"X\((\w+),\s*(-?\d+),\s*(-?\d+),\s*(\d+),\s*B4", text)
    return [(n, int(lo), int(hi), int(bits)) for n, lo, hi, bits in rows]


def message_report():
    xyz = '{"x":%d,"y":%d,"z":%d}'
    ter = '{"p":%d,"r":%d,"t":%d,"m":%d,"f":%d,"s":%d,"c":%d,"v":%d}'
    msgs = [  # topic, format, shortest values, longest values
        ("robot/telemetry/imu_acc", xyz, (0, 0, 0), (-2048, -2048, -2048)),
        ("robot/telemetry/imu_mag", xyz, (0, 0, 0), (-2048, -2048, -2048)),
        # pitch/roll -180..180, tilt 0..180 (int degrees); motion mg up to 5 chars;
        # conditions 0..9; valid 0/1.
        ("robot/telemetry/terrain", ter, (0, 0, 0, 0, 0, 0, 0, 0),
         (-180, -180, 180, -1000, 9, 9, 9, 1)),
    ]
    out = ["BEFORE (b96a929): three MQTT JSON text messages every 0.25 s (4 Hz)",
           f"{'Topic':26} {'payload B':>12} {'MQTT packet B':>14}"]
    pay = [0, 0]
    pkt = [0, 0]
    for topic, fmt, lo, hi in msgs:
        p = (json_bytes(fmt, lo), json_bytes(fmt, hi))
        k = (mqtt_packet(topic, p[0]), mqtt_packet(topic, p[1]))
        pay = [pay[0] + p[0], pay[1] + p[1]]
        pkt = [pkt[0] + k[0], pkt[1] + k[1]]
        out.append(f"{topic:26} {p[0]:5} .. {p[1]:4} {k[0]:6} .. {k[1]:4}")
    out.append(f"{'Per 0.25 s report':26} {pay[0]:5} .. {pay[1]:4} {pkt[0]:6} .. {pkt[1]:4}")
    out.append(f"{'Per second (x4)':26} {4 * pay[0]:5} .. {4 * pay[1]:4} "
               f"{4 * pkt[0]:6} .. {4 * pkt[1]:4}   = {8 * 4 * pay[0]}..{8 * 4 * pay[1]} payload bits/s")

    now = frame_fields((ROOT / "include/telemetry_frame.h").read_text())
    v1 = frame_fields(run(["git", "-C", str(ROOT), "show",
                           f"{V1_FRAME_COMMIT}:include/telemetry_frame.h"]))
    bits_now = sum(f[3] for f in now)
    bits_v1 = sum(f[3] for f in v1)
    frame_pkt = mqtt_packet("robot/frame", 21)
    out += ["",
            "AFTER: Buddy 4 fields inside the shared binary status frame "
            f"(robot/frame, 21 bytes, {FRAME_RATE_HZ} Hz)",
            f"{'Field':12} {'range':>12} {'bits':>5}   v1 ({V1_FRAME_COMMIT})"]
    v1_map = {f[0]: f for f in v1}
    for name, lo, hi, bits in now:
        old = v1_map.get(name) or (v1_map.get("T_HEADING") if name == "TURN_RATE" else None)
        old_txt = f"{old[0]} {old[1]}..{old[2]} {old[3]} bits" if old else "(new in v2)"
        out.append(f"{name:12} {f'{lo}..{hi}':>12} {bits:5}   {old_txt}")
    out += [f"Buddy 4 bits per frame: v1 {bits_v1}, now {bits_now} "
            f"(= {bits_now / 8:.3f} bytes; v2 added M_CLASS, reused T_HEADING's bits as "
            "TURN_RATE and trimmed HUMP_MM 8 -> 7 bits; frame still 21 bytes)",
            f"Buddy 4 payload per second: {bits_now} x {FRAME_RATE_HZ} = {bits_now * FRAME_RATE_HZ} bits/s",
            f"Whole frame packet (all buddies): {frame_pkt} bytes per 100 ms",
            "",
            "Comparison per second (payload only, Buddy 4's data):",
            f"  before {8 * 4 * pay[0]}..{8 * 4 * pay[1]} bits/s at 4 Hz  ->  now "
            f"{bits_now * FRAME_RATE_HZ} bits/s at 10 Hz "
            f"({(8 * 4 * pay[0]) / (bits_now * FRAME_RATE_HZ):.1f}x..{(8 * 4 * pay[1]) / (bits_now * FRAME_RATE_HZ):.1f}x less, "
            "while sending 2.5x as often)",
            f"  one update: before {pay[0]}..{pay[1]} bytes of JSON in 3 messages  ->  "
            f"now {bits_now} bits ({bits_now / 8:.3f} bytes) in the shared frame"]
    return out


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    fw_dir, out_dir = Path(sys.argv[1]), Path(sys.argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, lines in (("sizes.txt", ram_report(fw_dir)), ("messages.txt", message_report())):
        text = "\n".join(lines) + "\n"
        (out_dir / name).write_text(text, encoding="utf-8")
        print(text)


if __name__ == "__main__":
    main()
