#!/usr/bin/env python3
"""SmartRobo dashboard: decode Buddy 1's binary MQTT frames and send commands.

The field table is read from include/telemetry_frame.h (the TF_FIELDS
X-macro), so the firmware and this decoder share one definition.

  python tools/dashboard.py selftest
  python tools/dashboard.py decode 1073...ec00      (status or encoder frame)
  python tools/dashboard.py listen --host 192.168.1.10 --csv run.csv
  python tools/dashboard.py send stop --host 192.168.1.10

listen/send need paho-mqtt (pip install paho-mqtt).
"""

import argparse
import csv
import os
import random
import re
import sys
import time

HEADER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "include", "telemetry_frame.h")

TOPIC_FRAME = "robot/frame"
TOPIC_ENCODER = "robot/encoder"
TOPIC_EVENT = "robot/event"
TOPIC_CMD = "robot/cmd"
TOPIC_ACK = "robot/cmd/ack"
TOPIC_STATUS = "robot/status"

OPCODES = {"start": 1, "stop": 2, "estop": 3}
ACK_RESULTS = ["ok", "duplicate", "busy", "rejected"]
EVENT_BITS = ["motion_fault", "line_lost", "barcode", "hump_peak",
              "obstacle", "scan_done", "handover", "cmd_rejected"]
# include/motion.h enums, in code order.
STATES = ["INITIALISING", "READY", "DRIVING", "STOPPING", "STOPPED", "FAULT", "SHUTDOWN"]
FAULTS = ["NONE", "CONFIG", "SETUP", "TIMING", "FEEDBACK", "DEADLINE", "EXPIRED",
          "STOP_TIMEOUT", "FINAL_GOAL", "EVENT_OVERFLOW", "SYSTEM", "ESTOP", "SETUP_TIMEOUT"]
FB_CAUSES = ["none", "unreliable", "invalid_transition", "odometry", "no_progress",
             "restart_latch"]

# Encoder frame (telemetry_frame.h, "Encoder frame"): 26 bytes, big-endian.
ENC_VERSION = 1
ENC_BYTES = 26
ENC_AGE_UNKNOWN = 0xFFFF
ENC_FIELDS = ["seq", "reliable", "trusted", "fresh", "cause", "state", "fault",
              "sample_ms", "age_ms", "count_l", "count_r", "invalid_l", "invalid_r"]
STALE_AFTER_S = 1.0     # no message for this long (dashboard clock) = stale

# Same golden frame as tests/buddy1_frame/test_frame.c.
GOLDEN_HEX = "10730394808dcbe69475b02a7e69c002a77201ec00"
GOLDEN = dict(VERSION=1, SEQ=7, UPTIME_DS=12345, M_STATE=2, M_MODE=2, M_FAULT=0,
              M_FLAGS=2, OWNER=1, CMD_ID=0xB9, SPEED_L=-1, SPEED_R=80,
              CENTRE_MM=-1234, HEADING=-170, LINE_AVAIL=1, LINE_ERR=-1,
              LINE_TRACK=1, BC_VALID=1, BC_CMD=3, T_VALID=6, T_STATE=3,
              PITCH=-12, T_HEADING=0, HUMP_MM=42, O_FLAGS=14, O_AVAIL=1,
              O_DIST_CM=400, O_WIDTH_CM=15, REJECTED=3, DROPPED=0)
# Same golden encoder frame as tests/buddy1_frame/test_frame.c.
GOLDEN_ENC_HEX = "010707200012d687000cfffffffe0001e2400000000000000003"
GOLDEN_ENC = dict(seq=7, reliable=1, trusted=1, fresh=1, cause=0, state=2, fault=0,
                  sample_ms=1234567, age_ms=12, count_l=0xFFFFFFFE, count_r=123456,
                  invalid_l=0, invalid_r=3)


def load_layout(path=HEADER):
    """Return (version, [(name, min, max, bits), ...]) from the C header."""
    text = open(path, encoding="utf-8").read()
    version = int(re.search(r"#define\s+TF_LAYOUT_VERSION\s+(\d+)", text).group(1))
    fields = [(m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4)))
              for m in re.finditer(
                  r"X\(\s*(\w+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*,\s*\w+\s*,\s*\"", text)]
    if not fields:
        raise SystemExit("no TF_FIELDS found in " + path)
    return version, fields


VERSION, FIELDS = load_layout()
FRAME_BITS = sum(f[3] for f in FIELDS)
FRAME_BYTES = (FRAME_BITS + 7) // 8


def encode(values):
    """values: dict name -> int.  Clamps like tf_pack()."""
    word = 0
    for name, lo, hi, bits in FIELDS:
        v = min(max(int(values.get(name, lo)), lo), hi)
        word = (word << bits) | (v - lo)
    word <<= FRAME_BYTES * 8 - FRAME_BITS
    return word.to_bytes(FRAME_BYTES, "big")


def decode(frame):
    """Return a dict of field values, or raise ValueError like tf_unpack()."""
    if len(frame) != FRAME_BYTES:
        raise ValueError("frame is %d bytes, expected %d" % (len(frame), FRAME_BYTES))
    word = int.from_bytes(frame, "big")
    spare = FRAME_BYTES * 8 - FRAME_BITS
    if word & ((1 << spare) - 1):
        raise ValueError("spare bits set")
    pos = FRAME_BYTES * 8
    out = {}
    for name, lo, hi, bits in FIELDS:
        pos -= bits
        code = (word >> pos) & ((1 << bits) - 1)
        if code > hi - lo:
            raise ValueError("%s code %d out of range" % (name, code))
        out[name] = code + lo   # add the minimum back: signed values restored
    if out["VERSION"] != VERSION:
        raise ValueError("frame version %d, decoder %d" % (out["VERSION"], VERSION))
    return out


def encode_encoder(v):
    """v: dict with ENC_FIELDS keys.  Mirrors tf_enc_pack() for in-range values."""
    flags = (1 if v["reliable"] else 0) | (2 if v["trusted"] else 0) | \
            (4 if v["fresh"] else 0) | (v["cause"] << 3)
    return (bytes([ENC_VERSION, v["seq"] & 0xFF, flags, (v["state"] << 4) | v["fault"]])
            + v["sample_ms"].to_bytes(4, "big") + v["age_ms"].to_bytes(2, "big")
            + b"".join(v[k].to_bytes(4, "big")
                       for k in ("count_l", "count_r", "invalid_l", "invalid_r")))


def decode_encoder(frame):
    """Return a dict of encoder frame values, or raise ValueError like tf_enc_unpack()."""
    if len(frame) != ENC_BYTES:
        raise ValueError("encoder frame is %d bytes, expected %d" % (len(frame), ENC_BYTES))
    if frame[0] != ENC_VERSION:
        raise ValueError("encoder frame version %d, decoder %d" % (frame[0], ENC_VERSION))
    if frame[2] & 0xC0:
        raise ValueError("encoder reserved bits set")
    cause, state, fault = (frame[2] >> 3) & 7, frame[3] >> 4, frame[3] & 0xF
    if cause >= len(FB_CAUSES) or state >= len(STATES) or fault >= len(FAULTS):
        raise ValueError("encoder code out of range")

    def u32(i):
        return int.from_bytes(frame[i:i + 4], "big")

    return dict(seq=frame[1], reliable=frame[2] & 1, trusted=(frame[2] >> 1) & 1,
                fresh=(frame[2] >> 2) & 1, cause=cause, state=state, fault=fault,
                sample_ms=u32(4), age_ms=int.from_bytes(frame[8:10], "big"),
                count_l=u32(10), count_r=u32(14), invalid_l=u32(18), invalid_r=u32(22))


def count_delta(now, before):
    """Signed change of a 32-bit wrapping counter (reverse and wrap safe)."""
    return ((now - before + 0x80000000) & 0xFFFFFFFF) - 0x80000000


def encoder_line(e, prev):
    """One readable line for an encoder frame; prev is the previous frame or None."""
    if prev is not None:
        dl = count_delta(e["count_l"], prev["count_l"])
        dr = count_delta(e["count_r"], prev["count_r"])
        di = count_delta(e["invalid_l"], prev["invalid_l"]) + \
            count_delta(e["invalid_r"], prev["invalid_r"])
        delta = " d=%+d,%+d new_invalid=%d" % (dl, dr, di)
    else:
        delta = ""
    age = "unknown" if e["age_ms"] == ENC_AGE_UNKNOWN else "%dms" % e["age_ms"]
    health = []
    if not e["fresh"]:
        health.append("STALE-SAMPLE")
    if not e["reliable"]:
        health.append("UNRELIABLE")
    if not e["trusted"]:
        health.append("UNTRUSTED")
    return ("ENC seq=%3d counts=%d,%d%s invalid=%d,%d state=%s fault=%s fb=%s age=%s %s"
            % (e["seq"], e["count_l"], e["count_r"], delta, e["invalid_l"], e["invalid_r"],
               STATES[e["state"]], FAULTS[e["fault"]], FB_CAUSES[e["cause"]], age,
               " ".join(health) or "ok"))


def encode_cmd(opcode, session, seq):
    word = ((opcode & 7) << 13) | ((session & 31) << 8) | (seq & 0xFF)
    return word.to_bytes(2, "big")


def decode_ack(data):
    if len(data) != 2:
        raise ValueError("ack is %d bytes" % len(data))
    word = int.from_bytes(data, "big")
    return ACK_RESULTS[word >> 14], (word >> 11) & 7, (word >> 3) & 0xFF


def event_names(mask):
    return [n for i, n in enumerate(EVENT_BITS) if mask & (1 << i)] or ["none"]


def selftest():
    failures = 0

    def check(cond, what):
        nonlocal failures
        if not cond:
            failures += 1
            print("FAIL", what)

    check(FRAME_BYTES == 21 and FRAME_BITS == 161, "21-byte frame, 161 bits")
    check(decode(bytes.fromhex(GOLDEN_HEX)) == GOLDEN, "golden frame decodes")
    check(encode(GOLDEN).hex() == GOLDEN_HEX, "golden frame encodes")
    rng = random.Random(1)
    for _ in range(20000):
        vals = {n: rng.randint(lo, hi) for n, lo, hi, _ in FIELDS}
        vals["VERSION"] = VERSION
        if decode(encode(vals)) != vals:
            check(False, "random round trip %r" % vals)
            break
    clamped = decode(encode(dict(GOLDEN, SPEED_R=999, PITCH=-1000)))
    check(clamped["SPEED_R"] == 250 and clamped["PITCH"] == -90, "clamp, not wrap")
    check(decode(encode(dict(GOLDEN, SPEED_L=-1)))["SPEED_L"] == -1, "-1 stays -1")
    bad = bytearray.fromhex(GOLDEN_HEX)
    bad[-1] |= 1
    try:
        decode(bytes(bad))
        check(False, "spare bit accepted")
    except ValueError:
        pass
    check(decode_encoder(bytes.fromhex(GOLDEN_ENC_HEX)) == GOLDEN_ENC, "golden encoder decodes")
    check(encode_encoder(GOLDEN_ENC).hex() == GOLDEN_ENC_HEX, "golden encoder encodes")
    for _ in range(2000):
        e = dict(seq=rng.randint(0, 255), reliable=rng.randint(0, 1), trusted=rng.randint(0, 1),
                 fresh=rng.randint(0, 1), cause=rng.randint(0, len(FB_CAUSES) - 1),
                 state=rng.randint(0, len(STATES) - 1), fault=rng.randint(0, len(FAULTS) - 1),
                 sample_ms=rng.getrandbits(32), age_ms=rng.getrandbits(16),
                 count_l=rng.getrandbits(32), count_r=rng.getrandbits(32),
                 invalid_l=rng.getrandbits(32), invalid_r=rng.getrandbits(32))
        if decode_encoder(encode_encoder(e)) != e:
            check(False, "encoder round trip %r" % e)
            break
    for bad_hex in (GOLDEN_ENC_HEX[:-2], "02" + GOLDEN_ENC_HEX[2:],
                    GOLDEN_ENC_HEX[:4] + "47" + GOLDEN_ENC_HEX[6:],
                    GOLDEN_ENC_HEX[:6] + "70" + GOLDEN_ENC_HEX[8:]):
        try:
            decode_encoder(bytes.fromhex(bad_hex))
            check(False, "bad encoder frame accepted: " + bad_hex)
        except ValueError:
            pass
    check(count_delta(1, 0xFFFFFFFE) == 3 and count_delta(0xFFFFFFFE, 1) == -3,
          "count deltas wrap both ways")
    check(encode_cmd(OPCODES["stop"], 9, 1) == bytes([0x49, 0x01]), "command layout")
    check(decode_ack(bytes([0x10, 0x28])) == ("ok", 2, 5), "ack layout")
    print("%s: dashboard selftest, %d failures" % ("PASS" if failures == 0 else "FAIL", failures))
    return 1 if failures else 0


def mqtt_client(client_id):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        raise SystemExit("needs paho-mqtt: pip install paho-mqtt")
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:      # paho-mqtt 1.x
        return mqtt.Client(client_id=client_id)


def listen(args):
    names = [f[0] for f in FIELDS]
    writer = None
    if args.csv:
        handle = open(args.csv, "w", newline="")
        writer = csv.writer(handle)
        writer.writerow(["recv_time"] + names)
    enc_writer = None
    if args.encoder_csv:
        enc_handle = open(args.encoder_csv, "w", newline="")
        enc_writer = csv.writer(enc_handle)
        enc_writer.writerow(["recv_time"] + ENC_FIELDS)
    state = {"seq": None, "frames": 0, "lost": 0, "bad": 0, "printed_at": 0.0, "since_print": 0,
             "enc": None, "enc_frames": 0, "enc_bad": 0, "enc_printed_at": 0.0,
             # Message ages use only this computer's clock (time.monotonic);
             # robot uptime is never compared with wall-clock time.
             "last_frame": None, "last_enc": None, "stale": set(), "robot": "unknown"}

    def on_connect(client, userdata, flags, rc, properties=None):
        client.subscribe([(TOPIC_FRAME, 0), (TOPIC_ENCODER, 0), (TOPIC_EVENT, 1),
                          (TOPIC_ACK, 1), (TOPIC_STATUS, 1)])
        print("connected; listening on robot/#")

    def on_message(client, userdata, msg):
        now = time.time()
        if msg.topic == TOPIC_ENCODER:
            try:
                e = decode_encoder(msg.payload)
            except ValueError as err:
                state["enc_bad"] += 1
                print("bad encoder frame (%s): %s" % (err, msg.payload.hex()))
                return
            state["last_enc"] = time.monotonic()
            state["enc_frames"] += 1
            if enc_writer:
                enc_writer.writerow(["%.3f" % now] + [e[k] for k in ENC_FIELDS])
            if not args.quiet and now - state["enc_printed_at"] >= args.every:
                state["enc_printed_at"] = now
                print(encoder_line(e, state["enc"]))
            state["enc"] = e
            return
        if msg.topic == TOPIC_FRAME:
            state["last_frame"] = time.monotonic()
            try:
                v = decode(msg.payload)
            except ValueError as e:
                state["bad"] += 1
                print("bad frame (%s): %s" % (e, msg.payload.hex()))
                return
            if state["seq"] is not None:
                state["lost"] += (v["SEQ"] - state["seq"] - 1) % 256
            state["seq"] = v["SEQ"]
            state["frames"] += 1
            if writer:
                writer.writerow(["%.3f" % now] + [v[n] for n in names])
            state["since_print"] += 1
            # Every frame is logged and counted; the console shows one every
            # --every seconds so it stays readable.
            if not args.quiet and now - state["printed_at"] >= args.every:
                state["printed_at"] = now
                if args.every > 0:
                    print("[%d frames in last %gs]" % (state["since_print"], args.every), end=" ")
                state["since_print"] = 0
                print("seq=%3d t=%6.1fs state=%s fault=%s speed=%+4d,%+4d mm/s centre=%+5d mm "
                      "hdg=%+4d line=%s bc=%s pitch=%+3d hump=%3d obst=%s %dcm lost=%d"
                      % (v["SEQ"], v["UPTIME_DS"] / 10.0, STATES[v["M_STATE"]],
                         FAULTS[v["M_FAULT"]],
                         v["SPEED_L"], v["SPEED_R"], v["CENTRE_MM"], v["HEADING"],
                         "%+d" % v["LINE_ERR"] if v["LINE_TRACK"] else "lost",
                         v["BC_CMD"] if v["BC_VALID"] else "-", v["PITCH"], v["HUMP_MM"],
                         bin(v["O_FLAGS"]), v["O_DIST_CM"], state["lost"]))
        elif msg.topic == TOPIC_EVENT and len(msg.payload) == 2:
            print("EVENT #%d: %s" % (msg.payload[1], ", ".join(event_names(msg.payload[0]))))
        elif msg.topic == TOPIC_ACK:
            try:
                # "ok" means queued for the controller, not executed; watch
                # M_STATE / the encoder line for the actual outcome.
                print("ACK %s (queued, not executed) opcode=%d seq=%d" % decode_ack(msg.payload))
            except ValueError as e:
                print("bad ack:", e)
        elif msg.topic == TOPIC_STATUS:
            state["robot"] = msg.payload.decode("ascii", "replace")
            print("ROBOT %s" % state["robot"].upper())
        else:
            print("%s %r" % (msg.topic, msg.payload))

    def check_stale():
        """Warn once when a stream stops, and once when it comes back."""
        mono = time.monotonic()
        for name, key in (("status frames", "last_frame"), ("encoder frames", "last_enc")):
            last = state[key]
            age = None if last is None else mono - last
            stale = age is None or age > STALE_AFTER_S
            if stale and name not in state["stale"]:
                state["stale"].add(name)
                print("STALE: no %s for %s (robot %s)"
                      % (name, "ever" if age is None else "%.1fs" % age, state["robot"]))
            elif not stale and name in state["stale"]:
                state["stale"].discard(name)
                print("LIVE: %s resumed" % name)

    client = mqtt_client("smartrobo-dash-%04x" % random.getrandbits(16))
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port)
    client.loop_start()
    start = time.monotonic()
    try:
        while True:
            time.sleep(0.25)
            if time.monotonic() - start > STALE_AFTER_S:
                check_stale()
    except KeyboardInterrupt:
        pass
    client.loop_stop()
    print("frames=%d lost=%d bad=%d encoder_frames=%d encoder_bad=%d"
          % (state["frames"], state["lost"], state["bad"], state["enc_frames"], state["enc_bad"]))


def send(args):
    session = args.session if args.session is not None else random.getrandbits(5)
    payload = encode_cmd(OPCODES[args.command], session, args.seq)
    client = mqtt_client("smartrobo-cmd-%04x" % random.getrandbits(16))
    client.connect(args.host, args.port)
    client.loop_start()
    info = client.publish(TOPIC_CMD, payload, qos=1)
    info.wait_for_publish(5)
    client.loop_stop()
    client.disconnect()
    print("sent %s session=%d seq=%d bytes=%s" % (args.command, session, args.seq, payload.hex()))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("selftest")
    d = sub.add_parser("decode")
    d.add_argument("hex")
    for name in ("listen", "send"):
        s = sub.add_parser(name)
        s.add_argument("--host", required=True)
        s.add_argument("--port", type=int, default=1883)
        if name == "listen":
            s.add_argument("--csv")
            s.add_argument("--encoder-csv", help="log every encoder frame to this CSV")
            s.add_argument("--quiet", action="store_true")
            s.add_argument("--every", type=float, default=5.0,
                           help="seconds between printed frames (0 = print every frame)")
        else:
            s.add_argument("command", choices=sorted(OPCODES))
            s.add_argument("--session", type=int)
            s.add_argument("--seq", type=int, default=0)
    args = p.parse_args()
    if args.cmd == "selftest":
        return selftest()
    if args.cmd == "decode":
        raw = bytes.fromhex(args.hex)
        # Status (21 bytes) and encoder (26 bytes) frames differ in length.
        values = decode_encoder(raw) if len(raw) == ENC_BYTES else decode(raw)
        for k, v in values.items():
            print("%-11s %d" % (k, v))
        return 0
    if args.cmd == "listen":
        listen(args)
    else:
        send(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
