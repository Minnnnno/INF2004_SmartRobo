#!/usr/bin/env python3
"""SmartRobo dashboard: decode Buddy 1's binary MQTT frames and send commands.

The field table is read from include/telemetry_frame.h (the TF_FIELDS
X-macro), so the firmware and this decoder share one definition.

  python tools/dashboard.py selftest
  python tools/dashboard.py decode 1073...ec00
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
TOPIC_EVENT = "robot/event"
TOPIC_CMD = "robot/cmd"
TOPIC_ACK = "robot/cmd/ack"
TOPIC_STATUS = "robot/status"

OPCODES = {"start": 1, "stop": 2, "estop": 3}
ACK_RESULTS = ["ok", "duplicate", "busy", "rejected"]
EVENT_BITS = ["motion_fault", "line_lost", "barcode", "hump_peak",
              "obstacle", "scan_done", "handover", "cmd_rejected"]

# Same golden frame as tests/buddy1_frame/test_frame.c.
GOLDEN_HEX = "10730394808dcbe69475b02a7e69c002a77201ec00"
GOLDEN = dict(VERSION=1, SEQ=7, UPTIME_DS=12345, M_STATE=2, M_MODE=2, M_FAULT=0,
              M_FLAGS=2, OWNER=1, CMD_ID=0xB9, SPEED_L=-1, SPEED_R=80,
              CENTRE_MM=-1234, HEADING=-170, LINE_AVAIL=1, LINE_ERR=-1,
              LINE_TRACK=1, BC_VALID=1, BC_CMD=3, T_VALID=6, T_STATE=3,
              PITCH=-12, T_HEADING=0, HUMP_MM=42, O_FLAGS=14, O_AVAIL=1,
              O_DIST_CM=400, O_WIDTH_CM=15, REJECTED=3, DROPPED=0)


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
    state = {"seq": None, "frames": 0, "lost": 0, "bad": 0}

    def on_connect(client, userdata, flags, rc, properties=None):
        client.subscribe([(TOPIC_FRAME, 0), (TOPIC_EVENT, 1), (TOPIC_ACK, 1), (TOPIC_STATUS, 1)])
        print("connected; listening on robot/#")

    def on_message(client, userdata, msg):
        now = time.time()
        if msg.topic == TOPIC_FRAME:
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
            if not args.quiet:
                print("seq=%3d t=%6.1fs state=%d fault=%d speed=%+4d,%+4d mm/s centre=%+5d mm "
                      "hdg=%+4d line=%s bc=%s pitch=%+3d hump=%3d obst=%s %dcm lost=%d"
                      % (v["SEQ"], v["UPTIME_DS"] / 10.0, v["M_STATE"], v["M_FAULT"],
                         v["SPEED_L"], v["SPEED_R"], v["CENTRE_MM"], v["HEADING"],
                         "%+d" % v["LINE_ERR"] if v["LINE_TRACK"] else "lost",
                         v["BC_CMD"] if v["BC_VALID"] else "-", v["PITCH"], v["HUMP_MM"],
                         bin(v["O_FLAGS"]), v["O_DIST_CM"], state["lost"]))
        elif msg.topic == TOPIC_EVENT and len(msg.payload) == 2:
            print("EVENT #%d: %s" % (msg.payload[1], ", ".join(event_names(msg.payload[0]))))
        elif msg.topic == TOPIC_ACK:
            try:
                print("ACK %s opcode=%d seq=%d" % decode_ack(msg.payload))
            except ValueError as e:
                print("bad ack:", e)
        else:
            print("%s %r" % (msg.topic, msg.payload))

    client = mqtt_client("smartrobo-dash-%04x" % random.getrandbits(16))
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        pass
    print("frames=%d lost=%d bad=%d" % (state["frames"], state["lost"], state["bad"]))


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
            s.add_argument("--quiet", action="store_true")
        else:
            s.add_argument("command", choices=sorted(OPCODES))
            s.add_argument("--session", type=int)
            s.add_argument("--seq", type=int, default=0)
    args = p.parse_args()
    if args.cmd == "selftest":
        return selftest()
    if args.cmd == "decode":
        for k, v in decode(bytes.fromhex(args.hex)).items():
            print("%-11s %d" % (k, v))
        return 0
    if args.cmd == "listen":
        listen(args)
    else:
        send(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
