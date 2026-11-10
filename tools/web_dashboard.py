#!/usr/bin/env python3
"""SmartRobo web dashboard: live telemetry, a troubleshooting checklist, an
event log and Start / Stop / E-STOP buttons, in a browser.

Run it on the laptop that hosts the MQTT broker (or any machine that can reach
the broker); anyone on the same network opens the page:

  python tools/web_dashboard.py --host 127.0.0.1               # broker on this laptop
  python tools/web_dashboard.py --host 127.0.0.1 --log logs/run.log --csv logs/frames.csv
  python tools/web_dashboard.py --demo                          # simulated robot, no broker

then open http://<this laptop's IP>:8080 (the startup line prints the address).

Needs paho-mqtt (pip install paho-mqtt), except for --demo.  Frame decoding is
shared with tools/dashboard.py, which reads include/telemetry_frame.h.

What the buttons do (see docs/BUDDY1_HANDOFF_README.md, "Start, stop and
fault handling"):
  STOP    controlled stop; motors stop and movement is held.  Wi-Fi stays up.
  START   releases a held robot.  The controller only accepts it when the robot
          is STOPPED, fault-free, stationary and has trusted encoder feedback.
  E-STOP  latches an emergency-stop fault.  START cannot clear it: it needs the
          on-robot checked reset/release.
None of them power the Pico off; the page stays connected so the robot can be
started again.
"""

import argparse
import collections
import csv
import json
import os
import random
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dashboard as d  # noqa: E402  (shared decoder)

TOPIC_HEARTBEAT = "robot/heartbeat"
TOPIC_TEXT = "robot/telemetry/"
HEARTBEAT_STALE_S = 3.0      # heartbeat is sent every 1 s
START_CHECK_S = 2.5          # time for a START to clear the hold flag

MODES = ["DISTANCE_MOVE", "ENCODER_TURN", "WHEEL_SPEED_TRACKING", "MANUAL_OUTPUT", "ARC_MOVE"]
OWNERS = ["manual", "line", "barcode", "obstacle", "wifi", "terrain", "?", "none"]
BARCODES = ["left", "right", "straight", "U-turn"]
# Buddy 4: T_STATE codes (app_program/subsystem4/README.md, "Interfaces").
T_STATES = ["INIT", "WAITING", "CLIMBING", "DESCENDING", "RECOVERY", "FAULT", "STOP"]
# Buddy 4: M_CLASS codes, only sent once the layout v2 change request is applied.
M_CLASSES = ["STATIONARY", "MOVING", "ACCELERATING", "REVERSING", "TURNING",
             "CLIMBING", "DESCENDING", "IMPACT"]
# Buddy 5 publishes S5_STATE_* (subsystem5.h) in obstacle values[3]; the
# frame mapper copies that into O_FLAGS, so O_FLAGS currently holds this state.
S5_STATES = ["clear", "held / scanning", "pass left", "pass right", "no way around",
             "driving round it", "detour abandoned"]

# From the [ERROR] lines in app_program/subsystem2/motion_runtime.c.
FAULT_HELP = {
    "CONFIG": "Invalid motion configuration or start time. Check the firmware profile "
              "(hardware_confirmed is only true in production builds) and reflash.",
    "SETUP": "Motor/PWM/encoder/GPIO initialisation failed. Check wiring and motor power, "
             "then power-cycle.",
    "TIMING": "The 10 ms motion loop ran late or with a bad interval. Something is "
              "starving the motion task.",
    "FEEDBACK": "Encoder feedback cannot be trusted (see the encoder cause). Check encoder "
                "wiring and that the wheel turns freely. Needs the checked reset on the robot.",
    "DEADLINE": "A move did not finish in time: wheel stalled, blocked or battery low.",
    "EXPIRED": "A continuous wheel-speed command was not refreshed within ~300 ms: the "
               "steering task stalled or stopped sending.",
    "STOP_TIMEOUT": "The robot did not confirm it was stationary after stopping "
                    "(wheels still turning or encoder noise).",
    "FINAL_GOAL": "The final distance or heading check failed after a move.",
    "EVENT_OVERFLOW": "Motion event buffer filled: notifications were not drained in time.",
    "SYSTEM": "Internal system fault: motion synchronisation or command decoding failure.",
    "ESTOP": "Emergency stop from the dashboard. START will not clear it: do the checked "
             "reset/release on the robot.",
    "SETUP_TIMEOUT": "Start-up checks took too long (robot moved or encoders not ready at boot).",
}
FB_HELP = {
    "unreliable": "Encoder readings unavailable or unreliable.",
    "invalid_transition": "Invalid A/B transitions: check encoder wiring and signal quality.",
    "odometry": "Invalid movement measurement.",
    "no_progress": "Wheel commanded to move but its encoder count did not change: "
                   "stalled wheel, motor power or encoder wiring.",
    "restart_latch": "Encoder fault kept from before a restart: manual checks and "
                     "checked reset/release needed.",
}
EVENT_LEVEL = {"motion_fault": "error", "line_lost": "warn", "cmd_rejected": "warn"}
EVENT_TEXT = {
    "motion_fault": "motion fault raised", "line_lost": "line lost",
    "barcode": "barcode decoded", "hump_peak": "hump completed",
    "obstacle": "obstacle detected", "scan_done": "obstacle scan finished",
    "handover": "obstacle passed, handed back to the line follower",
    "cmd_rejected": "robot rejected a malformed command",
}


def ack_bytes(result, opcode, seq):
    """Inverse of dashboard.decode_ack (used by the simulator)."""
    return ((result << 14) | ((opcode & 7) << 11) | ((seq & 0xFF) << 3)).to_bytes(2, "big")


class Hub:
    """Everything the page shows.  MQTT callbacks, the HTTP threads and the
    stale-check ticker all go through self.lock."""

    def __init__(self, args):
        self.lock = threading.Lock()
        self.boot = "%08x" % random.getrandbits(32)
        self.broker_name = "demo" if args.demo else "%s:%d" % (args.host, args.mqtt_port)
        self.broker = "connecting"
        self.broker_at = time.monotonic()
        self.robot = "unknown"
        self.frame = self.frame_at = None
        self.enc = self.enc_prev = self.enc_at = None
        self.hb = self.hb_at = None
        self.obstacle_text = {}
        self.counts = dict(frames=0, lost=0, bad=0, enc_frames=0, enc_bad=0, events=0)
        self.last_bad = None            # (monotonic, message)
        self.seq = None
        self.stale = set()
        self.invalid_logged_at = 0.0
        self.session = random.getrandbits(5)
        self.cmd_seq = random.getrandbits(8)
        self.publish = None             # set by the transport: payload -> bool
        self.log = collections.deque(maxlen=3000)
        self.log_id = 0
        self.log_file = self._open(args.log, "a")
        self.frame_csv = self.enc_csv = None
        if args.csv:
            self.frame_csv = csv.writer(self._open(args.csv, "w", newline=""))
            self.frame_csv.writerow(["recv_time"] + [f[0] for f in d.FIELDS])
        if args.encoder_csv:
            self.enc_csv = csv.writer(self._open(args.encoder_csv, "w", newline=""))
            self.enc_csv.writerow(["recv_time"] + d.ENC_FIELDS)

    @staticmethod
    def _open(path, mode, **kw):
        if not path:
            return None
        folder = os.path.dirname(os.path.abspath(path))
        os.makedirs(folder, exist_ok=True)
        return open(path, mode, encoding="utf-8", buffering=1, **kw)

    # ---- log (caller holds the lock) ----

    def _add(self, level, src, msg):
        self.log_id += 1
        now = time.time()
        stamp = time.strftime("%H:%M:%S", time.localtime(now)) + ".%03d" % (now % 1 * 1000)
        self.log.append(dict(id=self.log_id, t=stamp, level=level, src=src, msg=msg))
        if self.log_file:
            self.log_file.write("%s %-5s %-6s %s\n" % (stamp, level.upper(), src, msg))

    def note(self, level, src, msg):
        with self.lock:
            self._add(level, src, msg)

    def set_broker(self, state):
        with self.lock:
            if state != self.broker:
                self.broker = state
                self.broker_at = time.monotonic()
                self._add("info" if state == "connected" else "error", "broker",
                          "dashboard %s MQTT broker %s"
                          % ("connected to" if state == "connected" else "lost the", self.broker_name))

    # ---- incoming MQTT ----

    def on_message(self, topic, payload):
        with self.lock:
            if topic == d.TOPIC_FRAME:
                self._on_frame(payload)
            elif topic == d.TOPIC_ENCODER:
                self._on_encoder(payload)
            elif topic == d.TOPIC_EVENT:
                self._on_event(payload)
            elif topic == d.TOPIC_ACK:
                self._on_ack(payload)
            elif topic == d.TOPIC_STATUS:
                text = payload.decode("ascii", "replace").strip()
                if text != self.robot:
                    self._add("info" if text == "online" else "error", "robot",
                              "robot is %s" % text.upper()
                              + ("" if text == "online" else
                                 " (Wi-Fi dropped, or the Pico reset or lost power)"))
                self.robot = text
            elif topic == TOPIC_HEARTBEAT:
                self.hb = payload.decode("ascii", "replace")
                self.hb_at = time.monotonic()
            elif topic.startswith(TOPIC_TEXT):
                self._on_text(topic[len(TOPIC_TEXT):], payload.decode("utf-8", "replace"))

    def _bad(self, what, err, payload):
        self.last_bad = (time.monotonic(), "%s: %s" % (what, err))
        self._add("error", "B1", "bad %s (%s): %s" % (what, err, payload.hex()))

    def _on_frame(self, payload):
        try:
            v = d.decode(payload)
        except ValueError as err:
            self.counts["bad"] += 1
            self._bad("status frame", err, payload)
            return
        now = time.time()
        p = self.frame
        if self.seq is not None:
            self.counts["lost"] += (v["SEQ"] - self.seq - 1) % 256
        self.seq = v["SEQ"]
        self.counts["frames"] += 1
        if self.frame_csv:
            self.frame_csv.writerow(["%.3f" % now] + [v[f[0]] for f in d.FIELDS])
        if p is None:
            self._add("info", "B2", "first status frame: motion %s, fault %s"
                      % (d.STATES[v["M_STATE"]], d.FAULTS[v["M_FAULT"]]))
        else:
            self._frame_changes(p, v)
        self.frame = v
        self.frame_at = time.monotonic()

    def _frame_changes(self, p, v):
        if v["UPTIME_DS"] + 50 < p["UPTIME_DS"] and p["UPTIME_DS"] < 65000:
            self._add("warn", "robot", "robot uptime went back to %.1f s: the robot rebooted"
                      % (v["UPTIME_DS"] / 10.0))
        if v["M_STATE"] != p["M_STATE"]:
            self._add("error" if d.STATES[v["M_STATE"]] == "FAULT" else "info", "B2",
                      "motion %s -> %s" % (d.STATES[p["M_STATE"]], d.STATES[v["M_STATE"]]))
        if v["M_FAULT"] != p["M_FAULT"]:
            name = d.FAULTS[v["M_FAULT"]]
            if v["M_FAULT"]:
                self._add("error", "B2", "FAULT %s: %s" % (name, FAULT_HELP.get(name, "")))
            else:
                self._add("info", "B2", "fault cleared")
        if (v["M_FLAGS"] ^ p["M_FLAGS"]) & 0x8:
            self._add("info", "B2", "movement %s" % ("HELD (stopped)" if v["M_FLAGS"] & 0x8
                                                      else "released"))
        if v["OWNER"] != p["OWNER"]:
            self._add("info", "ctrl", "motion owner: %s -> %s"
                      % (OWNERS[p["OWNER"]], OWNERS[v["OWNER"]]))
        if v["LINE_AVAIL"] != p["LINE_AVAIL"]:
            self._add("info" if v["LINE_AVAIL"] else "warn", "B3",
                      "line sensor status %s" % ("available" if v["LINE_AVAIL"] else "UNAVAILABLE"))
        if v["LINE_TRACK"] and not p["LINE_TRACK"]:
            self._add("info", "B3", "line found")
        if v["T_STATE"] != p["T_STATE"]:
            self._add("error" if T_STATES[v["T_STATE"]] == "FAULT" else "info", "B4",
                      "terrain %s -> %s" % (T_STATES[p["T_STATE"]], T_STATES[v["T_STATE"]]))
        if v["HUMP_MM"] > p["HUMP_MM"]:
            self._add("info", "B4", "highest hump this run: %d mm" % v["HUMP_MM"])
        if "M_CLASS" in v and v["T_VALID"] & 1 and v["M_CLASS"] != p.get("M_CLASS"):
            self._add("debug", "B4", "motion class %s" % M_CLASSES[v["M_CLASS"]])
        if v["O_AVAIL"] != p["O_AVAIL"]:
            self._add("info" if v["O_AVAIL"] else "warn", "B5",
                      "ultrasonic %s" % ("OK" if v["O_AVAIL"] else "NOT OK"))
        if v["O_FLAGS"] != p["O_FLAGS"] and v["O_FLAGS"] < len(S5_STATES):
            self._add("warn" if v["O_FLAGS"] in (4, 6) else "info", "B5",
                      "obstacle: %s" % S5_STATES[v["O_FLAGS"]])
        if v["REJECTED"]:
            self._add("warn", "ctrl", "controller rejected %d request(s) from a subsystem"
                      % v["REJECTED"])
        if v["DROPPED"]:
            self._add("warn", "B1", "robot dropped %d outgoing message(s): Wi-Fi or broker too slow"
                      % v["DROPPED"])

    def _on_encoder(self, payload):
        try:
            e = d.decode_encoder(payload)
        except ValueError as err:
            self.counts["enc_bad"] += 1
            self._bad("encoder frame", err, payload)
            return
        self.counts["enc_frames"] += 1
        if self.enc_csv:
            self.enc_csv.writerow(["%.3f" % time.time()] + [e[k] for k in d.ENC_FIELDS])
        p = self.enc
        if p is not None:
            for key, label in (("reliable", "encoder readings reliable"),
                               ("trusted", "encoder feedback trusted")):
                if e[key] != p[key]:
                    self._add("info" if e[key] else "warn", "B2",
                              "%s: %s" % (label, "yes" if e[key] else "NO"))
            if e["cause"] != p["cause"] and e["cause"]:
                cause = d.FB_CAUSES[e["cause"]]
                self._add("warn", "B2", "encoder cause %s: %s" % (cause, FB_HELP.get(cause, "")))
            new_l = d.count_delta(e["invalid_l"], p["invalid_l"])
            new_r = d.count_delta(e["invalid_r"], p["invalid_r"])
            mono = time.monotonic()
            if (new_l > 0 or new_r > 0) and mono - self.invalid_logged_at > 2.0:
                self.invalid_logged_at = mono
                self._add("warn", "B2", "invalid encoder transitions (left +%d, right +%d; totals "
                          "%d, %d): check encoder wiring" % (max(new_l, 0), max(new_r, 0),
                                                             e["invalid_l"], e["invalid_r"]))
        self.enc_prev, self.enc = p, e
        self.enc_at = time.monotonic()

    def _on_event(self, payload):
        if len(payload) != 2:
            self._bad("event", "length %d" % len(payload), payload)
            return
        self.counts["events"] += 1
        for name in d.event_names(payload[0]):
            src = {"hump_peak": "B4", "obstacle": "B5", "scan_done": "B5", "handover": "B5",
                   "line_lost": "B3", "barcode": "B3", "motion_fault": "B2"}.get(name, "B1")
            self._add(EVENT_LEVEL.get(name, "info"), src,
                      "event: %s" % EVENT_TEXT.get(name, name))

    def _on_ack(self, payload):
        try:
            result, opcode, seq = d.decode_ack(payload)
        except ValueError as err:
            self._bad("ack", err, payload)
            return
        name = {v: k for k, v in d.OPCODES.items()}.get(opcode, "opcode %d" % opcode).upper()
        text = {"ok": "queued for the controller",
                "duplicate": "ignored as a repeat of an earlier START",
                "busy": "DROPPED: the controller is not reading commands",
                "rejected": "REJECTED as malformed"}[result]
        self._add("info" if result == "ok" else "warn", "B1",
                  "robot received %s #%d: %s" % (name, seq, text))

    def _on_text(self, name, text):
        if name == "obstacle":
            for part in text.split():
                if "=" in part:
                    key, _, val = part.partition("=")
                    self.obstacle_text[key] = val
            self.obstacle_text["_at"] = time.time()
        self._add("info", "B5" if name == "obstacle" else name, text)

    # ---- staleness, commands, snapshot ----

    def tick(self):
        with self.lock:
            mono = time.monotonic()
            if self.broker != "connected" or mono - self.broker_at < HEARTBEAT_STALE_S:
                return              # give the robot a moment after (re)connecting
            for name, at, limit in (("status frames", self.frame_at, 1.0),
                                    ("encoder frames", self.enc_at, 1.0),
                                    ("heartbeat", self.hb_at, HEARTBEAT_STALE_S)):
                stale = at is None or mono - at > limit
                if stale and name not in self.stale:
                    self.stale.add(name)
                    self._add("warn", "B1", "no %s for %s (robot %s)"
                              % (name, "ever" if at is None else "%.1f s" % (mono - at), self.robot))
                elif not stale and name in self.stale:
                    self.stale.discard(name)
                    self._add("info", "B1", "%s resumed" % name)

    def command(self, name):
        if name not in d.OPCODES:
            return dict(ok=False, error="unknown command")
        with self.lock:
            self.cmd_seq = (self.cmd_seq + 1) & 0xFF
            seq = self.cmd_seq
            payload = d.encode_cmd(d.OPCODES[name], self.session, seq)
        # Publish outside the lock: the simulator answers from its own thread.
        sent = bool(self.publish and self.publish(payload))
        with self.lock:
            if sent:
                self._add("warn" if name == "estop" else "info", "you",
                          "sent %s #%d" % (name.upper(), seq))
            else:
                self._add("error", "you", "could not send %s: not connected to the broker"
                          % name.upper())
        if sent and name == "start":
            threading.Timer(START_CHECK_S, self._check_start, args=(seq,)).start()
        return dict(ok=sent, seq=seq, error=None if sent else "not connected to the broker")

    def _check_start(self, seq):
        with self.lock:
            f = self.frame
            # Skip if another command followed this START (a STOP re-holds it).
            if seq == self.cmd_seq and f is not None and f["M_FLAGS"] & 0x8:
                self._add("warn", "ctrl", "START had no effect: the robot is still held. It is "
                          "only released when STOPPED, fault-free, stationary and with trusted "
                          "encoder feedback (motion %s, fault %s)"
                          % (d.STATES[f["M_STATE"]], d.FAULTS[f["M_FAULT"]]))

    def snapshot(self, since, boot):
        with self.lock:
            mono = time.monotonic()

            def age(at):
                return None if at is None else round(mono - at, 2)

            fresh_log = boot != self.boot
            entries = [e for e in self.log if fresh_log or e["id"] > since][-500:]
            enc_delta = None
            if self.enc and self.enc_prev:
                enc_delta = [d.count_delta(self.enc["count_l"], self.enc_prev["count_l"]),
                             d.count_delta(self.enc["count_r"], self.enc_prev["count_r"])]
            out = dict(boot=self.boot, reset_log=fresh_log, log=entries, log_last=self.log_id,
                       broker=self.broker, broker_name=self.broker_name, robot=self.robot,
                       frame=self.frame, frame_age=age(self.frame_at),
                       enc=self.enc, enc_age=age(self.enc_at), enc_delta=enc_delta,
                       hb=self.hb, hb_age=age(self.hb_at), counts=dict(self.counts),
                       last_bad=None if self.last_bad is None else
                       dict(age=round(mono - self.last_bad[0], 1), msg=self.last_bad[1]),
                       obstacle_text=dict(self.obstacle_text), session=self.session)
            if fresh_log:
                out["meta"] = dict(states=d.STATES, faults=d.FAULTS, fb=d.FB_CAUSES, modes=MODES,
                                   owners=OWNERS, barcodes=BARCODES, t_states=T_STATES,
                                   m_classes=M_CLASSES, s5_states=S5_STATES,
                                   fault_help=FAULT_HELP, fb_help=FB_HELP,
                                   layout=d.VERSION, fields=[f[0] for f in d.FIELDS])
            return out

    def full_log(self):
        with self.lock:
            return "".join("%s %-5s %-6s %s\n" % (e["t"], e["level"].upper(), e["src"], e["msg"])
                           for e in self.log)


# ---- transports ----

def start_mqtt(hub, args):
    client = d.mqtt_client("smartrobo-web-%04x" % random.getrandbits(16))

    def on_connect(client, userdata, flags, rc, properties=None):
        client.subscribe([(d.TOPIC_FRAME, 0), (d.TOPIC_ENCODER, 0), (d.TOPIC_EVENT, 1),
                          (d.TOPIC_ACK, 1), (d.TOPIC_STATUS, 1), (TOPIC_HEARTBEAT, 0),
                          (TOPIC_TEXT + "#", 0)])
        hub.set_broker("connected")

    def on_disconnect(*_args):
        hub.set_broker("disconnected")

    def publish(payload):
        return client.publish(d.TOPIC_CMD, payload, qos=1).rc == 0

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = lambda c, u, m: hub.on_message(m.topic, m.payload)
    client.reconnect_delay_set(1, 5)
    client.connect_async(args.host, args.mqtt_port)
    client.loop_start()             # keeps reconnecting on its own
    hub.publish = publish


class Simulator(threading.Thread):
    """A fake robot for --demo: drives the line, meets a hump and an obstacle,
    and obeys START / STOP / E-STOP like the real controller."""

    def __init__(self, hub):
        super().__init__(daemon=True)
        self.hub = hub
        self.inbox = collections.deque()
        self.t = 0.0
        self.state, self.fault, self.hold = "DRIVING", 0, False
        self.stop_at = None
        self.count = [0, 0]
        self.invalid = [0, 0]
        self.seq = 0
        self.centre = 0.0
        self.heading = 0.0
        self.hump_max = 0
        self.ev_seq = 0

    def handle(self, payload):
        self.inbox.append(payload)
        return True

    def send(self, topic, payload):
        self.hub.on_message(topic, payload)

    def event(self, mask):
        self.send(d.TOPIC_EVENT, bytes([mask, self.ev_seq & 0xFF]))
        self.ev_seq += 1

    def commands(self):
        while self.inbox:
            word = int.from_bytes(self.inbox.popleft(), "big")
            opcode, seq = word >> 13, word & 0xFF
            self.send(d.TOPIC_ACK, ack_bytes(0, opcode, seq))
            if opcode == d.OPCODES["stop"] and self.state not in ("FAULT",):
                self.hold, self.state, self.stop_at = True, "STOPPING", self.t
            elif opcode == d.OPCODES["estop"]:
                self.hold, self.state, self.fault = True, "FAULT", d.FAULTS.index("ESTOP")
                self.event(0x01)
            elif opcode == d.OPCODES["start"] and self.state == "STOPPED" and not self.fault:
                self.hold, self.state = False, "DRIVING"

    def run(self):
        self.send(d.TOPIC_STATUS, b"online")
        while True:
            time.sleep(0.1)
            self.t += 0.1
            self.commands()
            if self.state == "STOPPING" and self.t - self.stop_at > 0.6:
                self.state = "STOPPED"
            self.send_frames()
            if int(self.t * 10) % 10 == 0:
                self.send(TOPIC_HEARTBEAT, b"hb %d" % int(self.t))

    def send_frames(self):
        t = self.t
        driving = self.state == "DRIVING"
        cycle = t % 40.0                         # 40 s loop: hump at 10 s, obstacle at 25 s
        obstacle = 0
        if driving and 25.0 <= cycle < 34.0:
            obstacle = 1 if cycle < 28.0 else (3 if cycle < 29.0 else 5)
        if driving and abs(cycle - 25.0) < 0.05:
            self.event(0x10)
        if driving and abs(cycle - 28.0) < 0.05:
            self.event(0x20)
            self.send(TOPIC_TEXT + "obstacle", b"obstacle width_mm=184 dist_mm=152 offset_mm=31 "
                      b"complete=1 position=left")
            self.send(TOPIC_TEXT + "obstacle", b"obstacle avoid=right shift_l_mm=230 shift_r_mm=172")
        if driving and abs(cycle - 34.0) < 0.05:
            self.event(0x40)
        terrain = 1
        if driving and 10.0 <= cycle < 13.0:
            terrain = 2 if cycle < 11.5 else 3
            if abs(cycle - 12.9) < 0.05:
                self.hump_max = max(self.hump_max, 30 + int(t) % 12)
                self.event(0x08)
        moving = driving and obstacle in (0, 5)
        err = (0 if int(t / 3) % 4 else (1 if int(t) % 2 else -1)) if moving else 0
        base = 150 if moving else 0
        speed = [base - 30 * err, base + 30 * err]
        for i in range(2):
            self.count[i] = (self.count[i] + int(speed[i] * 0.1 * 12.45)) & 0xFFFFFFFF
        if moving and random.random() < 0.004:
            self.invalid[1] += 1
        self.centre += (speed[0] + speed[1]) / 2 * 0.1
        if self.centre > 5000:
            self.centre -= 10000
        self.heading = (self.heading + (speed[1] - speed[0]) / 115.0 * 0.1 * 57.3 + 180) % 360 - 180
        stationary = not moving
        values = {name: lo for name, lo, _hi, _bits in d.FIELDS}
        values.update(
            VERSION=d.VERSION, SEQ=self.seq & 0xFF, UPTIME_DS=int(t * 10) & 0xFFFF,
            M_STATE=d.STATES.index(self.state), M_MODE=2, M_FAULT=self.fault,
            M_FLAGS=(1 if stationary else 0) | 2 | (8 if self.hold else 0),
            OWNER=(3 if obstacle == 5 else 1) if moving else 7, CMD_ID=int(t) & 0xFF,
            SPEED_L=speed[0], SPEED_R=speed[1], CENTRE_MM=int(self.centre),
            HEADING=int(self.heading), LINE_AVAIL=1, LINE_ERR=err,
            LINE_TRACK=1 if obstacle in (0,) else 0, T_VALID=2 | (4 if self.hump_max else 0),
            T_STATE=terrain, PITCH={2: 12, 3: -10}.get(terrain, 0), HUMP_MM=self.hump_max,
            O_FLAGS=obstacle, O_AVAIL=1, O_DIST_CM=15 if obstacle else 400,
            O_WIDTH_CM=18 if obstacle >= 2 else 0)
        if "M_CLASS" in values:               # layout v2 (Buddy 4 change request)
            values.update(T_VALID=values["T_VALID"] | 1,
                          M_CLASS=0 if stationary else (5 if terrain == 2 else 1),
                          TURN_RATE=int((speed[1] - speed[0]) / 115.0 * 57.3))
        self.send(d.TOPIC_FRAME, d.encode(values))
        self.send(d.TOPIC_ENCODER, d.encode_encoder(dict(
            seq=self.seq & 0xFF, reliable=1, trusted=1, fresh=1, cause=0,
            state=d.STATES.index(self.state), fault=self.fault,
            sample_ms=int(t * 1000) & 0xFFFFFFFF, age_ms=4,
            count_l=self.count[0], count_r=self.count[1],
            invalid_l=self.invalid[0], invalid_r=self.invalid[1])))
        self.seq += 1


# ---- HTTP ----

PAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_dashboard.html")


def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass                                    # keep the console quiet

        def reply(self, code, body, ctype, extra=None):
            data = body.encode("utf-8") if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/":
                with open(PAGE_PATH, "rb") as f:
                    self.reply(200, f.read(), "text/html; charset=utf-8")
            elif url.path == "/api/state":
                q = parse_qs(url.query)
                since = int(q.get("since", ["0"])[0] or 0)
                boot = q.get("boot", [""])[0]
                self.reply(200, json.dumps(hub.snapshot(since, boot)), "application/json")
            elif url.path == "/api/log.txt":
                name = time.strftime("smartrobo-log-%Y%m%d-%H%M%S.txt")
                self.reply(200, hub.full_log(), "text/plain; charset=utf-8",
                           {"Content-Disposition": 'attachment; filename="%s"' % name})
            else:
                self.reply(404, "not found", "text/plain")

        def do_POST(self):
            if urlparse(self.path).path != "/api/cmd":
                self.reply(404, "not found", "text/plain")
                return
            try:
                length = min(int(self.headers.get("Content-Length", 0)), 1024)
                body = json.loads(self.rfile.read(length) or b"{}")
                name = str(body.get("command", ""))
            except (ValueError, AttributeError):
                self.reply(400, json.dumps(dict(ok=False, error="bad request")), "application/json")
                return
            self.reply(200, json.dumps(hub.command(name)), "application/json")

    return Handler


def local_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.0.2.1", 9))             # no packet is sent
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1", help="MQTT broker address (default 127.0.0.1)")
    p.add_argument("--mqtt-port", type=int, default=1883)
    p.add_argument("--web-port", type=int, default=8080)
    p.add_argument("--bind", default="0.0.0.0",
                   help="web server address; 127.0.0.1 = this laptop only (default: whole network)")
    p.add_argument("--log", help="also append the event log to this text file")
    p.add_argument("--csv", help="log every status frame to this CSV")
    p.add_argument("--encoder-csv", help="log every encoder frame to this CSV")
    p.add_argument("--demo", action="store_true", help="simulated robot, no broker needed")
    args = p.parse_args()

    hub = Hub(args)
    if args.demo:
        sim = Simulator(hub)
        hub.publish = sim.handle
        hub.set_broker("connected")
        sim.start()
    else:
        start_mqtt(hub, args)

    def ticker():
        while True:
            time.sleep(0.5)
            hub.tick()

    threading.Thread(target=ticker, daemon=True).start()
    server = ThreadingHTTPServer((args.bind, args.web_port), make_handler(hub))
    server.daemon_threads = True
    where = local_ip() if args.bind == "0.0.0.0" else args.bind
    print("SmartRobo dashboard: http://%s:%d  (broker %s, frame layout v%d)%s"
          % (where, args.web_port, hub.broker_name, d.VERSION, "  [DEMO]" if args.demo else ""))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
