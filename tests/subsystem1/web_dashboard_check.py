"""S1-H05: the web dashboard against its simulated robot (no broker, no Pico).

Starts tools/web_dashboard.py --demo on a local port, drives it through the
same HTTP API the browser buttons use, and checks the link figures and the
event log.  Prints one PASS/FAIL line per check and a summary line.

    python tests/subsystem1/web_dashboard_check.py

The simulator always accepts a route, so a rejected route is a robot test
(S1-U18).  Stale/offline marking needs the robot to go away (S1-U20).
"""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import dashboard as d  # noqa: E402  (frame layout version)

failed = 0
passed = 0


def check(name, ok, detail=""):
    global failed, passed
    if ok:
        passed += 1
    else:
        failed += 1
    print("%s  %s%s" % ("PASS" if ok else "FAIL", name, ("  (" + detail + ")") if detail and not ok else ""))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def main():
    port = free_port()
    base = "http://127.0.0.1:%d" % port
    proc = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "web_dashboard.py"), "--demo",
                             "--web-port", str(port), "--bind", "127.0.0.1"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def get(path):
        with urllib.request.urlopen(base + path, timeout=5) as r:
            return r.read().decode()

    def post(path, body):
        req = urllib.request.Request(base + path, json.dumps(body).encode(),
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())

    def state():
        return json.loads(get("/api/state?since=0&boot="))

    def log_after(marker_seconds=1.0):
        time.sleep(marker_seconds)
        return get("/api/log.txt")

    try:
        for _ in range(50):
            try:
                get("/")
                break
            except OSError:
                time.sleep(0.2)
        else:
            check("dashboard starts", False, "no answer on " + base)
            return

        print("dashboard demo on %s, frame layout v%d" % (base, d.VERSION))
        time.sleep(1.5)
        s0 = state()
        time.sleep(3.0)
        s1 = state()
        df = s1["counts"]["frames"] - s0["counts"]["frames"]
        de = s1["counts"]["enc_frames"] - s0["counts"]["enc_frames"]
        check("broker connected, robot online", s1["broker"] == "connected" and s1["robot"] == "online",
              "%s / %s" % (s1["broker"], s1["robot"]))
        check("status frames at 10 Hz (3 s: 27..33)", 27 <= df <= 33, "got %d" % df)
        check("encoder frames at 10 Hz (3 s: 27..33)", 27 <= de <= 33, "got %d" % de)
        check("no lost or bad frames", s1["counts"]["lost"] == 0 and s1["counts"]["bad"] == 0
              and s1["counts"]["enc_bad"] == 0, str(s1["counts"]))
        check("telemetry fresh (frame < 0.5 s, heartbeat < 1.5 s)",
              s1["frame_age"] is not None and s1["frame_age"] < 0.5
              and s1["hb_age"] is not None and s1["hb_age"] < 1.5,
              "frame %s s, heartbeat %s s" % (s1["frame_age"], s1["hb_age"]))
        check("frame layout shown is v%d" % d.VERSION, s1["meta"]["layout"] == d.VERSION,
              str(s1["meta"]["layout"]))

        r = post("/api/cmd", {"command": "start"})
        log = log_after()
        check("START sent and acknowledged", r["ok"] and
              ("robot received START #%d: queued for the controller" % r["seq"]) in log, str(r))

        r2 = post("/api/cmd", {"command": "stop"})
        log = log_after()
        check("STOP acknowledged, new sequence number, robot held",
              r2["ok"] and r2["seq"] == (r["seq"] + 1) & 0xFF and
              ("robot received STOP #%d: queued for the controller" % r2["seq"]) in log
              and "movement HELD" in log, str(r2))

        r3 = post("/api/cmd", {"command": "estop"})
        log = log_after()
        check("E-STOP acknowledged and shown as a latched fault START cannot clear",
              r3["ok"] and ("robot received ESTOP #%d" % r3["seq"]) in log
              and "FAULT ESTOP" in log and "START will not clear it" in log, str(r3))

        r4 = post("/api/cmd", {"command": "launch"})
        check("unknown command refused, nothing sent", not r4["ok"] and r4["error"] == "unknown command", str(r4))

        r5 = post("/api/route", {"route": "L=B R=C S=D U=A"})
        log = log_after()
        check("route sent and acknowledged", r5["ok"] and "sent route L=B R=C S=D U=A" in log
              and "robot route: ok L=B R=C S=D U=A" in log, str(r5))

        r6 = post("/api/route", {"route": "   "})
        check("empty route refused, nothing sent", not r6["ok"], str(r6))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print("%s: web dashboard demo, %d passed, %d failed" % ("PASS" if failed == 0 else "FAIL", passed, failed))


if __name__ == "__main__":
    main()
    sys.exit(1 if failed else 0)
