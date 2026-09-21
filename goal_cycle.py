#!/usr/bin/env python3
"""goal_cycle.py — one step of the Twinkle goal loop, driven off nav's /state.

Design notes (2026-08-20):
  * nav's /state is the ONLY trustworthy stream probe on this box. `ros2 topic hz`
    from the CLI lies (short-lived subscribers often miss discovery under the
    no-SHM profile) — that false negative is what cascaded into a bringup restart
    + base reboot earlier today. Never gate decisions on it.
  * Chime directly via /cmd_audio instead of restarting tb4-nav to trigger its
    announcer. Same tune, no churn, ~2s instead of ~60s.
  * Recovery (chime.sh) is a LAST resort, only when a stream is actually down.
"""
import json, os, subprocess, sys, time, urllib.request

NAV = os.environ.get("TB4_NAV", "http://127.0.0.1:5000")
# Portable repo root — works regardless of where the repo is cloned
ROOT = os.path.dirname(os.path.abspath(__file__))

def state(timeout=6):
    with urllib.request.urlopen(NAV + "/state", timeout=timeout) as r:
        return json.load(r)

def post(path, payload=None, timeout=15):
    data = json.dumps(payload or {}).encode()
    req = urllib.request.Request(NAV + path, data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)

def all_live(s):
    return bool(s.get("cam_ok") and s.get("lidar_ok") and s.get("base_ok"))

def twinkle():
    """Play Twinkle Twinkle Little Star on the Create 3 speaker."""
    import rclpy
    from rclpy.node import Node
    from irobot_create_msgs.msg import AudioNoteVector, AudioNote
    from builtin_interfaces.msg import Duration
    NOTE = {"C":523,"D":587,"E":659,"F":698,"G":784,"A":880}; Q,H = 260,520
    SEQ = [("C",Q),("C",Q),("G",Q),("G",Q),("A",Q),("A",Q),("G",H),
           ("F",Q),("F",Q),("E",Q),("E",Q),("D",Q),("D",Q),("C",H)]
    rclpy.init(); n = Node("twinkle_goal"); p = n.create_publisher(AudioNoteVector, "/cmd_audio", 10)
    time.sleep(1.0)
    m = AudioNoteVector(); m.append = False
    for c, ms in SEQ:
        a = AudioNote(); a.frequency = int(NOTE[c])
        a.max_runtime = Duration(sec=ms//1000, nanosec=(ms % 1000)*1_000_000)
        m.notes.append(a)
    for _ in range(3):
        p.publish(m); time.sleep(0.2)
    time.sleep(0.5); n.destroy_node(); rclpy.shutdown()

def recover():
    """Last-resort: full chime.sh recovery. Only when a stream is genuinely down."""
    subprocess.run(["bash", os.path.join(ROOT, "chime.sh"), "--no-undock"],
                   cwd=ROOT, env={"TB4_SUDO_PW": os.environ.get("TB4_SUDO_PW", ""), "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
                                  "HOME": os.path.expanduser("~")},
                   timeout=300, capture_output=True)

def ensure_ready(max_wait=150):
    """Converge to cam+lidar+base live, then chime. Returns (ok, chimed, state)."""
    s = state()
    if not all_live(s):
        # Give nav's own self-heal a chance before escalating to chime.sh.
        deadline = time.time() + 60
        while time.time() < deadline:
            time.sleep(5); s = state()
            if all_live(s): break
    if not all_live(s):
        recover()
        deadline = time.time() + max_wait
        while time.time() < deadline:
            time.sleep(5)
            try: s = state()
            except Exception: continue
            if all_live(s): break
    if not all_live(s):
        return False, False, s
    twinkle()
    return True, True, s

def drive(dist=0.4, timeout=75):
    """Nudge forward `dist` m in the odom frame (well inside the 1 m cap)."""
    import math
    s = state()
    x, y, yaw = s["odom_x"], s["odom_y"], s["odom_yaw"]
    tx, ty = x + dist*math.cos(yaw), y + dist*math.sin(yaw)
    post("/goto_xy", {"x": tx, "y": ty, "mode": "move"})
    start = (x, y); deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(3); s = state()
        moved = math.hypot(s["odom_x"]-start[0], s["odom_y"]-start[1])
        if moved >= dist*0.75 or not s.get("nav_active"):
            break
        if moved > 1.0:                      # hard 1 m safety cap
            break
    post("/stop", {})
    s = state()
    return math.hypot(s["odom_x"]-start[0], s["odom_y"]-start[1]), s

if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    if cmd == "status":
        print(json.dumps(state()))
    elif cmd == "ready":
        ok, chimed, s = ensure_ready()
        print(json.dumps({"ok": ok, "chimed": chimed, "state": s}))
    elif cmd == "drive":
        d = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
        moved, s = drive(d)
        print(json.dumps({"moved": moved, "state": s}))
    elif cmd == "dock":
        post("/dock", {})
        deadline = time.time() + 120; s = state()
        while time.time() < deadline and not s.get("docked"):
            time.sleep(5); s = state()
        print(json.dumps({"docked": bool(s.get("docked")), "state": s}))
    else:
        print("usage: goal_cycle.py status|ready|drive [m]|dock", file=sys.stderr); sys.exit(2)
