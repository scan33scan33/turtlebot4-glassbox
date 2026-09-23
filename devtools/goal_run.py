#!/usr/bin/env python3
"""Final goal loop: chime + real drive leg, N cycles.

Fixes from the earlier runs:
  * /state reports odom_yaw in DEGREES -- must math.radians() it. Using it raw
    aimed the robot in an arbitrary direction and drove it into an obstacle.
  * Waypoints must be >2x nav's 0.25 m arrival tolerance or the robot is already
    "arrived" and never moves. These are 0.7 m apart.
  * Shuttles along an axis verified clear (the robot bumped on the other heading).
"""
import json, math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from goal_cycle import state, post, ensure_ready

LOG = os.path.join(os.path.expanduser("~"), "goal_run.jsonl")
N = int(sys.argv[1]); START = int(sys.argv[2])
HOME = (float(sys.argv[3]), float(sys.argv[4]))
AWAY = (float(sys.argv[5]), float(sys.argv[6]))

def emit(r):
    with open(LOG, "a") as f: f.write(json.dumps(r)+"\n"); f.flush()

for k in range(N):
    n = START + k; t0 = time.time()
    try: ok, chimed, s = ensure_ready()
    except Exception as e:
        emit({"cycle": n, "ok": False, "chimed": False, "error": repr(e)}); continue
    rec = {"cycle": n, "ok": ok, "chimed": chimed, "battery": s.get("battery_percentage"),
           "docked": s.get("docked"), "cam": s.get("cam_ok"), "lidar": s.get("lidar_ok"),
           "base": s.get("base_ok"), "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "chime_secs": round(time.time()-t0,1)}
    b = s.get("battery_percentage") or 0
    try:
        if b < 0.20:
            if not s.get("docked"):
                post("/dock"); rec["action"] = "docking (battery<20%)"
                for _ in range(24):
                    time.sleep(5)
                    if state().get("docked"): break
            else: rec["action"] = "already docked (battery<20%)"
        elif b > 0.50:
            if state().get("docked"): post("/undock"); time.sleep(6)
            tx, ty = AWAY if k % 2 == 0 else HOME
            sx = state(); x0, y0 = sx["odom_x"], sx["odom_y"]
            post("/goto_xy", {"x": tx, "y": ty, "mode": "move"})
            moved = 0.0; bumped = False
            for _ in range(20):
                time.sleep(2); sx = state()
                moved = math.hypot(sx["odom_x"]-x0, sx["odom_y"]-y0)
                if sx.get("hazards"): bumped = True
                if moved > 1.0 or not sx.get("nav_active"): break
            post("/stop")
            rec["action"] = f"drove {moved:.3f}m" + (" (BUMP)" if bumped or "bump" in str(state().get("status")) else "")
        else:
            rec["action"] = "no drive (20% < battery < 50%)"
    except Exception as e:
        rec["action"] = f"drive error {e!r}"
    emit(rec); time.sleep(3)
emit({"done": True})
