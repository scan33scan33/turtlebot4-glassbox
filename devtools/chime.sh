#!/usr/bin/env bash
# chime.sh — Bring the TurtleBot 4 to READY and make it sing "Twinkle Twinkle
# Little Star". Idempotent: safe to run from any state, safe to run twice.
#
#   bash chime.sh                  # recover + chime (prompts once for sudo)
#   bash chime.sh --no-undock      # stay on the dock (cannot reach READY, see below)
#   bash chime.sh --base           # also force-reboot the Create 3 base first
#   TB4_SUDO_PW=... bash chime.sh  # unattended (no sudo prompt)
#
# DOCKED IS NOT READY. turtlebot4_node deliberately parks the RPLIDAR motor and
# calls oakd/stop_camera while docked, so cam+lidar can NEVER be live on the
# dock. This script therefore undocks first by default. Hammering /start_motor
# against the dock's power-save is what faulted the lidar firmware
# ("Cannot start scan: '80008000'") and killed the driver on 2026-08-20 — so
# motor retries are capped and the driver is checked for life before each one.
set -u

ROOT=${TB4_ROOT:-/home/ubuntu/Workspace/turtlebot4-glassbox}
BASE=${TB4_BASE:-192.168.186.2}   # Create 3 base over usb0 (default; override with TB4_BASE env)
# Web UI host for status messages — auto-detected from the Pi's LAN address,
# or override with TB4_HOST (e.g. TB4_HOST=192.168.1.42 bash chime.sh)
_WEB_HOST=${TB4_HOST:-$(hostname -I 2>/dev/null | awk '{print $1}')}
WEB_HOST=${_WEB_HOST:-turtlebot4.lan}
NAV=tb4-nav.service                # web control (systemd)
BRINGUP=turtlebot4.service         # lidar + camera + Create 3 bridge
BUDGET=${TB4_BUDGET:-240}          # hard wall-clock ceiling, seconds

FORCE_BASE=0; UNDOCK=1
for a in "$@"; do case "$a" in
  --base|--full) FORCE_BASE=1 ;;
  --no-undock)   UNDOCK=0 ;;
  -h|--help) sed -n "2,16p" "$0"; exit 0 ;;
  *) echo "unknown option: $a (try --base, --no-undock or --help)" >&2; exit 2 ;;
esac; done

# TB4_SUDO_PW is never committed — it is an ephemeral env var for unattended sudo
# (e.g. CI or `TB4_SUDO_PW=... bash chime.sh`). Leave unset for interactive use.
SUDO(){ if [ -n "${TB4_SUDO_PW:-}" ]; then echo "$TB4_SUDO_PW" | sudo -S -p "" "$@"; else sudo "$@"; fi; }
set +u; source /opt/ros/jazzy/setup.bash; set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/fastdds_no_shm.xml"
cd "$ROOT"

DEADLINE=$(( $(date +%s) + BUDGET ))
left(){ echo $(( DEADLINE - $(date +%s) )); }
expired(){ [ "$(left)" -le 0 ]; }
say(){ echo -e "\n[chime] $*"; }
die(){ say "$*"; exit 1; }

flow(){ timeout 4 ros2 topic hz "$1" 2>/dev/null | grep -q "average rate"; }
# The rplidar driver is launch-managed with no respawn: if it died, /start_motor
# survives as a discovery ghost that eats a full timeout per call and can never
# succeed. Only a bringup restart brings it back.
lidar_alive(){ pgrep -f "rplidar_[c]omposition" >/dev/null 2>&1; }
motor(){ timeout 8 ros2 service call /start_motor std_srvs/srv/Empty >/dev/null 2>&1; }
docked_now(){ timeout 8 ros2 topic echo --once /dock_status 2>/dev/null | grep -q "is_docked: true"; }
all_live(){ curl -s -m 3 http://127.0.0.1:5000/state 2>/dev/null | python3 -c "import sys,json;d=json.load(sys.stdin);sys.exit(0 if d.get(\"cam_ok\") and d.get(\"lidar_ok\") and d.get(\"base_ok\") else 1)" 2>/dev/null; }

BRINGUP_RESTARTED=0
restart_bringup(){ say "restarting bringup"; SUDO systemctl restart "$BRINGUP"; BRINGUP_RESTARTED=1; command sleep 8; }
reboot_base(){
  say "rebooting Create 3 base ($BASE)"
  if ! curl -fsS -m 10 -X POST "http://$BASE/api/reboot" >/dev/null 2>&1; then
    echo "  reboot API returned non-2xx — the base may be wedged; a physical"
    echo "  power-button wake or firm re-dock may be required."
  fi
  for i in $(seq 1 90); do ping -c1 -W1 "$BASE" >/dev/null 2>&1 && { echo "  base reachable after ~${i}s"; break; }; command sleep 1; done
  restart_bringup
}

SUDO -v 2>/dev/null || true

# 1. Undock BEFORE touching nav — the dock power-save makes READY unreachable,
#    and /undock rides the nav-independent Create 3 action server.
if docked_now; then
  if [ "$UNDOCK" = 1 ]; then
    say "docked -> undocking (cam+lidar are force-parked on the dock)"
    timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" >/dev/null 2>&1 \
      || echo "  undock action failed or timed out — continuing, will verify"
    for i in $(seq 1 15); do docked_now || { echo "  undocked after ~$((i*2))s"; break; }; command sleep 2; done
    docked_now && die "still docked — cannot reach READY. Undock by hand and re-run."
  else
    say "docked and --no-undock given: the firmware parks the lidar and stops"
    echo "  the camera on the dock, so cam+lidar cannot go live. Base-only check."
  fi
fi

# 2. Stop web control FAST (it ignores SIGTERM ~90s); SIGKILL its cgroup, which
#    also clears any OAK pipeline it self-healed (avoids a duplicate).
say "stopping web control"
SUDO systemctl kill -s SIGKILL "$NAV" 2>/dev/null || true
SUDO systemctl stop "$NAV"           2>/dev/null || true
SUDO systemctl reset-failed "$NAV"   2>/dev/null || true

# 3. Base bridge healthy (odom flowing). Only restart bringup / reboot the base
#    when actually needed — extra restarts churn and can wedge the OAK.
if [ "$FORCE_BASE" = 1 ]; then
  reboot_base
elif ! flow /odom; then
  say "no odom -> restarting bringup"
  restart_bringup
  flow /odom || { say "still no odom -> rebooting base"; reboot_base; }
else
  say "odom already flowing -> leaving bringup alone"
fi

# 4. Lidar. A dead driver needs bringup, not more service calls. Cap the motor
#    kicks at 3: if /scan will not hold after that, something is wrong that
#    retrying only makes worse (see the firmware fault in the header).
if [ "$UNDOCK" = 1 ] || ! docked_now; then
  if ! lidar_alive; then
    say "rplidar driver is DEAD (not just parked) -> bringup restart required"
    [ "$BRINGUP_RESTARTED" = 1 ] || restart_bringup
    lidar_alive || die "rplidar did not come back after a bringup restart — check the USB/serial link (/dev/ttyUSB0)."
  fi
  if flow /scan; then
    say "scan already live -> leaving the motor alone"
  else
    say "starting lidar motor (max 3 attempts)"
    for i in 1 2 3; do
      expired && break
      lidar_alive || { say "rplidar died during motor start -> stopping (it needs a bringup restart)"; break; }
      motor; command sleep 3
      flow /scan && { echo "  scan live after attempt $i"; break; }
      echo "  attempt $i: no scan yet"
    done
  fi
  flow /scan || say "WARNING: /scan still not flowing — continuing without lidar"
fi

# 5. Start web control clean; its announcer chimes once cam+lidar+base all live.
SINCE=$(date "+%Y-%m-%d %H:%M:%S")
say "starting web control"
SUDO systemctl start "$NAV"

# 6. Wait for the navigator READY chime (announcer, the stock path).
say "waiting for READY chime (up to $(left)s left in budget)"
while ! expired; do
  journalctl -u "$NAV" --since "$SINCE" --no-pager 2>/dev/null | grep -qa "READY — chimed" \
    && { say "READY — Twinkle chimed (announcer). Web UI: http://${WEB_HOST}:5000"; exit 0; }
  all_live && break
  command sleep 3
done

# 7. Fallback so it ALWAYS sings: announcer stayed silent but streams are live.
if all_live; then
  say "announcer silent -> direct chime on the base speaker"
  python3 - <<"PY"
import rclpy, time
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
NOTE={"C":523,"D":587,"E":659,"F":698,"G":784,"A":880}; Q,H=260,520
SEQ=[("C",Q),("C",Q),("G",Q),("G",Q),("A",Q),("A",Q),("G",H),
     ("F",Q),("F",Q),("E",Q),("E",Q),("D",Q),("D",Q),("C",H)]
rclpy.init(); n=Node("chime_fallback"); p=n.create_publisher(AudioNoteVector,"/cmd_audio",10)
time.sleep(1.0)
m=AudioNoteVector(); m.append=False
for c,ms in SEQ:
    a=AudioNote(); a.frequency=int(NOTE[c]); a.max_runtime=Duration(sec=ms//1000, nanosec=(ms%1000)*1_000_000); m.notes.append(a)
for _ in range(3):
    p.publish(m); time.sleep(0.2)
time.sleep(0.5); n.destroy_node(); rclpy.shutdown()
PY
  say "Twinkle chimed (direct fallback). Web UI: http://${WEB_HOST}:5000"; exit 0
fi

curl -s -m 3 http://127.0.0.1:5000/state 2>/dev/null \
  | python3 -c "import sys,json;d=json.load(sys.stdin);print('  cam_ok=%s lidar_ok=%s base_ok=%s docked=%s'%(d.get('cam_ok'),d.get('lidar_ok'),d.get('base_ok'),d.get('docked')))" 2>/dev/null
die "streams NOT all live within ${BUDGET}s -> no chime. Check http://${WEB_HOST}:5000"
