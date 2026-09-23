#!/usr/bin/env bash
# Bring up the OAK-D Lite spatial-YOLO pipeline via the minimal custom launch
# (bypasses turtlebot4_bringup). Logs to ~/oak_rgbd.log. Run detached:
#   setsid bash ~/turtlebot4-glassbox/run_oakd.sh >/dev/null 2>&1 </dev/null &
set -euo pipefail

# ROS's setup.bash references unbound vars, so relax -u just for the source.
set +u; source /opt/ros/jazzy/setup.bash; set -u

# Portable workspace root — override with TB4_ROOT if your clone lives elsewhere
# (matches run_nav.sh and chime.sh).
ROOT=${TB4_ROOT:-$HOME/Workspace/turtlebot4-glassbox}
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/fastdds_no_shm.xml"
# Preflight: the blobs are Release assets, not committed. Fail with a clear
# pointer instead of letting depthai die on a missing file. The name comes from
# object_detection/DEFAULT_MODEL — the same source oakd_rgbd.launch.py reads — so this
# check can never pass for a blob the launch is not about to load.
MODEL="${TB4_OAKD_MODEL:-$(cat "$ROOT/object_detection/DEFAULT_MODEL")}"
if [ ! -f "$ROOT/object_detection/$MODEL.blob" ]; then
    echo "error: object_detection/$MODEL.blob is missing." >&2
    echo "       Fetch the model blobs first:  bash object_detection/download_models.sh" >&2
    exit 1
fi
# Kill a previous container (match the launch file name, NOT this script).
# `|| true`: pkill exits 1 when nothing matched, which is the normal case.
pkill -9 -f oakd_rgbd.launch 2>/dev/null || true
pkill -9 -f component_container 2>/dev/null || true
# The OAK-D USB handle takes several seconds to release after a kill; relaunching
# too soon stacks up containers that fail with X_LINK_DEVICE_ALREADY_IN_USE.
sleep 12
# Fully detach the launch (setsid + background) so it survives the SSH session
# that started this script. Logs to ~/oak_rgbd.log. Script returns immediately.
setsid stdbuf -oL -eL ros2 launch "$ROOT/object_detection/oakd_rgbd.launch.py" \
    > "$HOME/oak_rgbd.log" 2>&1 < /dev/null &
disown
echo "oakd launch detached, logging to ~/oak_rgbd.log"
