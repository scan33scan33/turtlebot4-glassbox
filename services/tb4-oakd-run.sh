#!/usr/bin/env bash
# Foreground OAK-D launcher for systemd (unlike run_oakd.sh which detaches).
# systemd owns the process lifetime + restarts it on failure.
set -euo pipefail

# ROS's setup.bash references unbound vars, so relax -u just for the source.
set +u; source /opt/ros/jazzy/setup.bash; set -u

# Portable workspace root — override with TB4_ROOT (the systemd unit sets the
# WorkingDirectory; this keeps a non-default checkout working too).
ROOT=${TB4_ROOT:-$HOME/Workspace/turtlebot4-glassbox}
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/fastdds_no_shm.xml"
# Preflight: the blobs are Release assets, not committed. Fail with a clear
# pointer instead of letting depthai die on a missing file.
if [ ! -f "$ROOT/models/yolov5mu_416_5shave.blob" ]; then
    echo "error: models/yolov5mu_416_5shave.blob is missing." >&2
    echo "       Fetch the model blobs first:  bash scripts/download_models.sh" >&2
    exit 1
fi
# Clear any stale container so the USB handle is free (OAK needs ~12s to release).
pkill -9 -f oakd_rgbd.launch 2>/dev/null || true
pkill -9 -f component_container 2>/dev/null || true
sleep 12
exec ros2 launch "$ROOT/oakd_rgbd.launch.py"
