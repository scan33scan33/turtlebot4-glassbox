#!/usr/bin/env bash
# Launch wrapper for tb4_claude_nav.py — starts the navigator cleanly.
#
# IMPORTANT: This does NOT start the robot bringup. The TurtleBot4 bringup
# (LiDAR, camera, Create 3 republisher bridge) is owned by the systemd
# service `turtlebot4.service` and is already running on boot. Do NOT run
# `ros2 launch turtlebot4_bringup robot.launch.py` by hand — a second bringup
# spawns duplicate `create3_republisher`/`turtlebot4_node` nodes that collide
# over the Create 3's BEST_EFFORT topics and silently break the bridge
# (frozen /odom, dead /cmd_vel). To restart the bridge instead use:
#     sudo systemctl restart turtlebot4.service
# After rebooting the Create 3 base, you MUST restart that service so the
# bridge re-handshakes the base's new DDS GUIDs.
set -e

source /opt/ros/jazzy/setup.bash

# Portable checkout root — override with TB4_ROOT if your clone lives elsewhere.
# TB4_ROOT is the CHECKOUT itself, not its parent, matching run_oakd.sh,
# chime.sh, services/tb4-oakd-run.sh and oakd_rgbd.launch.py.
ROOT=${TB4_ROOT:-$HOME/Workspace/turtlebot4-glassbox}

# Disable Fast-DDS shared memory — this box's SHM transport is flaky and
# causes "rcl node's context is invalid" crashes on rapid ros2 calls.
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/fastdds_no_shm.xml"

cd "$ROOT"

# Kill any previous navigator so port 5000 is free and we don't get two
# control loops fighting over /cmd_vel.
pkill -9 -f 'python3 .*tb4_claude_nav.py' 2>/dev/null || true
sleep 1

exec python3 -u tb4_claude_nav.py
