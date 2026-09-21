# TB4 operations — hard-won gotchas

Lessons from debugging "the robot won't move" (the navigator software was fine
the whole time — the failures were all in the ROS/DDS/base layer). The Pi is
reached over SSH (`turtlebot4.lan`); on boot the three systemd services
(`turtlebot4`, `tb4-oakd`, `tb4-nav`) bring everything up — see
`services/README.md`.

- **Never run `ros2 launch turtlebot4_bringup robot.launch.py` by hand.** The
  bringup (LiDAR, camera, Create 3 `create3_republisher` bridge) is owned by the
  systemd service `turtlebot4.service` and starts on boot. A second bringup
  spawns duplicate `create3_republisher`/`turtlebot4_node` nodes that collide
  over the Create 3's BEST_EFFORT topics and silently break the bridge → frozen
  `/odom`, dead `/cmd_vel`, no `/hazard_detection`. Fix: kill the duplicates, or
  `sudo systemctl restart turtlebot4.service`.

- **Create 3 base runs ROS 2 Iron firmware (`I.0.0.FastDDS`); the Pi runs
  Jazzy.** The republisher bridges the two. Sensor data (Iron→Pi)
  auto-rediscovers after a base reboot, but the outbound `/cmd_vel` path caches
  the base's DDS GUID — so **after rebooting the Create 3 you MUST restart
  `turtlebot4.service`** or commands silently go nowhere.

- **The Create 3 has a `/cmd_vel` watchdog: it only drives while receiving a
  steady command stream (~10–20 Hz).** Single or short `ros2 topic pub` pulses
  won't visibly move it. The navigator's control loop publishes at ~20 Hz,
  which satisfies it. Reboot the base via its HTTP API:
  `curl -X POST http://${TB4_BASE:-192.168.186.2}/api/reboot` (run from the Pi;
  the base is at `192.168.186.2` on `usb0` by default — override with `TB4_BASE`
  env if yours differs — ~40–60 s to come back).

- **A latched safety stop makes the base ignore `/cmd_vel` with
  `wheels_enabled: false` even when not docked** — usually from the robot being
  picked up/handled (wheel-drop/kidnap reflex). `/wheel_status` shows
  `wheels_enabled: false`, `pwm: 0` under a steady command stream. Clear it by
  rebooting the base (HTTP `/api/reboot`) then restarting `turtlebot4.service`
  (re-handshake). `chime.sh --base` does the whole sequence.

- **After a power/dock cycle the RPLIDAR motor can stay stopped → `/scan` has a
  publisher but publishes NO data** (`ros2 topic echo /scan` is empty; the
  navigator BEV is just the empty grid). Spin it back up:
  `ros2 service call /start_motor std_srvs/srv/Empty` (`/stop_motor` halts it).
  This is the *lidar* motor — nothing to do with the wheels. The navigator also
  self-heals this when undocked (it calls `/start_motor` on stale scans).

- **Watch the battery.** It drains fast under driving/spinning (saw 96%→49%
  across one session) and the Pi+base drop off WiFi when it gets low. Dock to
  charge; `/battery_state.percentage` ×100 is the level.

- **Develop while docked** for anything that doesn't need motion: the wheels
  are disabled on the dock (`wheels_enabled: false`, won't drive), but
  lidar/odom/camera publish and the planner (cost grid + A* + path smoothing)
  can be verified live without undocking.

- **The RPLIDAR is mounted yaw +90° from base_link** (`tf2_echo base_link
  rplidar_link` → RPY [0,0,90°]; base_x=-lidar_y, base_y=+lidar_x). Any
  scan→cartesian must rotate by this: `forward=-r·sinθ, left=r·cosθ`.
  `build_grid` originally used the raw scan → the planner's whole obstacle map
  was 90° off reality, so it wove around phantom obstacles and refused straight
  shots (it still reached goals in *open* space, which hid it). Fixed
  2026-06-15; `build_grid` and `render_lidar_png` now share the same transform.

- **The BEV once had a mirrored Y-axis split between layers** (fixed
  2026-06-23): the lidar layer pre-negated y while odom-anchored elements
  (goal, path, detections) went straight through `w2p`, so they rendered
  mirrored vs the lidar and BEV clicks landed opposite the real walls. All
  layers now use base_link (+x fwd, +y left) through one `w2p`. Driving was
  always correct (control uses odom math, not the render).

- **`/wheel_status` is the ground-truth motor debug topic** — `pwm_left/right`
  = commanded drive, `current_ma_left/right` + `wheels_enabled` = actual
  actuation. Far more reliable than inferring motion from odom. Pitfall: don't
  verify rotation via `odom.pose.orientation.z` near yaw≈±180°, where
  `z=sin(yaw/2)` is at its extremum and barely changes — read yaw (Euler) or
  `.w` instead.

- **Fast-DDS shared memory on this Pi is flaky** — rapid successive `ros2` CLI
  calls throw `rcl node's context is invalid`. Export
  `FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Workspace/turtlebot4-glassbox/fastdds_no_shm.xml`
  (the run scripts and systemd units do this).

- **OAK-D spatial YOLO — use the custom launch (`oakd_rgbd.launch.py`), not the
  turtlebot4_bringup wrapper.** The wrapper silently drops `oakd_lite.yaml`
  params (node-name/key mismatch) and falls back to `Pipeline type: RGB`;
  spatial NN then has no stereo depth and the camera publishers don't come up.
  The `tb4-oakd` systemd unit owns this launch on boot.
  - The OAK-D Lite is USB-bus-powered on the Pi4 (shared 1.2 A), so the
    RGBD+YOLO load can brown out / starve the XLink. Fix = slash bandwidth:
    `camera.i_usb_speed: HIGH` (USB2), diagnostics off (kills the
    `sys_logger_queue` X_LINK spam), `nn.i_enable_passthrough: False` (the
    `nn_pt` stream SIGABRTs the pipeline), low-bandwidth + low fps on rgb and
    stereo — see `oakd_rgbd.launch.py` for the current values. Prefer lowering
    `rgb.i_fps` over downgrading the model when the link is stressed.
  - Harmless leftover: `VideoEncoder ... Arrived frame type (14) is not
    NV12/YUV400p` warning from low-bandwidth RGB encoding; preview publishes.
  - To read the driver's real logs, capture stdout (`output='screen'` +
    `emulate_tty` → `~/oak_rgbd.log`); the composable node hides them in
    `~/.ros/log/` otherwise.
  - **OAK-D can wedge as `X_LINK_DEVICE_ALREADY_IN_USE`** (a zombie holds the
    USB handle; `pkill` of the container often won't free it). Make sure only
    ONE pipeline owner is running (systemd unit vs a hand-run `run_oakd.sh`);
    a Pi reboot reliably clears a true wedge.

- **Bump/cliff/wheel-drop feedback needs `/hazard_detection` bridged**, which
  is commented out by default in
  `/opt/ros/jazzy/share/turtlebot4_bringup/config/republisher.yaml`. Uncomment
  the `"hazard_detection", "irobot_create_msgs/msg/HazardDetectionVector",`
  line (sudo) and restart `turtlebot4.service`. The navigator subscribes and
  emergency-stops on BUMP/CLIFF/WHEEL_DROP. This firmware publishes
  `/hazard_detection` event-driven (silent when clear), so the nav auto-clears
  the banner ~2 s after the last msg. (System file, not in the repo —
  re-apply on a fresh image.)
