# turtlebot4-glassbox

Vision-driven navigation for a **TurtleBot 4** (iRobot Create 3 base + Raspberry
Pi 4 + RPLIDAR + OAK-D Lite), driven from a phone-friendly web UI with a small
task-scripting DSL on top.

> **Hardware split:** the dev Mac has **no ROS 2**. All ROS code runs on the
> robot's Pi over SSH (ROS 2 **Jazzy**). The Create 3 base runs its own firmware;
> a `create3_republisher` (part of `turtlebot4.service`) bridges the two.
> Hard-won operational notes live in `docs/operations.md`.

> **Glass-box?** Because you can watch it think. The web UI shows the live
> occupancy grid, the planned path, every detection, the push line and the
> running skill — and each drive is recorded and replayable as video. Nothing
> here is a black box: the behaviors are classical planning (A\* + pure pursuit)
> and geometry, sequenced by a small task DSL. *(Formerly `turtlebot-rl` — it
> began as an RL experiment; no reinforcement learning survives in it today.)*

## What it does

A single ROS 2 node + Flask web server (`tb4_claude_nav.py`) turns vision + lidar
into autonomous behaviors:

- **YOLOv8** runs on the OAK-D's VPU (80 COCO classes); detections are projected
  to the robot frame with a **p25 stereo-depth** distance (plus the lidar range
  at the same bearing).
- **A\*** path planning on a lidar occupancy grid + pure-pursuit driving.
- A tiny **task DSL** (`toyscript.py`) sequences named skills (`SCAN_FOR`,
  `PUSH_AWAY`, `PUSH_TO_GOAL`, `PUSH_THROUGH`, `FOLLOW`, …) into `.toy` programs
  under `programs/`.

## Behaviors (`programs/`)

| program | what it does |
|---|---|
| `push_ball_to_goal` | find the ball and push it to **the spot you marked**. Pick the goal, then run it. |
| `push_ball_to_wall` | find a ball, centre it, push it to the wall (camera-only). No goal needed — **the field-proven default.** |
| `push_to_wall` | same task with a lidar wall-goal + camera steering + lidar close-range ball tracking (camera+lidar fusion). Experimental. |
| `follow` | follow **anything** you name from the COCO list (person, dog, cat, ball, bottle, chair, cup, …), or just "follow me" / "help me carry" for the closest person. Routes around furniture and never drives into the target. Distance and technique adapt to what you named: a big target (person, dog) is followed at once from a 1 m standoff; a small low one (ball, bottle, cup) from 0.7 m, acquired with a step-and-stare sweep first — and say "…aggressively" / "track the ball" / "predict the ball" to also use the lidar + velocity predictor that bridges the camera's gaps. |
| `open_explore` | roam toward the most open space, avoiding obstacles. |

Trigger by phrase (e.g. "push the ball to the goal", "push the ball to the wall",
"follow me") or exact name.

### Pushing the ball to a goal you pick

1. Tick **🎯 Ball goal** in the web UI and click the BEV where the ball should
   end up (the marker is amber, labelled `BALL GOAL`). The robot does **not**
   move — it is just a mark. (`POST /set_ball_goal {"x":…,"y":…}`, `{"clear":true}`
   to unmark.)
2. Run the task **"push the ball to the goal"** (or `PUSH_TO_GOAL(x, y)` in a
   `.toy` to hard-code the point in odom metres).

Each round the robot finds the ball and marks it, works out the goal↔ball line,
drives to the point **behind** the ball on that line (`STANCE` on the BEV),
creeps the last metre *along* the line so it arrives square, and shoves — then
re-measures and repeats. Because every round is re-aimed from a fresh fix, a
crooked shove is corrected by the next one instead of compounding, so it
converges on the goal. The dashed amber line on the BEV is the push line; the
robot stops and re-lines-up if the ball veers off it.

Returns `at goal` / `stalled` (jammed) / `lost` / `blocked` / `no-goal`.

> **Status: works on the robot.** First real drive succeeded 2026-09-19 — the
> ball was pushed to the marked goal. The line-up + shove loop is also exercised
> closed-loop in `tests/test_push_to_goal.py` (a simulated base that rolls the
> ball), which is the fast way to check a change to the geometry or the push
> controller without undocking.

## Running it (on the Pi)

```bash
ssh ubuntu@<pi>
cd ~/Workspace/turtlebot4-glassbox
bash scripts/download_models.sh   # ONCE per clone: fetch the OAK-D YOLO blobs
bash run_oakd.sh     # OAK-D camera + on-VPU YOLO   (logs ~/oak_rgbd.log)
bash run_nav.sh      # navigator + web UI on :5000  (logs ~/nav.log)
```

The blobs are **not** committed — they are Release assets, so the clone stays
small and the repo doesn't redistribute AGPL-licensed weights (see
[NOTICE](NOTICE)). `run_oakd.sh` tells you if they're missing.

Open `http://<pi>:5000`. Pre-flight is one glance — the **Health** row shows
`cam · lidar · base` dots; **3 green = go**. Undock, then run a behavior.

`chime.sh` is one-command recovery from ANY state: it converges to
cam+lidar+base all live (rebooting the Create 3, restarting `turtlebot4.service`,
spinning the lidar motor, restarting nav — only the steps actually needed) and
guarantees the READY chime. Add `--base` to force a Create 3 reboot up front
(see `docs/operations.md`).

## Reliability

- **Self-heal:** if the lidar motor stops (the TB4 stops it on dock) or the
  OAK-D freezes (USB2), the navigator restarts them automatically while idle.
- **Health badge:** `cam_ok`/`lidar_ok`/`base_ok` in `/state` surface sensor
  freshness so a problem is visible before a run.

## Glass-box recording

Arm recording in the UI; each drive is saved to `datasets/drive_*/` (per-step
lidar grid, planned paths, detections, camera frame, and the running skill).
Render a `camera | BEV | skill` video:

```bash
python3 dataset_tools.py datasets/drive_YYYYMMDD_HHMMSS --video
```

## Tests

No ROS needed — the ROS message packages are stubbed and the navigator is
imported for real, then driven against a simulated base:

```bash
pip install -r requirements.txt
python3 tests/test_push_to_goal.py          # 23 tests, ~3 min (real-time sim)
python3 toyscript.py programs/push_ball_to_goal.toy   # DSL + MockRobot smoke run
```

## Security

The web UI (`tb4_claude_nav.py` Flask on `:5000`) has **no authentication** — it is
intended for a trusted Wi-Fi LAN (your home / lab). Anyone on the same network
can `POST /goto_xy`, `/run`, `/dock`, etc. and drive the robot. **Do not expose
port 5000 to the internet**; if you need remote access, use an SSH tunnel
(`ssh -L 5000:localhost:5000 ubuntu@<pi>`) or put the Pi behind a VPN. To restrict
to localhost only, change `app.run(host='0.0.0.0', ...)` to `host='127.0.0.1'`.

## Layout

- `tb4_claude_nav.py` — ROS 2 node + Flask UI (`templates/index.html`) + nav loop + behaviors
- `toyscript.py` — the task DSL interpreter (+ a `MockRobot` for offline tests)
- `programs/*.toy` — the behaviors
- `tests/test_push_to_goal.py` — closed-loop tests for the ball-push geometry and
  `PUSH_TO_GOAL` (ROS stubbed, no hardware needed)
- `dataset_tools.py` — load / inspect / render recorded drives
- `oakd_rgbd.launch.py` — OAK-D spatial-YOLO launch (custom; bypasses bringup)
- `run_nav.sh` / `run_oakd.sh` — launch;  `chime.sh` — one-command recovery + chime
  (honors `TB4_HOST`, `TB4_BASE`, `TB4_ROOT`, `TB4_SUDO_PW` env vars — never
  committed; `run_oakd.sh` also honors `TB4_OAKD_MODEL` to pick the detector)
- `services/` — systemd units: everything auto-starts on boot (+ ready chime)
- `models/` — `nn_base.json` (the shared decode config: COCO-80 labels +
  thresholds) and `DEFAULT_MODEL` (which blob to launch). `oakd_rgbd.launch.py`
  combines the two at launch and writes the per-blob config to `/tmp`, so nothing
  committed carries an absolute path. The `.blob` files themselves are Release
  assets fetched by `scripts/download_models.sh`, not committed; source `.pt`
  weights are gitignored (ultralytics re-downloads them)
- `scripts/download_models.sh` — fetch + SHA-256 verify the OAK blobs
- `docs/operations.md` — operational notes (`/cmd_vel` recovery, OAK gotchas)
- `templates/index.html` — the web UI, served at `/` (kept out of the Python so
  it is lintable; Flask resolves it relative to the module, so it works from any
  cwd)
- `devtools/` — operator scaffolding, **not** imported by anything: a standalone
  chime + drive-leg loop driven off nav's `/state`, used to validate `/cmd_vel`
  end-to-end. See `devtools/README.md`.
- `fastdds_no_shm.xml` — Fast-DDS profile that disables shared-memory transport;
  this box's SHM is flaky and caused "rcl node's context is invalid" crashes.
  Every launch script exports it as `FASTRTPS_DEFAULT_PROFILES_FILE`.
- `training/` and `objdet18/` — optional custom-detector training pipelines
  (needs `ultralytics`; see the licensing note below)

## License

Project code is **MIT** — see [LICENSE](LICENSE).

⚠️ **The model blobs are not MIT.** `yolov5mu_416_5shave.blob` and
`yolov8s_416_fixed_6shave.blob` are compiled exports of **Ultralytics** YOLO
weights, which are **AGPL-3.0** (Ultralytics also sells a commercial Enterprise
License). They are distributed as Release assets rather than committed here, so
this repository's tree is MIT throughout — but the blobs you download are not,
and neither is anything you build from them. The same applies to the
`training/` and `objdet18/` scripts that import `ultralytics`. The navigator
itself does not — inference runs on the OAK-D's VPU from a compiled blob —
which is why `ultralytics` is not in the default `requirements.txt` install.

If you plan to use this commercially or in a closed-source product, read
[NOTICE](NOTICE) first: it spells out what each third-party component
(Ultralytics, COCO, Objects365, ROS 2, DepthAI) actually requires.
