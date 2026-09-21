# services/ — auto-start on boot + "ready" chime

Power on the robot → the OAK camera and the nav control server come up on their
own, and the Create 3 **plays Twinkle-Twinkle when everything's live** so you
know it's ready to drive — no SSH needed.

## What starts (and in what order)
1. `turtlebot4.service` — lidar + Create 3 bridge (already ships on the robot)
2. **`tb4-oakd.service`** — the OAK-D YOLO camera pipeline
3. **`tb4-nav.service`** — the nav control server (web UI on `:5000`) + startup chime

## Install (once, on the Pi)
```bash
cd ~/Workspace/turtlebot4-glassbox && git pull
bash services/install.sh
sudo systemctl start tb4-oakd tb4-nav      # start now (or just reboot)
```

## The sound tells you the state
- **🎵 Twinkle-Twinkle** (`C C G G A A G, F F E E D D C`) → **cam + lidar + base all live — ready to drive.**
- **descending 3-note tone** → came up but something's missing; the web UI status says which (`missing: cam` / `lidar` / `base`).
- **no sound at all after ~2 min** → the base/bridge is down (this is the "sees but can't move" case) — reboot the Create 3 base, then `sudo systemctl restart turtlebot4.service`.

## ⚠️ Important caveat re: "sees, plans, but can't move"
The chime confirms **cam/lidar/base health** (odom is streaming), but it does
**not** prove `/cmd_vel` reaches the wheels. That bridge can go stale even with
odom live — the known fix is rebooting the **Create 3 base** then restarting
`turtlebot4.service`. On a fresh power-on the base is fresh, so this is rare;
it mostly bites after service-only restarts. (Possible improvement: a distinct
"commanded-but-not-moving" alert tone hooked to the stall watchdog, for audible
feedback on that case too.)

## Handy
```bash
sudo systemctl status tb4-nav          # is it up?
journalctl -u tb4-nav -f               # live logs
sudo systemctl restart tb4-nav         # restart control server (re-chimes)
sudo systemctl disable tb4-nav tb4-oakd  # stop auto-start
```
