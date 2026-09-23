# devtools/ — operator scaffolding, not part of the product

Nothing here is imported by the navigator, the web UI, or the tests. These are
one-off scripts written to answer a specific question on a specific day about
the robot in front of us; they are kept because the questions come back.

| script | what it was for |
|---|---|
| `goal_cycle.py` | Converge to cam+lidar+base all live and chime, driven purely off nav's `/state`, then nudge the base a fixed distance in odom. Probed whether `/cmd_vel` was actually reaching the wheels — the "sees, plans, but can't move" failure mode in `docs/operations.md`. |
| `goal_run.py` | Runs `goal_cycle.ensure_ready()` + a real drive leg for N cycles, shuttling between two odom points and logging each cycle to `~/goal_run.jsonl`. The endurance version of the same test. |

Both talk to the navigator over HTTP only (`TB4_NAV`, default
`http://127.0.0.1:5000`), so they run from anywhere that can reach the Pi —
they need no ROS of their own except `goal_cycle.twinkle()`, which publishes
straight to `/cmd_audio`.

```bash
python3 devtools/goal_cycle.py status          # dump /state as JSON
python3 devtools/goal_cycle.py ready           # converge + chime
python3 devtools/goal_cycle.py drive 0.4       # nudge 0.4 m along current heading
python3 devtools/goal_cycle.py dock
python3 devtools/goal_run.py 10 1 0.0 0.0 0.7 0.0   # 10 cycles from #1, home/away
```

Read the docstrings before trusting the numbers: they record what was learned
the hard way (`/state` reports `odom_yaw` in **degrees**; waypoints must be more
than 2× nav's arrival tolerance or the robot is "already arrived" and never
moves; `ros2 topic hz` lies under the no-SHM profile, so never gate a decision
on it).
