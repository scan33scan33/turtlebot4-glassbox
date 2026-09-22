# Simplification plan

Audit of `turtlebot4-glassbox` at `71879e2`, 2026-09-21.

> **Status.** **Tier 1 is done** — §1.1, §1.3, §1.4, §1.5, §1.7, §1.8, §1.9;
> §1.6 skipped by choice, §1.2 deferred because it depends on §2.6. Each section
> header carries its status and commit. Tiers 2 and 3 are untouched.
>
> Tier 1 turned up two things the audit missed, and both are now closed:
> **§3.5 — `/run`'s generic-follow special case shadowed three programs' own
> declared triggers**, leaving the `FOLLOW_FAST` skill unreachable by the phrases
> that advertised it; and a `_FOLLOW_VERBS` / `.toy`-trigger vocabulary drift
> that made `follow_dog.toy` look un-mergeable. Consolidating the four
> `follow*.toy` programs into one (§1.1) resolved both, because the special case
> was only lossy while the program it routed to could not express what the
> shadowed programs expressed. 9 `.toy` programs → 5.
>
> Test suite went 23 → 29 and is green. The follow change alters robot behaviour
> for ball and small-object phrases — deliberately, restoring tuning that the
> routing bug was discarding; §1.1 enumerates every delta.

## Baseline

| | |
|---|---|
| Files tracked | 60 |
| Working-tree size | 837 KB (**396 KB of that is `posters/`**) |
| Python | 3 615 lines source + 477 lines of tests |
| Shell | 435 lines across 8 scripts |
| `.toy` programs | 188 lines across 9 files |
| Test suite | `tests/test_push_to_goal.py` — 23 tests, **currently green** (verified: `Ran 23 tests in 158.9s … OK`) |

The repo is not big in absolute terms. What makes it *feel* complicated is
concentration and duplication:

```
tb4_claude_nav.py = 2346 lines   (60% of all Python in the repo)

  127      1-127   imports + tuning constants       ##########
   72    128-199   Flask app + _state dict          ######
   74    200-273   geometry helpers (pure)          ######
  113    274-386   grid + A* + smoothing            #########
  230    387-616   BEV / camera rendering           ###################
  274    617-890   NavNode (ROS) + find_target      ######################
   76    891-966   DriveRecorder                    ######
  262    967-1228  nav_loop()                       #####################
  720   1229-1948  NavRobot skills                  ############################################################
  109   1949-2057  program load/match/run           #########
  213   2058-2270  Flask routes                     #################
   76   2271-2346  chime + main()                   ######
```

Twelve unrelated concerns in one file, two of them (`NavRobot` at 720 lines and
`nav_loop` at 262) larger than every other Python file in the repo combined.

**Safety net:** the existing tests exercise push geometry, `push_to_goal`,
the Flask routes, program loading/matching and BEV rendering — by importing the
real module with ROS stubbed. That covers Tier 1 and most of Tier 2 below.
It does **not** cover the follow *loops*, `explore_open`, `push_to_wall`,
`nav_loop`'s replanning, the chime, the shell scripts, or either training
pipeline. Tier 3 touches exactly those, so it needs care (or new tests first).
(Since the audit, follow *parameter resolution* is covered — `_follow_params` is
pure and table-tested, which is the part §1.1 changed; the loops it feeds are
not.)

---

## Tier 1 — mechanical, near-zero risk

### 1.1 Four `.toy` programs were one behaviour with four sets of constants  ✅ *done — 9 programs → 5*

```
follow.toy                SET r = FOLLOW("person", 1.0)
follow_human.toy          SET r = FOLLOW("person", 1.0)   <- byte-identical
follow_dog.toy            SET r = FOLLOW("dog",    1.0)   <- one string differs
follow_ball.toy           SCAN_FOR("ball",360) ; FOLLOW("ball", 0.7)
follow_ball_aggressive.toy SCAN_FOR("ball",360) ; FOLLOW_FAST("ball", 0.7)
```

They differed on exactly three axes, and all three follow from *what you named*,
not from which file you happened to land in:

| axis | big target (person, dog) | small, low target (ball, bottle, cup) |
|---|---|---|
| standoff | 1.0 m | 0.7 m — at 1 m the 416-px preview loses a ~40 cm object |
| acquisition | start `FOLLOW` at once | `SCAN_FOR` step-and-stare first: `FOLLOW`'s re-acquire is a *continuous* spin, which the OAK detects poorly through, so it sails past a ball |
| controller | `FOLLOW` | `FOLLOW` + the lidar/velocity predictor when asked to go hard |

**So all four are now one `follow.toy` that calls `FOLLOW()` with no arguments**,
and the three axes are resolved in one place — `_follow_params(text)` →
`{target, standoff, scan_first, fast}` — passed through state and surfaced in
`/state` under `follow`, so you can see what it decided. `FOLLOW` takes
`(name, standoff, fast)`; `FOLLOW_FAST` is no longer a DSL verb.

Two vocabulary drifts died with this:

| drift | before | after |
|---|---|---|
| `'track'` was in programs' `# triggers:` but not in `_FOLLOW_VERBS` | "track the dog" worked *only* via a literal trigger in `follow_dog.toy` | one verb list; `track` parses |
| each program hard-coded its own target string | 4 copies to keep in sync | 0 — the parser supplies it |

**Verified** by re-deriving the effective `(target, standoff, fast, scan_first)`
4-tuple for 46 phrases against the pristine base commit, tracing the executed DSL
calls in both trees:

- **32 phrases identical**, including every dog, person and cat phrase.
- `track the dog` → `('dog', 1.0, fast=False, scan=False)` — **unchanged**. This
  was the regression that blocked the merge; it is fixed, not worked around.
- `track the ball` / `predict the ball` → still `fast=True, scan_first=True,
  0.7 m`. The predictor skill stays reachable; only the target string moves to
  the canonical COCO label (`'ball'` → `'sports ball'`), which `_live_det`
  matches identically.
- **14 phrases changed, all of them ball/small-object, all of them fixes**: the
  `/run` special case described in §3.5 used to hand `follow the ball`,
  `chase the ball`, `follow the bottle`, `follow the ball aggressively` … to the
  *generic* program at a 1 m standoff with no scan and no predictor, silently
  discarding the tuning in `follow_ball*.toy`. They now get the behaviour those
  files were written for.
- Non-follow phrases (`push …`, `explore`, `roam`, `find the red ball`, `dock`)
  untouched.

`fast` is gated on `_is_close_target()`, so "follow the dog fast" cannot reach the
predictor — its lidar model hunts a close convex blob on the floor and is simply
wrong for a person. Likewise a stray speed word with no object named
("go fast", "track my package") resolves to nothing rather than inventing a ball.

Tests 25 → 29: `test_every_follow_phrase_reaches_the_one_program`,
`test_follow_program_hardcodes_no_target`, `test_track_the_dog_still_follows_a_dog`,
`test_small_targets_get_the_close_standoff_and_a_scan_first`,
`test_the_lidar_predictor_stays_reachable_for_balls_only`,
`test_a_fast_word_alone_does_not_invent_a_target`.

### 1.2 `push_ball_to_wall.toy` and `push_to_wall.toy` are the same strategy written twice  ⏳ *deferred — needs §2.6 first*

`push_ball_to_wall.toy` is 62 lines of DSL orchestrating `SCAN_FOR → FIND →
PUSH_AWAY (loop) → PUSH_THROUGH`. `push_to_wall.toy` is 20 lines calling one
skill, `PUSH_TO_WALL`, which implements the same scan→shove→re-acquire loop in
Python (plus lidar fusion the DSL can't express).

Two implementations of one behaviour, in two languages, that must be kept in
step by hand.

**Do:** keep `PUSH_TO_WALL` as the single home (it's the better strategy — the
README calls the camera-only version "field-proven" and the fusion one
"experimental", but the fusion one is strictly a superset). Both of the `.toy`'s
extra behaviours already exist in the skill: the "ball at our feet →
`PUSH_THROUGH`" case is at `push_to_wall():1485`, and the `FIND("apple")`
mis-detection fallback is handled inside `_live_det()` and `_scan_hit()` (§2.6).
So `push_ball_to_wall.toy` is 62 lines of DSL re-deriving what the skill already
does — retire it and keep `push_to_wall.toy`.

If you'd rather keep the field-proven one as default, do the reverse — the
point is to pick **one** place for the strategy.

### 1.3 The COCO 80-label list exists four times  🟡 *4 → 2 in `ab10d9a`; 2 → 1 wants §3.1*

| copy | where |
|---|---|
| 1 | `tb4_claude_nav.py:COCO_LABELS` (Python list, ~15 lines) |
| 2 | `models/nn_yolov5mu.json` → `mappings.labels` |
| 3 | `models/nn_yolov8n.json` → `mappings.labels` |
| 4 | `models/nn_yolov8s.json` → `mappings.labels` |

`models/YOLOV8S_SWAP.md` even says the quiet part out loud: *"Labels
byte-identical to `nn_yolov5mu.json`"*.

The three JSONs are identical except for `model.model_name` — the blob path.
So three files, ~2.4 KB each, carry one distinguishing string.

**Do:** make one source of truth. Either
- `models/nn_config.template.json` + a `--model {v5mu,v8s,v8n}` generator in
  `scripts/download_models.sh` (which already knows where it put the blob), or
- generate them at launch: `oakd_rgbd.launch.py` already computes `_ws` and
  joins the config path, so it can write the JSON with the correct
  `model_name` into a runtime dir instead of reading a committed one.

### 1.4 …and those JSONs hardcode one user's home directory  ✅ *done — `ab10d9a`*

```json
"model_name": "/home/ubuntu/Workspace/turtlebot4-glassbox/models/yolov5mu_416_5shave.blob"
```

Every shell script in the repo goes out of its way to be portable
(`ROOT=${TB4_ROOT:-…}` appears in four of them, and `run_nav.sh` has a comment
specifically about keeping the convention consistent across files). The nn
configs quietly defeat all of it: clone anywhere but `~/Workspace/…` on a user
named `ubuntu` and the OAK-D fails to load the blob.

`oakd_rgbd.launch.py` *does* resolve `TB4_ROOT` for the config path — it just
can't fix the absolute path baked *inside* the config. Fixing 1.3 fixes this
for free. **This is the only item in this plan that is a live bug, not just
duplication** — worth doing first.

### 1.5 `nn_yolov8n.json` configures a model that doesn't exist  ✅ *done — `ab10d9a`*

`models/README.md`: *"template config for a YOLOv8n build (**blob not yet
exported** — run `training/export_for_oak.py` to generate …)"*. It is not
referenced by any code — only by a `# fallback:` comment in the launch file
pointing at a file that can't work.

**Do:** delete it until the blob exists. Re-adding a generated config is one
command once 1.3 lands.

### 1.6 `posters/` is 47% of the working tree and is a build artifact  ⏸️ *skipped by choice*

| | |
|---|---|
| `poster-algorithms.pdf` | 185 KB |
| `poster-public.pdf` | 178 KB |
| both `.html` sources | 27 KB |

`posters/README.md` documents exactly how to regenerate the PDFs from the HTML
(`chromium --headless --print-to-pdf`), and calls the HTML "the source of truth
— edit this". So the PDFs are compiled output sitting next to their source.

The repo already has the right convention for this: model blobs are Release
assets, deliberately not committed, *"so the clone stays small"*.

**Do:** same treatment — keep the two `.html`, move the two `.pdf` to a Release
(or just delete them; anyone can render). Working tree 837 KB → ~470 KB. Also
removes the drift trap the README warns about ("if those change, update the
posters").

### 1.7 `goal_cycle.py` / `goal_run.py` are one-off debug scaffolding at the repo root  ✅ *done — `c9c5359`*

178 lines. The README describes them as *"a shuttle test used to validate
`/cmd_vel` end-to-end; **not imported by the navigator**"*. Their docstrings are
dated field notes (`Design notes (2026-08-20)`, *"Fixes from the earlier runs"*).
Nothing imports them except each other; they appear in CI only because the
`compileall` step lists every file by hand.

They are the reason the repo root has six loose Python files and reads like a
scratch directory.

**Do:** move to `devtools/` (or delete — git history keeps them). Root becomes:
`tb4_claude_nav.py`, `toyscript.py`, `dataset_tools.py`, three `.sh`, one launch
file, one XML. That single change does more for "feels complicated" than any
line-count reduction.

### 1.8 `_HTML` gives the index route two render paths  ✅ *done — `f330c3e`*

```python
_TEMPLATE_PATH = os.path.join(os.path.dirname(__file__), "templates", "index.html")
try:
    with open(_TEMPLATE_PATH) as _tf: _HTML = _tf.read()
except FileNotFoundError:
    _HTML = """<!DOCTYPE html>…Template missing…</html>"""
...
@app.route('/')
def index():
    try:    return render_template("index.html")
    except Exception: return render_template_string(_HTML)
```

The same file is read twice by two mechanisms, and the bare `except Exception`
swallows genuine Jinja errors and re-raises them from a different renderer.

**Do:** `render_template("index.html")` only. If you want the friendly failure,
assert the template exists once at startup. Drops `_HTML`, `_TEMPLATE_PATH` and
the `render_template_string` import.

### 1.9 CI enumerates files instead of globbing them  ✅ *done — `c178be7`*

`.github/workflows/tests.yml` hardcodes two lists:

```yaml
python -m compileall -q tb4_claude_nav.py toyscript.py dataset_tools.py goal_cycle.py \
  goal_run.py oakd_rgbd.launch.py tests training objdet18
for f in chime.sh run_nav.sh run_oakd.sh scripts/download_models.sh \
         services/install.sh services/tb4-oakd-run.sh \
         objdet18/run_all.sh objdet18/run_coco_only.sh; do bash -n "$f"; done
```

Every new file must be remembered in two places, and a forgotten one silently
stops being checked. (Note `.toy` parsing already globs correctly — the pattern
is right there in the same file.)

**Do:**
```yaml
python -m compileall -q $(git ls-files '*.py')
git ls-files '*.sh' | xargs -n1 bash -n
```
Never goes stale, and shrinks with every file deleted above.

---

## Tier 2 — small extractions inside the monolith (test-covered)

These are pure de-duplication. Each is ~10–40 lines out and removes a category
of bug rather than just some noise.

### 2.1 `local → odom` rotation is hand-written seven times

`_odom_to_local()` exists at line 217 and is used (and round-trip-tested at
`test_odom_local_round_trip`). Its **inverse does not exist**, so every caller
that needs robot-frame → odom re-derives the rotation matrix inline:

```
1044   goal_odom = (ox + gx_loc*math.cos(oyaw) - gy_loc*math.sin(oyaw), …)   nav_loop
1113   look_odom = (ox + look_x*math.cos(oyaw) - look_y*math.sin(oyaw), …)   nav_loop
1292   return {'x': round(ox + xl*math.cos(oyaw) - yl*math.sin(oyaw), 3), …) NavRobot.find
1438   gx = ox + best[0]*math.cos(oyaw) - best[1]*math.sin(oyaw)             explore_open
1550   return (ox + xl*math.cos(oyaw) - yl*math.sin(oyaw), …)                _ball_odom
1811   px = ox + xl*math.cos(oyaw) - yl*math.sin(oyaw)                       follow  (was follow_human)
1856   bx = ox + det['x_loc']*math.cos(oyaw) - det['y_loc']*math.sin(oyaw)   follow_ball_fast
```

Seven copies of a sign-sensitive 2×2. The file already documents what happens
when one of them is wrong:

> *"The grid is in the robot/base frame, so the odom-frame goal delta MUST be
> rotated by -yaw into it — otherwise the A\* goal cell is only correct at
> yaw≈0 and lands in an obstacle once the robot turns."*

and `goal_run.py`'s docstring records the same class of mistake
(*"`/state` reports odom_yaw in DEGREES — using it raw … drove it into an
obstacle"*).

**Do:** add `_local_to_odom(pt, ox, oy, oyaw)` beside `_odom_to_local`, plus a
`_pose()` returning `(ox, oy, oyaw)` — that triple is fetched with
`_get('odom_x'), _get('odom_y'), _get('odom_yaw')` at **10 separate sites**.
Extend `test_odom_local_round_trip` to cover both directions. Then
`NavRobot._ball_odom` becomes `_local_to_odom((det['x_loc'], det['y_loc']), *_pose())`
and the other six collapse to one line each.

### 2.2 The goal-issue protocol has six copies, two of which are already the right abstraction

`NavRobot.go_to()` and `push_to()` (1258, 1264) are *exactly* the helper this
needs:

```python
def go_to(self, x, y):
    _set(goal_odom=(x, y), goal_mode='move', direct_goal=True, nav_active=True,
         path=[], goal_local=None, destination='', match_info='toyscript',
         status=f'navigating → ({x:.2f}, {y:.2f})')
    return self._wait()
```

Four other sites re-spell that same nine-kwarg `_set` by hand instead of calling
a shared setter:

| site | what it is |
|---|---|
| 1440 | `explore_open` |
| 1821 | `follow` (was `follow_human`) |
| 1888 | `follow_ball_fast` |
| 2161 | the `/goto_xy` Flask route |

All four pass the identical nine keys, just reordered and with different values.
Nothing enforces that — omit `path=[]` and the BEV keeps drawing the previous
goal's path over the new one, with no error anywhere.

The release side is messier and genuinely has three variants, so it should be
extracted as three named helpers rather than one:

```python
_set(nav_active=False, goal_odom=None); ros_node.stop()                     # 1449, 1826, 1836, 1892, 1894 — skill "hold/stop"
_set(nav_active=False, goal_odom=None, path=[], goal_local=None, status=…)  # 1832 — skill "lost target"
_set(nav_active=False, goal_mode='move', status=…, path=[])                 # 1130, 1136, 1165 — nav_loop "arrived/blocked"
```
(16 `nav_active=False` sites in total; the rest are route-level resets at
2147/2196/2218/2053 that also clear run state.)

**Do:** one `set_goal(gx, gy, mode, tag, status)` module-level function — the
single definition of the nine-key contract — then `go_to`, `push_to`,
`explore_open`, `follow`, `follow_ball_fast` and `/goto_xy` all become
one-liners. Add `hold()` / `lose_target(status)` for the two skill-side
releases; leave `nav_loop`'s arrival variants alone (they carry different
meaning). ~35 lines out, and a new skill physically cannot forget a key.

### 2.3 The stall watchdog is duplicated between two push skills

Identical six lines at `push_to_wall` (1529) and `_push_segment` (1702):

```python
cx, cy = _get('odom_x'), _get('odom_y')
if math.hypot(cx - prog_x, cy - prog_y) > PROG_M:
    last_prog = time.time(); prog_x, prog_y = cx, cy
elif time.time() - last_prog > STALL_TIMEOUT:
    …
```

plus the same four-variable init (`last_prog`, `prog_x`, `prog_y`, and the
`lost` counter) at both sites, and a third reset pattern
(`last_prog = time.time(); prog_x, prog_y = _get('odom_x'), _get('odom_y')`)
repeated three more times inside `push_to_wall` alone.

The comment at 1526 explains the subtlety — *"a per-tick threshold false-stalls
whenever the ball is off-centre"* — and that hard-won reasoning is now
duplicated too, so it can be fixed in one copy and missed in the other.

**Do:** a ~12-line `StallWatch` with `.moved()` / `.stalled()` / `.reset()`.

### 2.4 The Twinkle chime is implemented three times

| copy | where | how |
|---|---|---|
| 1 | `tb4_claude_nav.py:2274-2296` | `_NOTE` table + `READY_CHIME` seq + `_play_notes()` via the node's `audio_pub` |
| 2 | `goal_cycle.py:33-51` | same note table, same 14-note sequence, spins up its own `rclpy` node |
| 3 | `chime.sh:147-163` | same note table, same sequence, in a bash heredoc spinning up its own `rclpy` node |

Three copies of `NOTE={"C":523,…}` and the 14-note `SEQ`. `chime.sh` step 6
waits for the navigator's announcer to chime, then step 7 fires a *different*
implementation of the same tune as a fallback — so the two must agree or the
robot plays two different songs depending on which path won the race.

**Do:** extract `chime.py` holding `_NOTE`, `READY_CHIME`, `WARN_TONE` and a
`play_tune(notes, node_name=None)` that creates a transient publisher when given
no node. `tb4_claude_nav.py` and `goal_cycle.py` import it; `chime.sh` step 7
becomes `python3 "$ROOT/chime.py" ready`. ~25 duplicated lines out, and the
tune can't drift.

### 2.5 Four shell scripts re-derive the same environment

```bash
set +u; source /opt/ros/jazzy/setup.bash; set -u     # chime.sh, run_oakd.sh, tb4-oakd-run.sh
ROOT=${TB4_ROOT:-$HOME/Workspace/turtlebot4-glassbox} # run_nav.sh, run_oakd.sh, tb4-oakd-run.sh
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/fastdds_no_shm.xml"   # all four
```

`run_nav.sh` carries an explicit comment about this being a convention that must
be kept in sync by hand across five files. That's the duplication announcing
itself.

Worse, `run_oakd.sh` and `services/tb4-oakd-run.sh` are ~90% the same script —
identical ROS source, identical `ROOT`, identical export, an identical 5-line
blob preflight (flagged by the block-duplication scan), the identical two
`pkill`s and `sleep 12`. They differ only in the last line: `setsid … &` versus
`exec`.

**Do:** one `scripts/tb4_env.sh` exporting `tb4_root`, `tb4_source_ros`,
`tb4_require_blob`, `tb4_kill_stale_oakd`. Then:
```bash
# run_oakd.sh            (33 -> ~10 lines)
source scripts/tb4_env.sh; tb4_source_ros; tb4_require_blob; tb4_kill_stale_oakd
setsid stdbuf -oL -eL ros2 launch "$TB4_ROOT/oakd_rgbd.launch.py" > "$HOME/oak_rgbd.log" 2>&1 </dev/null & disown

# services/tb4-oakd-run.sh  (24 -> ~5 lines)
source scripts/tb4_env.sh; tb4_source_ros; tb4_require_blob; tb4_kill_stale_oakd
exec ros2 launch "$TB4_ROOT/oakd_rgbd.launch.py"
```
435 shell lines → ~330, and the portability convention becomes enforced rather
than documented.

### 2.6 "The ball is sometimes labelled `apple`" is handled three different ways

COCO has no `ball` class, only `sports ball` — and in practice the VPU reports a
toy ball as `apple` often enough that the code compensates. But it compensates
inconsistently:

| where | mechanism | has the fallback? |
|---|---|---|
| `find(name)` :1279 | substring match on the label | **no** |
| `_scan_hit(name)` :1295 | wrapper: `if h is None and "ball" in name: h = self.find("apple")` | yes |
| `_live_det(name)` :1342 | inline predicate: `or ('ball' in q and 'apple' in lbl)` | yes |

Three lookup helpers, two fallback mechanisms, one of them absent from the
function the DSL's `FIND` primitive actually calls. That's why
`push_ball_to_wall.toy` writes `FIND("ball")` followed by an explicit
`IF … NONE THEN FIND("apple")` at *two* places — the program is papering over a
gap in `find()` from the outside.

The two matching predicates also differ: `_scan_hit` only retries when `name`
contains `"ball"`, while `_live_det` matches `apple` labels for any query
containing `ball` — and `find` additionally has a `q.split()[0] in lbl` clause
that `_live_det` lacks. So `find("red ball")` and `_live_det("red ball")` can
genuinely disagree about what's in front of the robot.

**Do:** one `_label_matches(query, label)` predicate plus one `_BALL_ALIASES`
set, used by all three. `find` then gets the fallback for free, `_scan_hit`
disappears, and the DSL-level `FIND("apple")` retries in `push_ball_to_wall.toy`
become unnecessary — which removes the last reason that file is longer than the
skill it calls.

---

## Tier 3 — structural (do after Tiers 1–2 have landed)

### 3.1 Split the monolith into a package

The twelve sections in the baseline table are already near-clean modules; only
`_state` and the tuning constants cross all of them.

```
tb4/
  config.py      constants + the _state dict + _get/_set/_lock        (~150)
  geometry.py    _quat_to_yaw, _wrap_angle, _odom_to_local,
                 _local_to_odom, push_line, push_stance,
                 stance_error, line_offset, line_signed_offset         (~90)  pure — no ROS, no numpy
  planning.py    build_grid, astar, _line_of_sight, smooth_path,
                 _add_detection_obstacles                              (~130) numpy only
  render.py      render_lidar_png, render_image_with_detections,
                 _detect_color                                         (~240)
  ros_node.py    NavNode + callbacks + find_target/_keyword_match      (~280)
  recorder.py    DriveRecorder                                         (~80)
  nav.py         nav_loop                                              (~260)
  skills.py      NavRobot                                              (~720)
  programs.py    _plan_steps, load_programs, match_program,
                 _run_program                                          (~110)
  web.py         Flask app + routes                                    (~220)
  chime.py       notes + _play_notes + _ready_announcer                 (~80)
  dsl.py         <- toyscript.py, moved in                             (~360)
tb4_claude_nav.py   thin main() shim:  from tb4.chime import main      (~10)
```

Keep the root `tb4_claude_nav.py` as a shim so `run_nav.sh`, `tb4-nav.service`
and every documented invocation keep working untouched.

**Why this is the real win:** `geometry.py` becomes importable with *no* ROS
stubbing at all — today `tests/test_push_to_goal.py` needs 60 lines of
`_stub_modules()` before it can test six pure functions. Once geometry and
planning are ROS-free modules, tests for them get trivial, which unlocks
covering the parts Tier 2 warned about.

**Sequencing:** move one section per commit, run the suite after each. Start
with `geometry.py` (smallest, purest, best covered), then `planning.py`,
`render.py`, `recorder.py`, `chime.py`, `dsl.py`, `config.py` — and leave
`nav.py`/`skills.py`/`web.py`/`ros_node.py` (the interdependent core) until last.

**Risk:** moderate but mechanical. The module-level `app`, `_state`, `ros_node`
globals and the `from tb4_claude_nav import *`-style access in the tests
(`tb4._get(...)`, `tb4.load_programs()`, `tb4._odom_to_local(...)`) are the
things that break. Mitigate by having the shim re-export the public names, so
`import tb4_claude_nav as tb4` keeps resolving everything it does today.

### 3.2 `follow` and `follow_ball_fast` are one skill with two target estimators  🟡 *rename done in 1.1; extraction still open*

Both loops are: *while not timed out → get a target estimate → if farther than
standoff, `drive_to` a point `standoff` short of it; else hold → if lost, sweep
to re-acquire.* The only real difference is **how the target position is
estimated**:

| | source |
|---|---|
| `follow` | latest camera detection, with a 6-tick grace then a turn-toward-last-seen-side sweep |
| `follow_ball_fast` | camera detection → lidar blob near the prediction → velocity extrapolation → decay to `scan_for` after 1.5 s |

**Do:** extract the estimator as a strategy object and share the outer loop:
```python
class CameraTarget:      def estimate(self) -> (x, y) | None …
class PredictedTarget:   # + lidar gate + velocity belief
def follow(self, target, standoff, max_time): …
```
~50 lines out, and a new follow-variant becomes an estimator rather than a copy
of a 60-line loop. Not test-covered today — worth adding a `FakeBase` follow
test (the harness already publishes a ball detection and integrates `cmd_vel`,
so a moving-target version is a small step).

**Done (in 1.1):** the rename. `follow_human` followed anything — it was called
with `"dog"` and `"ball"` and honoured `_get('follow_target')` for arbitrary COCO
classes, so `FOLLOW(...) → robot.follow_human` actively misled. It is now
`follow`, and it owns parameter resolution too: it reads `follow_target` /
`follow_standoff` / `follow_scan_first` / `follow_fast` from state, lets explicit
DSL arguments win, and delegates to `follow_ball_fast` when `fast` resolves for a
small target. `FOLLOW_FAST` is no longer a separate verb, so the predictor's
validity condition is checked in one place instead of at every call site.

What remains here is purely the inner-loop extraction above — a smaller and
better-defined job than before, since the two entry points now share one
signature and one set of resolved parameters.

### 3.3 `training/` and `objdet18/` are one pipeline, duplicated

| | `training/` | `objdet18/` |
|---|---|---|
| `train.py` | 42 lines | 36 lines — same argparse flags, same `YOLO(m).train(...)` call, same augmentation values (±`hsv_s`, `patience`, `close_mosaic`) |
| `data.yaml` | 2 classes | 19 classes |
| `README.md` | 407 words | 487 words — overlapping "deploying to the OAK-D" sections |
| `export_for_oak.py` | ✔ | — (prints `python ../training/export_for_oak.py`) |
| downloaders | `autolabel.py` | `download_coco.py`, `download_objects365.py`, `taxonomy.py` |
| runners | — | `run_all.sh`, `run_coco_only.sh` |

`objdet18/train.py` reaching into `../training/` for the exporter is the tell:
these were never two pipelines, they're one pipeline with two dataset configs.

`run_all.sh` and `run_coco_only.sh` duplicate each other too — same four env
vars, same `pip install ultralytics`, same two `download_coco.py` invocations,
same `train.py` call, differing in whether Objects365 is fetched.

**Do:**
```
training/
  train.py                    # one script; --data picks the dataset
  export_for_oak.py
  autolabel.py
  datasets/
    few_class.yaml            # was training/data.yaml
    coco_o365_18.yaml         # was objdet18/data.yaml
    download_coco.py
    download_objects365.py
    taxonomy.py
  run.sh                      # DATASET=coco|coco+o365 replaces both runners
  README.md                   # one, merged
```
Deletes `objdet18/` entirely. ~435 lines → ~330, six files fewer, one README,
one export path. Update `requirements.txt` (which points at both READMEs) and
the root README's License section (which names `objdet18/` twice).

**Note:** neither pipeline is test-covered and both need `ultralytics` (AGPL) +
a GPU, so verify by reading rather than running. `compileall` in CI will catch
import-path breakage.

### 3.4 Root-level layout after Tiers 1–3

```
tb4_claude_nav.py     10-line shim
run_nav.sh            ~15
run_oakd.sh           ~10
chime.sh              ~140
oakd_rgbd.launch.py   60
fastdds_no_shm.xml
dataset_tools.py      184
requirements.txt  README.md  LICENSE  NOTICE
tb4/                  the package
toyscript.py -> tb4/dsl.py
programs/  templates/  models/  scripts/  services/  training/  tests/  docs/  devtools/
posters/
```

### 3.5 A routing bug found while doing 1.1: `/run` shadows three programs' own triggers  ✅ *resolved by 1.1*

Not duplication — a defect, recorded here because 1.1 is what surfaced it and
because any follow-program consolidation depended on the answer.

`/run` had a special case ahead of the matcher:

```python
_ftgt = _parse_follow_target(data.get('text', ''))
if _ftgt and progs.get('follow'):
    matched, score = progs['follow'], 1.0        # generic 'follow the <COCO object>'
else:
    matched, score = match_program(data.get('text', ''), progs)
```

So **any** phrase naming a follow verb plus a COCO object went to the generic
`follow` program, and `match_program` — including a program's own verbatim
`# triggers:` — was never consulted. Measured against the then-shipping tree:

| you said | the `.toy` declared it as a trigger | `/run` actually started |
|---|---|---|
| "follow the dog", "follow dog", "chase the dog", "follow the puppy" | `follow_dog` | `follow` |
| "follow the ball", "follow ball", "chase the ball" | `follow_ball` | `follow` |
| "follow the ball aggressively", "aggressive ball follow" | `follow_ball_aggressive` | `follow` |
| "follow person" | `follow_human` | `follow` |

Only phrases where `_parse_follow_target` returned `None` reached
`match_program` — "track the ball", "predict the ball", "follow me",
"help me carry".

Consequences, in increasing order of seriousness:

1. `follow_dog` — harmless. The generic path resolved `follow_target='dog'` and
   ran `FOLLOW("dog", 1.0)`, which is exactly what `follow_dog.toy` did.
2. `follow_ball` — degraded. It wanted `FOLLOW("ball", 0.7)`; the generic path
   gave 1.0 m, so the robot trailed the ball **at 1.0 m instead of 0.7 m**, with
   no step-and-stare acquisition.
3. `follow_ball_aggressive` — effectively dead. It was the **only** caller of
   `FOLLOW_FAST`, the ~60-line velocity-belief + lidar-prediction skill, and both
   of its ball phrases were intercepted. The README advertised it as "chase the
   ball hard, predicting where it went with the lidar" — which is not what
   "follow the ball aggressively" did.

**None of the three options originally listed here was taken.** All of them kept
the behaviour decision in DSL text, so each one had to choose between the
matcher and the special case. The fix was to notice that the special case was
only *lossy* because the program it routed to could not express what the
shadowed programs expressed:

- one resolver, `_follow_params(text)`, now produces
  `{target, standoff, scan_first, fast}` from the phrase — the same three axes
  the four `.toy` files encoded as literals;
- `/run` calls it once, publishes it to state (and to `/state` under `follow`);
- `follow.toy` calls bare `FOLLOW()` and the skill reads the parameters.

The special case is still there and still wins — but it no longer throws
anything away, because it *is* the source of the parameters rather than a
shortcut past them. `match_program` remains the fallback for phrases that name
no COCO object ("follow me", "help me carry"), and the programs' `# triggers:`
lines are now documentation of what the parser accepts rather than a second
routing table that can disagree with it. §1.1's `'track'` drift was fixed in the
same pass by making `_FOLLOW_VERBS` the one list.

Behaviour deltas are enumerated in §1.1. Net: 3 unreachable/degraded programs
removed, the `FOLLOW_FAST` skill made reachable by the phrases that always
claimed to invoke it, and the ball standoff restored to its tuned value.

---

## Suggested order

| # | item | effort | risk | test-covered | status |
|---|---|---|---|---|---|
| 1 | **1.4 / 1.3** nn-config hardcoded path → generate | S | none | — (live bug) | ✅ `ab10d9a` |
| 2 | **1.9** CI globs instead of file lists | XS | none | ✔ | ✅ `c178be7` |
| 3 | **1.7** `goal_*` → `devtools/` | XS | none | ✔ | ✅ `c9c5359` |
| 4 | **1.8** drop the `_HTML` second render path | XS | none | ✔ | ✅ `f330c3e` |
| 5 | **1.6** posters `.pdf` → Release | XS | none | — | ⏸️ skipped by choice |
| 6 | **1.5** delete unused `nn_yolov8n.json` | XS | none | — | ✅ in `ab10d9a` |
| 7 | **1.1** merge the `follow*.toy` programs (all four → one) | S | **behaviour** | ✔ | ✅ done — also closes §3.5; see §1.1 for every delta |
| 8 | **2.1** `_local_to_odom` + `_pose()` | S | low | ✔ (extend round-trip test) | — |
| 9 | **2.5** `scripts/tb4_env.sh` | S | low | `bash -n` only | — |
| 10 | **2.3** `StallWatch` | S | low | partly | — |
| 11 | **2.2** `set_goal()` / `hold()` | S | med | partly | — |
| 12 | **2.4** single chime source | S | low | — | — |
| 13 | **2.6** one label-match predicate (+ ball/apple alias) | S | low | ✔ (`find` is DSL-reachable) | — |
| 14 | **3.3** merge `objdet18/` into `training/` | M | low | `compileall` only | — |
| 15 | **1.2** retire `push_ball_to_wall.toy` | M | med | ✔ — do after 13 | — |
| 16 | **3.1** split the monolith into `tb4/` | L | med | ✔ for ~half | — |
| 17 | **3.2** unify the follow skills | M | med | ✘ — add tests first | — |
| 18 | **3.5** decide the `/run` follow-shadowing routing | S | **med — changes robot behaviour** | ✘ — needs new tests | ⚠️ **needs your decision** |

Items 1–7 are done (5 deliberately skipped). 8–14 are each one
small commit with the suite green after. 14–17 want their own PRs.

**Net effect.** Measured for Tier 1 (which is *not* a size win — it fixed a bug,
removed duplicated config, and added documentation and regression tests), then
projected for the rest. The file *count* stays roughly flat overall because
splitting the monolith trades 1 file for ~13.

| metric | baseline `71879e2` | after Tier 1 (measured) | after everything (projected) |
|---|---|---|---|
| tracked bytes | 668 KB, 60 files | 722 KB, 57 files — **+54 KB**, of which +40 KB is this plan doc | ~390 KB if §1.6 lands, else ~710 KB |
| working tree | 837 KB | 878 KB | ~480 KB / ~840 KB |
| largest file | `tb4_claude_nav.py`, 2 346 lines | `tb4_claude_nav.py`, 2 486 | `tb4/skills.py`, ~760 |
| code lines (py+sh, excl. tests) | 4 050 | 4 276 (**+226**: launch-time config generation, preflight comments, and the follow resolver `_follow_params` + its rationale) | ~3 800 |
| test lines | 477 (23 tests) | 584 (29 tests) | ~740 |
| top-level loose files | 6 `.py` + 3 `.sh` + 1 `.xml` | 4 `.py` + 3 `.sh` + 1 `.xml` | 3 `.py` (shim, `dataset_tools`, launch) + 3 `.sh` + 1 `.xml` |
| hardcoded `/home/ubuntu/...` in config | 3 committed JSONs | **0** | 0 |
| copies of the COCO-80 label list | 4 | **2** (`nn_base.json`, `COCO_LABELS`) | 1 |
| committed per-model nn configs | 3 (1 for a blob that never existed) | **0** — 1 shared base + 1 `DEFAULT_MODEL` line | same |
| `.toy` programs | 9 | **5** (§3.5 decided and closed) | 4 (needs §1.2) |
| copies of the frame-rotation math | 7 inline + 1 helper | 7 inline + 1 helper | 1 |
| copies of the goal-issue protocol | 6 (2 already helpers) | 6 | 1 |
| copies of the Twinkle chime | 3 | 3 | 1 |
| copies of the ROS-env / `TB4_ROOT` preamble | 4 scripts | 4 scripts | 1 sourced file |
| training pipelines | 2 dirs, 13 files | 2 dirs, 13 files | 1 dir, 10 files |
| ball↔apple label fallback | 3 helpers, 2 mechanisms, 1 missing it | unchanged | 1 predicate |
| follow-verb vocabularies that must agree | 2 (`_FOLLOW_VERBS` vs `.toy` triggers) | 2 — but now **documented and test-pinned** | 1 |

Tier 1 was never going to be the size win — §1.6 (skipped) and Tiers 2–3 are.
What Tier 1 bought is in rows 7–11: one live bug fixed, three duplicated configs
and one dead file gone, the COCO label list from four copies to two, and a second
vocabulary drift found, documented in the source it affects, and pinned by a test
so it cannot silently widen.

The size and line-count savings from the remaining tiers are real but modest. The
actual win is rows 12–18: contracts that today are kept in sync by hand and by
comment ("*matching run_nav.sh and chime.sh*", "*Labels byte-identical to
`nn_yolov5mu.json`*", "*TB4_ROOT is the CHECKOUT itself … matching …*") each drop
to a single definition that cannot drift. Two of those eight are already down to
one; six remain, and every one of them is a place where this repo has already
been bitten — the comments recording past bugs are sitting right next to the
duplicated code that caused them.

