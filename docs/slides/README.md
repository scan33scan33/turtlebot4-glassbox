# Maker Faire slideshow

A self-contained HTML deck for booth talks and short walk-throughs. The stack
was built with Claude Code on [Arena.ai](https://arena.ai).

| file | what |
|---|---|
| `maker-faire.html` | **17 slides** — open in any browser, hit `F` for fullscreen |

## Audience

People who **know basic algorithms and computers**, but **don’t live in ML**:
makers, CS students, curious parents, engineers from other fields. The deck
assumes “I’ve heard of A* / pathfinding / a neural net” and builds from there.
It deliberately does **not** assume ROS, depth cameras, or deep-learning math.

## Arc (≈ 8–12 min talk, or click-through at the booth)

The narrative has five acts: **high level** (1–2) → **what it can do** (3) →
**technology** (4–9) → **deep dive + difficulty** (10–16) → **end** (17).

| # | slide | point |
|---|---|---|
| 1 | Title | The project in one sentence |
| 2 | High level | Command → installed script → visible execution |
| 3 | What it can do | Wall push · follow me · push to a marked goal |
| 4 | Hardware | OAK-D · RPLIDAR · Pi 4 · Create 3 |
| 5 | Glass-box architecture | Generated code is readable; runtime decisions are visible |
| 6 | VLA analogy | Claude: goal + run evidence → next candidate script |
| 7 | `.toy` DSL | The readable artifact Claude writes; `0.40` = one 40 cm push |
| 8 | Runtime match | Verbatim trigger, else token-overlap score |
| 9 | See → Plan → Move | The installed script selects a skill; local algorithms execute it |
| 10 | Vision | YOLO names and boxes objects; it does not steer |
| 11 | Sensor fusion | p25 stereo + lidar@bearing → one fast `min(...)` |
| 12 | Cost map | Live 120×120 grid with a soft clearance band |
| 13 | A* | Animated game map: search wave → best route → moving sprite |
| 14 | Pure pursuit | A virtual moving carrot keeps motion on the safe path |
| 15 | Deep dive | Stop–look–push, with a diagram and short-segment rationale |
| 16 | Difficulty | Five field problem→reason→fix stories, including the ASUS router |
| 17 | End | Repo, Claude Code + Arena.ai credit, and come say hi |

The generate → physical test → reward → revision story reflects the project’s
actual development workflow. Every runtime constant, trigger phrase and eval
number is taken from the source (`tb4_claude_nav.py`, `programs/*.toy`,
`models/YOLOV8S_SWAP.md`) — same rule as the posters. If those change, update
the deck.

## Presenting

```bash
# from anywhere — just open the file
xdg-open docs/slides/maker-faire.html      # Linux
open docs/slides/maker-faire.html          # macOS

# or serve it (useful on the booth laptop / phone cast)
python3 -m http.server 8000 --directory docs/slides
# → http://localhost:8000/maker-faire.html
```

**Keys:** `→` / `Space` next · `←` previous · `Home` / `End` · `1`–`9` jump ·
`F` fullscreen. Click the right third of the slide to advance, left third to
go back. Swipe works on a phone/tablet. Deep-link with `#12`.

**Auto-advance / kiosk mode:** append `?refresh=20s` to advance every 20
seconds, for example `maker-faire.html?refresh=20s#1`. Bare seconds such as
`?refresh=20` also work. It loops from the last slide to the first, resets the
timer after manual navigation, and pauses while the tab is hidden.

Print-to-PDF (one slide per page) works from Chromium’s print dialog —
“Background graphics” on, margins none, landscape or portrait both fine
(slides are fluid).

## Sibling materials

- `../posters/poster-public.html` — 18×24" booth poster for everyone walking by
- `../posters/poster-algorithms.html` — 18×24" for the algorithm-curious
- This deck is the **spoken** version of both: more narrative, less density,
  aimed between the two posters.
