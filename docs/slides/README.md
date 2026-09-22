# Maker Faire slideshow

A self-contained HTML deck for booth talks and short walk-throughs. The stack
was built with Claude Code on [Arena.Ai](https://arena.ai).

| file | what |
|---|---|
| `maker-faire.html` | **17 slides** — open in any browser, hit `F` for fullscreen |

## Audience

People who **know basic algorithms and computers**, but **don’t live in ML**:
makers, CS students, curious parents, engineers from other fields. The deck
assumes “I’ve heard of A* / pathfinding / a neural net” and builds from there.
It deliberately does **not** assume ROS, depth cameras, or deep-learning math.

## Arc (≈ 8–12 min talk, or click-through at the booth)

| # | slide | point |
|---|---|---|
| 1 | Title | Claude generates scripts; the robot evaluates and runs them |
| 2 | The pitch | Build/evaluate loop vs. local demo/runtime loop |
| 3 | Hardware | OAK-D · RPLIDAR · Pi 4 · Create 3 |
| 4 | Glass box | Generated code is readable; runtime decisions are visible |
| 5 | See → Plan → Move | The local runtime loop — no Claude call while driving |
| 6 | Vision | YOLO in one sentence; why it runs on the camera chip |
| 7 | Distance | Simple, fast fusion: p25 stereo + lidar@bearing → `min(...)` |
| 8 | Cost map | Live 120×120 grid, soft exponential inflation |
| 9 | A* | Animated game map: search wave → best route → moving sprite |
| 10 | Pure pursuit | Intuitive moving-carrot explanation: stay on the safe path |
| 11 | `.toy` DSL | Claude composes bounded skills; `0.40` means one 40 cm push segment |
| 12 | Runtime match | Verbatim trigger, else token-overlap score |
| 13 | VLA analogy | Claude: goal + run evidence → next candidate script |
| 14 | Push the ball | Stop–look–push, with a diagram and the reason for short segments |
| 15 | Demo menu | Wall push · follow me · push to a marked goal |
| 16 | Field hacks | Five problem→reason→fix stories, including the ASUS router run |
| 17 | Open source | Repo, Claude Code + Arena.Ai credit, and come say hi |

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

Print-to-PDF (one slide per page) works from Chromium’s print dialog —
“Background graphics” on, margins none, landscape or portrait both fine
(slides are fluid).

## Sibling materials

- `../posters/poster-public.html` — 18×24" booth poster for everyone walking by
- `../posters/poster-algorithms.html` — 18×24" for the algorithm-curious
- This deck is the **spoken** version of both: more narrative, less density,
  aimed between the two posters.
