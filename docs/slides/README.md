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
| 2 | Visitor experience | Command → installed script → visible execution |
| 3 | VLA analogy | Claude: goal + run evidence → next candidate script |
| 4 | Glass box | Generated code is readable; runtime decisions are visible |
| 5 | Hardware | OAK-D · RPLIDAR · Pi 4 · Create 3 |
| 6 | `.toy` DSL | The readable artifact Claude writes; `0.40` = one 40 cm push |
| 7 | Runtime match | Verbatim trigger, else token-overlap score |
| 8 | See → Plan → Move | The installed script selects a skill; local algorithms execute it |
| 9 | Vision | YOLO names and boxes objects; it does not steer |
| 10 | Sensor fusion | p25 stereo + lidar@bearing → one fast `min(...)` |
| 11 | Cost map | Live 120×120 grid with a soft clearance band |
| 12 | A* | Animated game map: search wave → best route → moving sprite |
| 13 | Pure pursuit | A virtual moving carrot keeps motion on the safe path |
| 14 | Push the ball | Stop–look–push, with a diagram and short-segment rationale |
| 15 | Demo menu | Wall push · follow me · push to a marked goal |
| 16 | Field hacks | Five distinct problem→reason→fix stories, including the ASUS router |
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
