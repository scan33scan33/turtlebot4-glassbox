# Maker Faire slideshow

A self-contained HTML deck for booth talks and short walk-throughs.

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
| 1 | Title | “Tell it what to do. Watch it think.” |
| 2 | The pitch | Say a phrase → ranked behavior → live thinking on screen |
| 3 | Hardware | OAK-D · RPLIDAR · Pi 4 · Create 3 |
| 4 | Glass box | ML names objects; algorithms decide motion |
| 5 | See → Plan → Move | The whole loop on one slide |
| 6 | Vision | YOLO in one sentence; why it runs on the camera chip |
| 7 | Distance | p25 stereo + lidar@bearing → `min(...)` |
| 8 | Cost map | Live 120×120 grid, soft exponential inflation |
| 9 | A* | The algorithm they already know, on a live map |
| 10 | Pure pursuit | 0.5 m carrot — why driving looks smooth |
| 11 | `.toy` DSL | Named skills sequenced in a tiny language |
| 12 | NL → program | Verbatim trigger, else token-overlap score |
| 13 | Push the ball | Geometry of the crowd-favorite demo |
| 14 | Demo menu | Three phrases to try + “3 green dots = go” |
| 15 | Hacks | p25 depth, stall-as-success, self-heal, … |
| 16 | Takeaways | Five things to steal for your own project |
| 17 | Open source | Repo link, key files, come say hi |

Every constant, trigger phrase and eval number is taken from the source
(`tb4_claude_nav.py`, `programs/*.toy`, `models/YOLOV8S_SWAP.md`) — same rule
as the posters. If those change, update the deck.

## Presenting

```bash
# from anywhere — just open the file
xdg-open slides/maker-faire.html          # Linux
open slides/maker-faire.html              # macOS

# or serve it (useful on the booth laptop / phone cast)
python3 -m http.server 8000 --directory slides
# → http://localhost:8000/maker-faire.html
```

**Keys:** `→` / `Space` next · `←` previous · `Home` / `End` · `1`–`9` jump ·
`F` fullscreen. Click the right third of the slide to advance, left third to
go back. Swipe works on a phone/tablet. Deep-link with `#12`.

Print-to-PDF (one slide per page) works from Chromium’s print dialog —
“Background graphics” on, margins none, landscape or portrait both fine
(slides are fluid).

## Sibling materials

- `posters/poster-public.html` — 18×24" booth poster for everyone walking by
- `posters/poster-algorithms.html` — 18×24" for the algorithm-curious
- This deck is the **spoken** version of both: more narrative, less density,
  aimed between the two posters.
