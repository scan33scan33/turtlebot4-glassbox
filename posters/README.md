# Maker Faire posters

Two 18"×24" (portrait) posters for the booth:

| poster | audience | what it covers |
|---|---|---|
| `poster-public` | everyone walking by | the robot's anatomy (OAK-D Lite, RPLIDAR, Pi 4, Create 3, phone UI), the see → plan → move loop in plain words, the three demo behaviors with their real trigger phrases, and the "3 green dots = go" health check |
| `poster-algorithms` | the algorithm-curious | the cost-map formula (soft exponential inflation), the lidar rotation matrix + quaternion→yaw, A* with the clearance penalty in `g`, the two-stage natural-language→program matcher and its scoring formula, the yolov8s vs yolov5mu offline AP eval, the hacks that make it work (p25 stereo depth, min(stereo, lidar@bearing) fusion, detections-as-obstacles, stall-as-success, the ignored backup_limit alarm, sensor self-heal), and "the same A* routes RPG characters / GPS / warehouse robots" |

Every formula, constant, trigger phrase and eval number is taken from the
source (`tb4_claude_nav.py`, `programs/*.toy`, `models/YOLOV8S_SWAP.md`,
`training/`, `objdet18/`) — if those change, update the posters.

Each poster exists as a self-contained `.html` (the source of truth — edit
this) and a print-ready `.pdf` rendered from it.

## Printing

The PDFs are exactly 18×24 in with no bleed margins — hand them straight to a
print shop. They also scale cleanly to A2/A1 (similar aspect) or tabloid for a
cheap test print; print "fit to page".

## Regenerating the PDFs after editing the HTML

Any recent Chromium/Chrome works (the page size is set via CSS `@page`):

```bash
cd posters
for p in poster-*.html; do
  chromium --headless --no-pdf-header-footer --print-to-pdf="${p%.html}.pdf" "$p"
done
```

Fonts are standard Linux system fonts (Liberation Sans / DejaVu Sans Mono), so
the render is reproducible on any stock Linux box; on macOS the substitutes
(Arial / a mono font) shift the layout slightly — check the output before printing.
