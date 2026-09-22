# Staged: yolov8s detector swap (config-only) — see issue #4

Everything needed to A/B **yolov8s** against the deployed **yolov5mu** is in place.
Nothing is wired in yet — the launch still defaults to v5mu (`models/DEFAULT_MODEL`).
Flipping is one environment variable; there is no source edit to make.

## Artifacts
| file | role | tracked? |
|---|---|---|
| `yolov8s_416_fixed_6shave.blob` | compiled RVC2 blob, 416 input, 6 shaves. Same `\x7felf` format as the deployed v5mu blob. | Release asset |
| `nn_base.json` | the shared decode config (COCO-80 labels, conf 0.3, iou 0.5 — the values v5mu already uses). The per-blob `model` block is injected at launch, so **one** config serves both detectors and `tb4_claude_nav.py` needs **zero changes** for the swap. | committed |
| `DEFAULT_MODEL` | the blob name the launch uses when `TB4_OAKD_MODEL` is unset. Currently `yolov5mu_416_5shave`. | committed |

## Deploy (on the Pi)
```bash
cd ~/Workspace/turtlebot4-glassbox
git pull                              # brings nn_base.json + DEFAULT_MODEL
bash scripts/download_models.sh       # brings both blobs (Release assets, not committed)
```
Then run the OAK pipeline with the override — **no source edit, no line number
to get wrong**:
```bash
TB4_OAKD_MODEL=yolov8s_416_fixed_6shave bash run_oakd.sh
```
Restart nav afterwards (`bash run_nav.sh`, or `sudo systemctl restart tb4-nav`).

Under systemd, put the override in the unit instead:
```bash
sudo systemctl edit tb4-oakd        # drop-in, survives package updates
#   [Service]
#   Environment=TB4_OAKD_MODEL=yolov8s_416_fixed_6shave
sudo systemctl restart tb4-oakd
```

To make v8s the default permanently, change the one line in `models/DEFAULT_MODEL`
and commit that.

## Rollback
Drop the override (or the systemd drop-in) and restart — the launch falls back to
`models/DEFAULT_MODEL`, which is still v5mu. Both blobs stay on disk, so the A/B
is a restart, not a redeploy.

**Check the swap actually took:** `run_oakd.sh` prints the resolved pair at
startup, so one glance confirms which detector is live:
```
[oakd] nn config : /tmp/tb4_nn_yolov8s_416_fixed_6shave.json
[oakd] nn blob   : /home/ubuntu/Workspace/turtlebot4-glassbox/models/yolov8s_416_fixed_6shave.blob
```


## Watch during the A/B
- **Ball recall** — the reason for the swap. Offline: yolov8s AP 0.83 vs v5mu 0.60 (issue #4). Confirm it survives the on-device ÷255 + FP16 quantization.
- **FPS / X_LINK health** — v8s blob is **6-shave** vs v5mu's 5-shave; with `nn.i_num_inference_threads: 2` that's 6×2=12 shaves (OAK-D-Lite has 16, OK). If X_LINK errors or FPS drops, set `i_num_inference_threads: 1` first, then consider lowering `rgb.i_fps` (see [[oakd-fps-preference]]) before giving up.
- **Person** — should tie (offline 0.80 vs 0.79). Don't regress `follow` ("follow me").

## Regenerate the blob from scratch (if ever needed)
```bash
# 1) ONNX at deploy size (reproducible):
python -c "from ultralytics import YOLO; YOLO('models/yolov8s.pt').export(format='onnx', imgsz=416, opset=12, simplify=True)"
# 2) ONNX -> RVC2 blob + json via luxonis tools (does the YOLO head surgery for on-device decode):
pip install 'git+https://github.com/luxonis/tools.git'
tools models/yolov8s.onnx --imgsz 416 --use-rvc2       # or web UI: https://tools.luxonis.com
```
Note: a raw ultralytics ONNX output is `(1,84,3549)` — the luxonis tools reshape the
head into the separate tensors DepthAI's on-device YOLO decoder expects. A mis-compiled
head is exactly what caused the earlier bogus "v8s ~15%" result.
