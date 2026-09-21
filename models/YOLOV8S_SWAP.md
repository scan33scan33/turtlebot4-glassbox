# Staged: yolov8s detector swap (config-only) — see issue #4

Everything needed to A/B **yolov8s** against the deployed **yolov5mu** is in place.
Nothing is wired in yet — the launch still points at v5mu. Flip one line when ready.

## Artifacts (all in `models/`, git-tracked)
| file | role |
|---|---|
| `yolov8s_416_fixed_6shave.blob` | compiled RVC2 blob, 416 input, 6 shaves. Same `\x7felf` format as the deployed v5mu blob. |
| `nn_yolov8s.json` | decode config. **Labels byte-identical to `nn_yolov5mu.json`** (COCO-80, same spellings: sofa/tvmonitor/pottedplant/diningtable/motorbike/aeroplane), so `tb4_claude_nav.py` needs **zero changes**. conf 0.3, iou 0.5 — matched to v5mu. |

## Deploy (on the Pi)
```bash
cd /home/ubuntu/Workspace/turtlebot4-glassbox
git pull                              # brings nn_yolov8s.json
bash scripts/download_models.sh       # brings the blob (Release asset, not committed)
```
Then edit `oakd_rgbd.launch.py` line 26:
```python
# from:
'nn.i_nn_config_path': '.../models/nn_yolov5mu.json',
# to:
'nn.i_nn_config_path': '/home/ubuntu/Workspace/turtlebot4-glassbox/models/nn_yolov8s.json',
```
Restart OAK + nav (`run_oakd.sh`, then the nav service).

## Rollback
Revert line 26 to `nn_yolov5mu.json`, restart. (Both blobs stay on disk.)

## Watch during the A/B
- **Ball recall** — the reason for the swap. Offline: yolov8s AP 0.83 vs v5mu 0.60 (issue #4). Confirm it survives the on-device ÷255 + FP16 quantization.
- **FPS / X_LINK health** — v8s blob is **6-shave** vs v5mu's 5-shave; with `nn.i_num_inference_threads: 2` that's 6×2=12 shaves (OAK-D-Lite has 16, OK). If X_LINK errors or FPS drops, set `i_num_inference_threads: 1` first, then consider lowering `rgb.i_fps` (see [[oakd-fps-preference]]) before giving up.
- **Person** — should tie (offline 0.80 vs 0.79). Don't regress follow_human.

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
