# object_detection/ — OAK-D YOLO blobs, configs & training

This directory contains everything related to on-device object detection for the OAK-D Lite camera:
1. **Deployed Models:** Deployed neural network configurations and pointers for depthai.
2. **`training/`:** Offline PyTorch/Ultralytics dataset preparation, training, and export pipeline.

The `.blob` compiled weights are **not committed** to Git to keep clones lightweight; they are GitHub Release assets fetched on demand.

## Quickstart (fetch deployed blobs)

```bash
bash object_detection/download_models.sh
```

This downloads:
- `yolov5mu_416_5shave.blob` (COCO-80, 5-shave) — **the default**
- `yolov8s_416_fixed_6shave.blob` (COCO-80, 6-shave) — staged replacement (see `YOLOV8S_SWAP.md`)

## Switching Detectors

The default detector is configured in `DEFAULT_MODEL`:
```
yolov5mu_416_5shave
```

To run a different detector without changing Git files, pass `TB4_OAKD_MODEL`:
```bash
TB4_OAKD_MODEL=yolov8s_416_fixed_6shave bash object_detection/run_oakd.sh
```

## Config Generation at Launch

`nn_base.json` defines the shared decode parameters (the COCO-80 label list, anchors, and thresholds).
At launch, `oakd_rgbd.launch.py` reads `nn_base.json`, injects the absolute path to the active blob (`$TB4_ROOT/object_detection/<model>.blob`), and generates `/tmp/tb4_nn_<model>.json` for the depthai driver. Nothing committed carries an absolute path.

## Custom Training

See [`training/README.md`](training/README.md) for how to:
- Pseudo-label robot camera frames using `training/autolabel.py`.
- Download and train an 18-class indoor robot detector using `training/run.sh`.
- Export trained `.pt` weights to an ONNX file and compile an RVC2 `.blob` using `training/export_for_oak.py`.
