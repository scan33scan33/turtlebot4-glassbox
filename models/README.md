# models/ — OAK-D YOLO blobs + configs

This directory holds the **deployed** OAK-D blobs (RVC2, 416 input) and their
decode configs. The `.json` configs are committed; **the `.blob` files are
not** — they are published as **GitHub Release assets** and fetched on demand:

```bash
bash scripts/download_models.sh
```

That keeps a clone small (the two blobs are ~71 MB) and, more importantly, keeps
this MIT-licensed repository from redistributing AGPL-3.0-derived Ultralytics
weights. See [NOTICE](../NOTICE) for the licensing detail — it matters if you
fork this commercially.

Source `.pt` weights are not committed either (`.gitignore: *.pt`) — ultralytics
re-downloads them on demand.

| file | role | size | tracked? |
|---|---|---|---|
| `yolov5mu_416_5shave.blob` | **deployed** YOLOv5mu (medium anchor-free), 5 shaves, FP16 | ~49 MB | Release asset |
| `yolov8s_416_fixed_6shave.blob` | **staged** YOLOv8s (swap candidate, see `YOLOV8S_SWAP.md`), 6 shaves, FP16 | ~22 MB | Release asset |
| `nn_yolov5mu.json` | decode config for the v5mu blob (COCO-80) | | committed |
| `nn_yolov8s.json` | decode config for the v8s blob (labels byte-identical to v5mu) | | committed |
| `nn_yolov8n.json` | template config for a **YOLOv8n** build (blob not yet exported — run `training/export_for_oak.py` to generate `yolov8n_416_6shave.blob` and update this JSON) | | committed |

### Fetching the blobs

`scripts/download_models.sh` pulls each blob from the Release and verifies its
SHA-256 before putting it in place, so a truncated download can't leave a
corrupt blob for depthai to choke on. It skips files that are already present
and valid, so it is safe to re-run; `--force` re-fetches regardless.

If you forked or renamed the repo, point it at your own Release:

```bash
TB4_MODELS_REPO=<owner>/<name> TB4_MODELS_TAG=models-v1 bash scripts/download_models.sh
```

`run_oakd.sh` and `services/tb4-oakd-run.sh` check for the deployed blob at
startup and tell you to run this script if it's missing.

#### Publishing a new blob

Create (or edit) a Release and attach the `.blob` files as assets, then update
the expected SHA-256 in `scripts/download_models.sh`:

```bash
sha256sum models/*.blob
```

### Paths

The JSONs contain an absolute `model_name`
(`/home/ubuntu/Workspace/turtlebot4-glassbox/models/*.blob`). That is the **stock
TurtleBot 4 Pi path** (`ubuntu` is the default Pi user). On the Pi it works
as-is. If you clone elsewhere, point `oakd_rgbd.launch.py` at your JSON (the
launch honors `TB4_ROOT`, e.g. `TB4_ROOT=$HOME/Workspace/turtlebot4-glassbox`) and edit
the JSON's `model_name` to match your checkout, or keep the Pi default.

### Regenerating a blob

See `YOLOV8S_SWAP.md` for the `ultralytics` + `luxonis/tools` export steps
(`yolov8s.pt --imgsz 416` → ONNX → `blobconverter.from_onnx(..., version='2022.1',
optimizer_params=['--scale_values=[255,255,255]','--reverse_input_channels'])`).
Note that a depthai-model-zoo blob is **not** a drop-in substitute: those lack
the `/255` input scale and under-detect on-device.
