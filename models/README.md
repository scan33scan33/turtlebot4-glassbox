# models/ — OAK-D YOLO blobs + configs

This directory holds the **deployed** OAK-D blobs (RVC2, 416 input) and the
decode config they share. The config is committed; **the `.blob` files are
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
| `nn_base.json` | the shared decode config: COCO-80 label mappings + conf 0.3 / iou 0.5. The per-blob `model` block is injected at launch, so **one** config serves every detector | 1.7 KB | committed |
| `DEFAULT_MODEL` | the blob name to launch when `TB4_OAKD_MODEL` is unset | 22 B | committed |

### Adding a detector

There is no per-model config to write. Drop the `.blob` in `models/` (or publish
it as a Release asset and add its SHA-256 to `scripts/download_models.sh`), then
point the launch at it:

```bash
TB4_OAKD_MODEL=yolov8n_416_6shave bash run_oakd.sh
```

Both blobs must use the same COCO-80 label set and spellings as `nn_base.json`
(`sofa`, `tvmonitor`, `pottedplant`, `diningtable`, `motorbike`, `aeroplane`) —
`tb4_claude_nav.py` maps `class_id` through its own `COCO_LABELS`, so a
detector trained on different names needs that list updating too. A YOLOv8n
build is the obvious next candidate: `training/export_for_oak.py` produces the
blob.

### Fetching the blobs

`scripts/download_models.sh` pulls each blob from the Release and verifies its
SHA-256 before putting it in place, so a truncated download can't leave a
corrupt blob for depthai to choke on. It skips files that are already present
and valid, so it is safe to re-run; `--force` re-fetches regardless.

If you forked or renamed the repo, point it at your own Release:

```bash
TB4_MODELS_REPO=<owner>/<name> TB4_MODELS_TAG=models-v1 bash scripts/download_models.sh
```

`run_oakd.sh` and `services/tb4-oakd-run.sh` read `models/DEFAULT_MODEL` (or
`TB4_OAKD_MODEL`) and check that blob exists at startup, telling you to run this
script if it's missing — the same resolution `oakd_rgbd.launch.py` does, so the
preflight can't pass for a blob the launch isn't about to load.

#### Publishing a new blob

Create (or edit) a Release and attach the `.blob` files as assets, then update
the expected SHA-256 in `scripts/download_models.sh`:

```bash
sha256sum models/*.blob
```

### Paths

Nothing committed under `models/` contains an absolute path. At launch,
`oakd_rgbd.launch.py` resolves `TB4_ROOT` (default `~/Workspace/turtlebot4-glassbox`,
the stock TurtleBot 4 Pi location), reads `nn_base.json`, injects a `model` block
pointing at wherever the blob actually is, and writes the result to
`/tmp/tb4_nn_<model>.json`. It prints both paths so you can confirm at a glance
which detector is live:

```
[oakd] nn config : /tmp/tb4_nn_yolov5mu_416_5shave.json
[oakd] nn blob   : /home/ubuntu/Workspace/turtlebot4-glassbox/models/yolov5mu_416_5shave.blob
```

So a clone anywhere works with no edits:

```bash
TB4_ROOT=$HOME/projects/turtlebot4-glassbox bash run_oakd.sh
```

> This replaced three committed `nn_yolov*.json` files that were byte-identical
> apart from `model.model_name` — i.e. the 80 COCO labels were stored three
> times, and the one value that differed was hardcoded to
> `/home/ubuntu/Workspace/turtlebot4-glassbox/...`. A clone at any other path
> silently pointed depthai at a blob that did not exist. The COCO-80 list is now
> in two places (`nn_base.json` and `COCO_LABELS` in the navigator) instead of
> four; collapsing those two into one is tracked in
> `docs/simplification-plan.md`.

### Regenerating a blob

See `YOLOV8S_SWAP.md` for the `ultralytics` + `luxonis/tools` export steps
(`yolov8s.pt --imgsz 416` → ONNX → `blobconverter.from_onnx(..., version='2022.1',
optimizer_params=['--scale_values=[255,255,255]','--reverse_input_channels'])`).
Note that a depthai-model-zoo blob is **not** a drop-in substitute: those lack
the `/255` input scale and under-detect on-device.
