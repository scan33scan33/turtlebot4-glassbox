# objdet18 — COCO + Objects365 mix, 18 classes

Trains a detector on **COCO-2017 ∪ Objects365** filtered to 18 classes:

> chair, book, bottle, dining table, bowl, handbag, backpack, potted plant,
> couch, cell phone, suitcase, vase, sports ball, tv, dog, teddy bear,
> refrigerator, bed

More data + Objects365's diversity → a stronger, more general model than COCO
alone. Everything is remapped to a shared 0–17 label space (`taxonomy.py`).

## One command (on a RunPod GPU box)
```bash
git clone <this repo> && cd turtlebot4-glassbox/objdet18
MAX_PATCHES=8 EPOCHS=80 ./run_all.sh          # quick/cheap first run
# or a full run:
MAX_PATCHES=51 ./run_all.sh
```
Output: `runs/detect/coco_o365_18/weights/best.pt`.

## ⚠️ Read before you spin up RunPod — the Objects365 reality
Objects365 has **no per-class download**. It ships as **51 train patch tarballs,
~700 GB total**. Our downloader streams **patch → keep only your-class images →
delete patch**, so *peak disk* stays ~one patch, **but you still transfer every
patch over the network.** Plan accordingly:

| lever | guidance |
|---|---|
| **Disk** | ~80–120 GB volume is plenty (filtered subset + 1 patch at a time). |
| **Network/time** | Full O365 = ~700 GB download. The KS3 host is **slow/flaky outside China** — a full pull can take **many hours**. **Start with `MAX_PATCHES=6–10`** to validate the pipeline and get a usable subset before committing to all 51. |
| **RAM** | The train annotation JSON is ~1.5 GB → needs **~8 GB RAM** to parse. |
| **Cost** | You pay RunPod per hour, incl. the long download. A GPU is wasted while downloading — consider downloading on a **cheap CPU pod**, saving `dataset/` to a network volume, then attaching a GPU pod just for `train.py`. |

**Faster O365 mirrors** if KS3 crawls: [OpenDataLab](https://opendatalab.com/OpenDataLab/Objects365) (`pip install openxlab`) or a HuggingFace mirror — swap the `BASE`/patch logic in `download_objects365.py`.

## Files
| file | role |
|---|---|
| `taxonomy.py` | 18-class space + COCO/O365 name→id maps (imperfect matches noted) |
| `download_coco.py` | FiftyOne: only COCO images with our classes → YOLO |
| `download_objects365.py` | Stream+filter O365 patches → YOLO |
| `data.yaml` | 18-class dataset config |
| `train.py` | Ultralytics training (default `yolov9t`) |
| `run_all.sh` | env → download → train |

## Notes on the mappings (see `taxonomy.py`)
- **dining table ← O365 "Desk"** (O365 has no dining-table class; approximate).
- **suitcase ← "Luggage"**, **teddy bear ← "Stuffed Toy"**, **tv ← "Monitor/TV"**.
- **sports ball ← every O365 ball type** (Basketball, Soccer, Baseball, …) — good
  for a robust ball detector. Colour is still handled at runtime by
  `_detect_color`, so don't split balls by colour here.
- O365 names are resolved **by name against the annotation JSON**, and any that
  don't match are printed and skipped (so listing extra ball types is safe).

## Deploying to the OAK-D
Train (or export) at **416** to match the camera preview, then:
```bash
python ../training/export_for_oak.py --weights runs/detect/coco_o365_18/weights/best.pt --imgsz 416
```
Follow the printed `luxonis/tools --use-rvc2` step for the blob, and update the
label names in `tb4_claude_nav.py` (`'sports ball'` etc. already match).
