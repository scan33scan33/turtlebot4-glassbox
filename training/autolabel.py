#!/usr/bin/env python3
"""
autolabel.py — bootstrap a YOLO dataset by running a strong *teacher* model over
your unlabelled robot frames.

You have thousands of recorded camera frames (datasets/drive_*/step_*.jpg) but
no bounding boxes. This runs a big COCO model at high resolution, keeps the
detections for the classes you care about, and writes YOLO-format labels — a
pseudo-labelled dataset you can train on immediately.

    python autolabel.py --images '/path/to/frames/*.jpg' --out dataset

Classes are mapped teacher->target via CLASS_MAP below. Keep TARGET in sync with
data.yaml (same order = same class ids).

IMPORTANT — pseudo-label caveat: the student inherits the teacher's *misses*.
A frame where the teacher failed to see the ball becomes a "no ball here"
negative, which actively teaches the student the wrong thing. Two mitigations,
both used here / recommended:
  * teacher = yolov8x at imgsz 640 (much stronger than your on-device 416 model,
    so it catches the hard balls your deployed model misses),
  * then HAND-FIX the hard frames in Roboflow/CVAT before the final train.
"""
import argparse, glob, os, random, shutil
from ultralytics import YOLO

# target classes — MUST match data.yaml order (index = class id) ---------------
TARGET = ["ball", "person"]

# teacher(COCO) label -> your target label -------------------------------------
CLASS_MAP = {
    "sports ball": "ball",
    "person":      "person",
    # add more if you widen TARGET, e.g. "chair": "chair", "tv": "tv",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="glob of source images, e.g. 'frames/*.jpg'")
    ap.add_argument("--out", default="dataset", help="output dataset dir")
    ap.add_argument("--teacher", default="yolov8x.pt", help="strong COCO model used to label")
    ap.add_argument("--conf", type=float, default=0.35, help="min teacher confidence to keep a box")
    ap.add_argument("--imgsz", type=int, default=640, help="teacher inference size (bigger = better labels)")
    ap.add_argument("--val", type=float, default=0.15, help="val split fraction")
    ap.add_argument("--keep-empty", action="store_true",
                    help="also keep frames with no target objects (as negatives)")
    args = ap.parse_args()

    tid = {name: i for i, name in enumerate(TARGET)}
    files = sorted(glob.glob(args.images))
    assert files, f"no images matched {args.images!r}"
    model = YOLO(args.teacher)

    for split in ("train", "val"):
        os.makedirs(f"{args.out}/images/{split}", exist_ok=True)
        os.makedirs(f"{args.out}/labels/{split}", exist_ok=True)

    random.seed(0); random.shuffle(files)
    n_val = int(len(files) * args.val)
    counts = {c: 0 for c in TARGET}; kept = 0

    for i, f in enumerate(files):
        split = "val" if i < n_val else "train"
        r = model(f, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
        lines = []
        for b in r.boxes:
            tgt = CLASS_MAP.get(model.names[int(b.cls)])
            if tgt is None:
                continue
            x, y, w, h = b.xywhn[0].tolist()          # normalised cx, cy, w, h
            lines.append(f"{tid[tgt]} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")
            counts[tgt] += 1
        if not lines and not args.keep_empty:
            continue
        base = f"{i:06d}"
        shutil.copy(f, f"{args.out}/images/{split}/{base}.jpg")
        with open(f"{args.out}/labels/{split}/{base}.txt", "w") as fh:
            fh.write("\n".join(lines))
        kept += 1

    print(f"scanned {len(files)} frames -> kept {kept} labelled images in {args.out}/")
    print("boxes per class:", counts)
    print("Next: eyeball a few labels (esp. the ball), fix the hard ones in "
          "Roboflow/CVAT, then run train.py")


if __name__ == "__main__":
    main()
