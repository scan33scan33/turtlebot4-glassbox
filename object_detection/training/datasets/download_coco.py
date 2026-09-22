#!/usr/bin/env python3
"""
download_coco.py — build a COCO-2017 subset for our 18 classes, FiftyOne-free.

Downloads the bulk train/val image zip (fast + resumable) and stream-extracts
ONLY the images that contain our classes, writing YOLO labels. Avoids FiftyOne's
per-image download (slow + dies on a single flaky request).

    python download_coco.py --split train --root dataset
    python download_coco.py --split val   --root dataset --max-samples 500
"""
import argparse, json, os, subprocess, zipfile
from taxonomy import COCO_TO_ID

BASE = "http://images.cocodataset.org/"


def curl(url, out):
    # -C - resumes a partial file; --retry-all-errors retries on mid-transfer
    # connection resets/closes (not just timeouts), so a flaky S3 drop won't kill it.
    subprocess.run(
        f'curl -L -C - --retry 15 --retry-delay 5 --retry-all-errors '
        f'--connect-timeout 30 -o "{out}" "{url}"',
        shell=True, check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", choices=["train", "val"], required=True)
    ap.add_argument("--root", default="dataset")
    ap.add_argument("--max-samples", type=int, default=None)
    ap.add_argument("--tmp", default="_coco_tmp")
    args = ap.parse_args()
    coco = "train2017" if args.split == "train" else "val2017"

    img_dir = os.path.join(args.root, "images", args.split)
    lbl_dir = os.path.join(args.root, "labels", args.split)
    for d in (img_dir, lbl_dir, args.tmp):
        os.makedirs(d, exist_ok=True)

    # 1) annotations -> per-image YOLO boxes for our classes ------------------
    ann = os.path.join(args.tmp, f"instances_{coco}.json")
    if not os.path.exists(ann):
        z = os.path.join(args.tmp, "annotations_trainval2017.zip")
        if not os.path.exists(z):
            curl(BASE + "annotations/annotations_trainval2017.zip", z)
        with zipfile.ZipFile(z) as zf, zf.open(f"annotations/instances_{coco}.json") as s, \
                open(ann, "wb") as out:
            out.write(s.read())          # context-managed: the ~1.5 GB write is
                                         # flushed before json.load reads it back
    with open(ann) as fh:
        A = json.load(fh)
    catid = {c["id"]: COCO_TO_ID[c["name"]] for c in A["categories"] if c["name"] in COCO_TO_ID}
    meta = {im["id"]: (im["file_name"], im["width"], im["height"]) for im in A["images"]}
    boxes = {}
    for a in A["annotations"]:
        tid = catid.get(a["category_id"])
        if tid is None:
            continue
        fn, W, H = meta[a["image_id"]]
        x, y, w, h = a["bbox"]
        if w <= 1 or h <= 1:
            continue
        boxes.setdefault(fn, []).append(
            f"{tid} {(x + w/2)/W:.6f} {(y + h/2)/H:.6f} {w/W:.6f} {h/H:.6f}")
    wanted = list(boxes)
    if args.max_samples:
        wanted = wanted[:args.max_samples]
    wanted = set(wanted)
    print(f"{coco}: {len(wanted)} images contain our classes; fetching image zip...")

    # 2) bulk zip -> stream-extract only wanted images ------------------------
    z = os.path.join(args.tmp, f"{coco}.zip")
    curl(f"{BASE}zips/{coco}.zip", z)
    got = 0
    with zipfile.ZipFile(z) as zf:
        for m in zf.namelist():
            bn = os.path.basename(m)
            if bn not in wanted:
                continue
            with zf.open(m) as s:
                open(os.path.join(img_dir, "coco_" + bn), "wb").write(s.read())
            open(os.path.join(lbl_dir, "coco_" + os.path.splitext(bn)[0] + ".txt"), "w").write(
                "\n".join(boxes[bn]))
            got += 1
    os.remove(z)
    print(f"COCO {args.split}: wrote {got} images -> {args.root}")


if __name__ == "__main__":
    main()
