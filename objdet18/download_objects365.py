#!/usr/bin/env python3
"""
download_objects365.py — build an Objects365 subset for our 18 classes WITHOUT
storing the full ~700 GB.

O365 has no per-class download; it's 51 train patch tarballs. Strategy:
  1. download + parse the train annotation JSON once -> the set of image
     filenames that contain >=1 of our classes, and their (remapped) boxes,
  2. for each patch tar: download -> extract ONLY the wanted images + write
     their YOLO labels -> DELETE the tar.
So peak disk = one patch (~15 GB) + the growing filtered subset, not 700 GB.

    python download_objects365.py --root dataset --max-patches 51        # full
    python download_objects365.py --root dataset --max-patches 6         # quick/cheap subset

NOTE: the KS3 host is slow/flaky outside China. See README for OpenDataLab /
HuggingFace mirrors if downloads stall. URLs mirror ultralytics' Objects365.yaml
— if a URL 404s, check that file for the current scheme.
"""
import argparse, json, os, subprocess, sys, tarfile
from taxonomy import O365_TO_ID, norm

BASE = "https://dorc.ks3-cn-beijing.ksyun.com/data-set/2020Objects365%E6%95%B0%E6%8D%AE%E9%9B%86/train/"
ANNO_TAR = "zhiyuan_objv2_train.tar.gz"       # contains zhiyuan_objv2_train.json
N_PATCHES = 51


def sh(cmd):
    print("+", cmd); subprocess.run(cmd, shell=True, check=True)


def curl(url, out):
    # -L follow redirects, -C - resume, --retry survive flaky host
    sh(f'curl -L -C - --retry 8 --retry-delay 5 -o "{out}" "{url}"')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="dataset")
    ap.add_argument("--split", default="train", choices=["train"],
                    help="only train is wired up (val: swap BASE + anno name)")
    ap.add_argument("--max-patches", type=int, default=N_PATCHES)
    ap.add_argument("--tmp", default="_o365_tmp")
    args = ap.parse_args()

    img_dir = os.path.join(args.root, "images", "train")
    lbl_dir = os.path.join(args.root, "labels", "train")
    os.makedirs(img_dir, exist_ok=True); os.makedirs(lbl_dir, exist_ok=True)
    os.makedirs(args.tmp, exist_ok=True)

    # 1) annotations -----------------------------------------------------------
    anno_json = os.path.join(args.tmp, "zhiyuan_objv2_train.json")
    if not os.path.exists(anno_json):
        tar = os.path.join(args.tmp, ANNO_TAR)
        curl(BASE + ANNO_TAR, tar)
        with tarfile.open(tar) as t:
            t.extractall(args.tmp)
        os.remove(tar)
    print("loading annotations (large, needs ~8 GB RAM)...")
    A = json.load(open(anno_json))

    # resolve our O365 names -> category_id via the JSON's own category list
    want_names = {norm(k): v for k, v in O365_TO_ID.items()}
    catid_to_id, matched = {}, []
    for c in A["categories"]:
        tid = want_names.get(norm(c["name"]))
        if tid is not None:
            catid_to_id[c["id"]] = tid; matched.append(c["name"])
    print("matched O365 classes:", sorted(matched))
    missing = set(O365_TO_ID) - {c["name"] for c in A["categories"] if norm(c["name"]) in want_names}
    if missing:
        print("  (not found in O365, skipped):", sorted(missing))

    # image_id -> (basename, W, H); collect boxes for wanted images
    img_meta = {im["id"]: (os.path.basename(im["file_name"]), im["width"], im["height"])
                for im in A["images"]}
    boxes = {}   # basename -> list[str yolo lines]
    for a in A["annotations"]:
        tid = catid_to_id.get(a["category_id"])
        if tid is None:
            continue
        bn, W, H = img_meta[a["image_id"]]
        x, y, w, h = a["bbox"]                         # absolute top-left x,y,w,h
        if w <= 1 or h <= 1:
            continue
        boxes.setdefault(bn, []).append(
            f"{tid} {(x + w/2)/W:.6f} {(y + h/2)/H:.6f} {w/W:.6f} {h/H:.6f}")
    wanted = set(boxes)
    print(f"{len(wanted)} O365 images contain our classes "
          f"({sum(len(v) for v in boxes.values())} boxes)")
    del A

    # 2) stream patches --------------------------------------------------------
    got = 0
    for i in range(args.max_patches):
        tar = os.path.join(args.tmp, f"patch{i}.tar.gz")
        try:
            curl(f"{BASE}patch{i}.tar.gz", tar)
        except subprocess.CalledProcessError:
            print(f"!! patch{i} failed to download, skipping"); continue
        try:
            with tarfile.open(tar) as t:
                for m in t:
                    if not m.isfile():
                        continue
                    bn = os.path.basename(m.name)
                    if bn not in wanted:
                        continue
                    data = t.extractfile(m).read()
                    with open(os.path.join(img_dir, "o365_" + bn), "wb") as f:
                        f.write(data)
                    with open(os.path.join(lbl_dir, "o365_" + os.path.splitext(bn)[0] + ".txt"), "w") as f:
                        f.write("\n".join(boxes[bn]))
                    got += 1
        except tarfile.TarError as e:
            print(f"!! patch{i} bad tar ({e}), skipping")
        finally:
            if os.path.exists(tar):
                os.remove(tar)
        print(f"  patch{i}: total kept so far = {got}")

    print(f"O365: wrote {got} images -> {args.root}")
    print("tip: rm -rf", args.tmp, "when done (keeps the annotation JSON cached otherwise)")


if __name__ == "__main__":
    main()
