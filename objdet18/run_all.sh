#!/usr/bin/env bash
# One-shot pipeline for a RunPod GPU box: env -> COCO -> Objects365 -> train.
# Tune with env vars, e.g.:  MAX_PATCHES=8 EPOCHS=80 MODEL=yolov9t.pt ./run_all.sh
set -euo pipefail
cd "$(dirname "$0")"

MAX_PATCHES="${MAX_PATCHES:-51}"      # 51 = full O365 train (~700GB download!); start small
MODEL="${MODEL:-yolov9t.pt}"
IMGSZ="${IMGSZ:-640}"
EPOCHS="${EPOCHS:-100}"
BATCH="${BATCH:--1}"                  # -1 = auto batch

echo "== deps =="
pip install -q ultralytics fiftyone

echo "== COCO (only images with our 18 classes) =="
python download_coco.py --split train --root dataset
python download_coco.py --split val   --root dataset

echo "== Objects365 (streaming ${MAX_PATCHES} patches, filtered) =="
python download_objects365.py --root dataset --max-patches "${MAX_PATCHES}"

echo "== dataset summary =="
echo "train imgs: $(ls dataset/images/train 2>/dev/null | wc -l)   val imgs: $(ls dataset/images/val 2>/dev/null | wc -l)"

echo "== train =="
python train.py --data data.yaml --model "${MODEL}" --imgsz "${IMGSZ}" --epochs "${EPOCHS}" --batch "${BATCH}"

echo "== done: runs/detect/coco_o365_18/weights/best.pt =="
