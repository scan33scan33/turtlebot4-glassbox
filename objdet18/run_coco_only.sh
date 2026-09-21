#!/usr/bin/env bash
# COCO-only baseline: deps -> filtered COCO (bulk zip) -> train. FiftyOne-free.
#   bash run_coco_only.sh                 # download + train
#   DOWNLOAD_ONLY=1 bash run_coco_only.sh # just download (e.g. on a cheap CPU pod)
set -euo pipefail
cd "$(dirname "$0")"

MODEL="${MODEL:-yolov9t.pt}"
IMGSZ="${IMGSZ:-640}"
EPOCHS="${EPOCHS:-100}"
BATCH="${BATCH:--1}"          # -1 = auto-fit GPU memory

echo "== deps =="
pip install -q ultralytics

echo "== COCO val + train (bulk zip, filtered to the 18 classes) =="
python download_coco.py --split val   --root dataset      # small/fast first
python download_coco.py --split train --root dataset
echo "train: $(ls dataset/images/train 2>/dev/null | wc -l)   val: $(ls dataset/images/val 2>/dev/null | wc -l)"

if [ -n "${DOWNLOAD_ONLY:-}" ]; then
  echo "== DOWNLOAD_ONLY set — dataset is on disk, skipping training =="
  exit 0
fi

echo "== train =="
python train.py --data data.yaml --model "$MODEL" --imgsz "$IMGSZ" \
                --epochs "$EPOCHS" --batch "$BATCH" --name coco_only_18
echo "== done: runs/detect/coco_only_18/weights/best.pt =="
