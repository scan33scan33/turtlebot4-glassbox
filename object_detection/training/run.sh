#!/usr/bin/env bash
# Unified training runner for a GPU instance: deps -> dataset download -> train.
#
# Usage:
#   DATASET=coco        ./run.sh    # COCO-only 18-class indoor subset
#   DATASET=coco+o365   ./run.sh    # Full COCO + Objects365 18-class subset
#   DATASET=few_class   ./run.sh    # Train on existing custom dataset (e.g. from autolabel.py)
set -euo pipefail
cd "$(dirname "$0")"

DATASET="${DATASET:-coco}"
MAX_PATCHES="${MAX_PATCHES:-10}"    # For Objects365 patches (1..51)
MODEL="${MODEL:-yolov9t.pt}"
IMGSZ="${IMGSZ:-416}"               # 416 matches OAK preview
EPOCHS="${EPOCHS:-100}"
BATCH="${BATCH:--1}"                # -1 = auto batch

echo "== checking / installing deps =="
pip install -q ultralytics

if [ "$DATASET" = "coco" ]; then
    echo "== downloading COCO 18-class subset =="
    python datasets/download_coco.py --split val   --root dataset
    python datasets/download_coco.py --split train --root dataset
    YAML="datasets/coco_o365_18.yaml"
    NAME="coco_18"
elif [ "$DATASET" = "coco+o365" ]; then
    echo "== downloading COCO + Objects365 (${MAX_PATCHES} patches) =="
    python datasets/download_coco.py --split val   --root dataset
    python datasets/download_coco.py --split train --root dataset
    python datasets/download_objects365.py --root dataset --max-patches "${MAX_PATCHES}"
    YAML="datasets/coco_o365_18.yaml"
    NAME="coco_o365_18"
elif [ "$DATASET" = "few_class" ]; then
    YAML="datasets/few_class.yaml"
    NAME="few_class"
else
    echo "Unknown DATASET: $DATASET (expected coco | coco+o365 | few_class)" >&2
    exit 1
fi

echo "== training model =="
python train.py --data "$YAML" --model "$MODEL" --imgsz "$IMGSZ" --epochs "$EPOCHS" --batch "$BATCH" --name "$NAME"

echo "== export to ONNX =="
python export_for_oak.py --weights "runs/detect/$NAME/weights/best.pt" --imgsz "$IMGSZ"
echo "== done! convert ONNX to blob with luxonis/tools for OAK-D deploy =="
