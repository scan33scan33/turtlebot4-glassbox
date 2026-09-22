# Training Custom YOLO Detectors for OAK-D

Pipeline to train and export lightweight YOLO models (`yolov9t`, `yolov8s`, etc.) tuned for indoor TurtleBot4 navigation and deployed on the OAK-D VPU (Myriad X / RVC2).

## 1. Datasets Available

Under `datasets/`:
- **`few_class.yaml`**: Minimal target classes (e.g. `ball`, `person`). Use `autolabel.py` to pseudo-label frames recorded by the robot (`datasets/drive_*/step_*.jpg`) using a high-res teacher model.
- **`coco_o365_18.yaml`**: 18 indoor classes (chair, bottle, dog, sports ball, person, table, etc.) remapped from COCO and Objects365 subsets. Includes automated downloaders:
  - `download_coco.py`: Fast bulk download and extraction of only the 18 classes.
  - `download_objects365.py`: Stream-extracts matching images per patch to avoid storing the full 700GB dataset.

## 2. Quickstart Training (on GPU workstation / Colab)

Run the unified runner:
```bash
# 18-class indoor subset from COCO:
DATASET=coco ./run.sh

# Full COCO + Objects365 training:
DATASET=coco+o365 MAX_PATCHES=10 ./run.sh

# Train on custom few-class data:
DATASET=few_class ./run.sh
```

Or run `train.py` manually:
```bash
python train.py --data datasets/coco_o365_18.yaml --model yolov9t.pt --imgsz 416 --epochs 100
```
*(Always train or export at `--imgsz 416` to match the OAK-D RGB preview).*

## 3. Export to OAK-D Blob

1. Export trained weights to ONNX:
   ```bash
   python export_for_oak.py --weights runs/detect/coco_18/weights/best.pt --imgsz 416
   ```
2. Convert ONNX to an RVC2 `.blob` using Luxonis tools:
   ```bash
   pip install 'git+https://github.com/luxonis/tools.git'
   tools runs/detect/coco_18/weights/best.onnx --imgsz 416 --use-rvc2
   # Or via browser: https://tools.luxonis.com
   ```
3. Copy the compiled `.blob` into `object_detection/` on the robot and update `object_detection/DEFAULT_MODEL` (or set `TB4_OAKD_MODEL`).
