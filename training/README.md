# training/ — custom YOLOv9-tiny for the OAK-D

Train a **nano detector on just your objects** (ball, person, …). A few-class
nano beats general models on your scene (Objects365 / OpenImages both *missed*
the ball that a focused model nails) and is smaller/faster on the VPU.

`yolov9t` is chosen over YOLOv7 because it trains cleanly in Ultralytics and is
**gen2/RVC2-deployable** (it decodes on your OAK-D-Lite; YOLO11 does not on the
legacy stack).

## Pipeline
```
frames  ->  autolabel.py  ->  (hand-fix)  ->  train.py  ->  export_for_oak.py  ->  blob
```

### 1. Gather frames
Any folder of camera images works. Easiest sources you already have:
```bash
# (a) the raw recorded camera frames on the Pi (replace <pi> with your Pi's
#     LAN address or mDNS name, e.g. turtlebot4.lan):
rsync -av ubuntu@<pi>:'/home/ubuntu/Workspace/turtlebot4-glassbox/datasets/drive_*/step_*.jpg' ./frames/

# (b) or pull individual frames out of a downloaded drive video:
python - <<'PY'
import cv2, os; os.makedirs('frames', exist_ok=True)
c=cv2.VideoCapture('drive_20260701_030149.mp4'); i=0   # <- your downloaded drive video
while True:
    ok,f=c.read()
    if not ok: break
    if i%5==0: cv2.imwrite(f'frames/{i:05d}.jpg', f)   # every 5th frame
    i+=1
PY
```
Aim for a few hundred to a few thousand varied frames (different distances,
angles, lighting, and *all the ball colours*).

### 2. Auto-label with a strong teacher
```bash
python autolabel.py --images 'frames/*.jpg' --out dataset --teacher yolov8x.pt --imgsz 640
```
Uses a big COCO model at 640 (catches balls your 416 on-device model misses) and
writes YOLO labels into `dataset/`. Edit `TARGET` + `CLASS_MAP` in `autolabel.py`
(and `data.yaml`) to change classes.

### 3. Review (important for the hard cases)
Pseudo-labels inherit the teacher's mistakes. Upload `dataset/` to
[Roboflow](https://roboflow.com) or [CVAT](https://cvat.ai), fix/complete the
boxes (especially the pink ball / close-ups), export back as YOLOv8 format.
Skip this only for a quick first pass.

### 4. Train (GPU: Colab/Kaggle)
```bash
python train.py --data data.yaml --epochs 120 --imgsz 416 --batch 16
# -> runs/detect/oak_nano/weights/best.pt
```

### 5. Export to an OAK blob
```bash
python export_for_oak.py --weights runs/detect/oak_nano/weights/best.pt --imgsz 416
```
Follow the printed `luxonis/tools` command to get `.blob` + `nn_config.json`,
copy them to the Pi, point `oakd_rgbd.launch.py` at the JSON, restart.

## Notes
- **Keep resolution consistent:** train, export, and the OAK preview all at 416.
- **Don't make colours into classes** — keep `ball` and let `_detect_color`
  tag the colour at runtime (as it does now).
- After swapping the model, update label checks in `tb4_claude_nav.py`
  (`'sports ball'`, `'apple'`) to your new `'ball'` class.
- Requires `ultralytics` (already in the `yolo-ws` env). Training wants a GPU.
