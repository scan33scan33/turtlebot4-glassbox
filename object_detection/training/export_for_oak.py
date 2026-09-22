#!/usr/bin/env python3
"""
export_for_oak.py — turn your trained best.pt into an OAK-D (RVC2) blob.

    python export_for_oak.py --weights runs/detect/oak_nano/weights/best.pt --imgsz 416

Step 1 (here): export the ultralytics model to ONNX at the deployment size.
Step 2 (printed): convert ONNX -> RVC2 blob + decode JSON with luxonis/tools.
"""
import argparse
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--imgsz", type=int, default=416, help="MUST match your OAK preview + training")
    args = ap.parse_args()

    path = YOLO(args.weights).export(format="onnx", imgsz=args.imgsz, opset=12, simplify=True)
    print(f"\nONNX written: {path}\n")
    print(f"""Now convert to an RVC2 blob (yolov9t is a gen2-supported head, so on-device
decoding will work — unlike YOLO11 on the legacy path):

  pip install 'git+https://github.com/luxonis/tools.git'
  tools {path} --imgsz {args.imgsz} --use-rvc2
  # (or the web UI: https://tools.luxonis.com  ->  select the .onnx / .pt)

That yields <best>.blob + an nn_config JSON. On the Pi:
  1. copy both next to your existing blobs (they're gitignored),
  2. point oakd_rgbd.launch.py's nn.i_nn_config_path at the new JSON,
  3. make sure the JSON's class list == data.yaml names, in the same order,
  4. restart the OAK (object_detection/run_oakd.sh) + nav.

Then in tb4_claude_nav.py the detections just flow through as usual — your
label strings become 'ball' / 'person' instead of COCO 'sports ball' / 'person'
(update any name checks like `'sports ball'` / `'apple'` -> `'ball'`).""")


if __name__ == "__main__":
    main()
